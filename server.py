import json
import os
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import serial
import serial.tools.list_ports
from dxf_converter import convert_dxf_to_gcode

PORT_HOST = "127.0.0.1"
PORT_HTTP = 8765

BLOCKING_PREFIXES = ("G28", "G29", "G30", "G32", "G4", "M400", "M109", "M190")


class CNCController:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.connected = False
        self.port_name = ""
        self.baudrate = 115200

        self.status = "Disconnected"
        self.machine_name = "WorkBench 200 Series - Volcanos"
        self.firmware = "RepRapFirmware for Duet Ethernet 1.19.2"
        self.pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E0": 0.0, "E1": 0.0}
        self.machine_pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.axes_homed = [0, 0, 0]
        self.endstops = {"X": "unknown", "Y": "unknown", "Z": "unknown", "Z_probe": "unknown"}
        self.temps = {
            "bed": {"current": 0.0, "active": 0.0},
            "t0": {"current": 0.0, "active": 0.0},
            "t1": {"current": 0.0, "active": 0.0},
            "mcu": 0.0,
        }
        self.v_in = 0.0
        self.speed_factor = 100.0
        self.fan_percent = [0.0, 0.0, 0.0]
        self.babystep = 0.0

        self.console_log = deque(maxlen=400)
        self.log_seq = 0

        self.job_lines = []
        self.job_filename = ""
        self.job_running = False
        self.job_paused = False
        self.job_index = 0
        self.job_total = 0
        self.job_last_resume_time = None
        self.job_accum_elapsed = 0.0
        self.job_current_line = ""

        self._stop_threads = False
        self._poll_thread = threading.Thread(target=self._poll_worker, daemon=True)
        self._poll_thread.start()
        self._job_thread = threading.Thread(target=self._job_worker, daemon=True)
        self._job_thread.start()

    def add_log(self, direction, text, is_poll=False):
        if is_poll:
            return
        with self.log_lock:
            self.log_seq += 1
            ts = time.strftime("%H:%M:%S")
            self.console_log.append({
                "id": self.log_seq,
                "time": ts,
                "dir": direction,
                "text": text.strip()
            })

    def list_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
                "manufacturer": p.manufacturer or "",
                "product": p.product or "",
            })
        return ports

    def connect(self, port="COM6", baudrate=115200):
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=int(baudrate),
                    timeout=1.2,
                    write_timeout=1.2
                )
                self.ser.dtr = True
                self.ser.rts = True
                time.sleep(0.15)
                self.ser.reset_input_buffer()
                self.connected = True
                self.port_name = port
                self.baudrate = int(baudrate)
                self.status = "Idle"
                self.add_log("SYS", f"Connected to {port} @ {baudrate} baud")
            except Exception as e:
                self.connected = False
                self.status = "Disconnected"
                self.add_log("ERR", f"Failed to connect to {port}: {e}")
                return False, str(e)

        self.send_command("M115", is_poll=False)
        self.send_command("M550", is_poll=False)
        # Allow Z probe and Z axis to move below Z=0 (override config.g M208 Z0 S1 & M564 S1)
        self.send_command("M208 X-1000 Y-1000 Z-510 S1", is_poll=False)
        self.send_command("M564 S0 H0", is_poll=False)
        self.send_command("M408 S2", is_poll=True)
        self.send_command("M119", is_poll=True)
        self.send_command("M122", is_poll=True)
        return True, f"Connected to {port}"

    def disconnect(self):
        self.job_running = False
        self.job_paused = False
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self.connected = False
            self.status = "Disconnected"
            self.add_log("SYS", f"Disconnected from {self.port_name}")
        return True

    def emergency_stop(self):
        self.job_running = False
        self.job_paused = False
        self.job_current_line = "E-STOP HALTED"
        self.add_log("ERR", "EMERGENCY STOP TRIGGERED (M112 + M999)")
        self.send_command("M112", is_poll=False, timeout=1.0)
        self.send_command("M999", is_poll=False, timeout=1.5)
        self.send_command("G90", is_poll=True, timeout=1.0)
        return True

    def send_command(self, cmd, is_poll=False, timeout=1.8):
        cmd_clean = cmd.strip()
        if not cmd_clean:
            return ""
        if not self.connected or not self.ser:
            if not is_poll:
                self.add_log("ERR", "Not connected to controller")
            return "Error: Not connected"

        upper_cmd = cmd_clean.upper()
        if any(upper_cmd.startswith(prefix) for prefix in BLOCKING_PREFIXES):
            timeout = max(timeout, 30.0)

        with self.lock:
            try:
                if self.ser.in_waiting > 0:
                    self.ser.reset_input_buffer()

                if not is_poll:
                    self.add_log("TX", cmd_clean, is_poll=False)
                self.ser.write((cmd_clean + "\n").encode("utf-8", errors="ignore"))
                self.ser.flush()

                lines = []
                deadline = time.time() + timeout
                while time.time() < deadline:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline().decode("utf-8", errors="replace").strip()
                        if raw:
                            lines.append(raw)
                            if raw == "ok" or raw.startswith("ok ") or raw.startswith("Error:"):
                                break
                    else:
                        time.sleep(0.012)

                time.sleep(0.008)
                while self.ser.in_waiting > 0:
                    raw = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if raw:
                        lines.append(raw)

                resp = "\n".join(lines)
                self._parse_response(cmd_clean, resp)
                if resp and not is_poll:
                    self.add_log("RX", resp, is_poll=False)
                return resp
            except Exception as e:
                self.connected = False
                self.status = "Disconnected"
                self.add_log("ERR", f"Serial error: {e}")
                return f"Error: {e}"

    def _parse_response(self, cmd, resp):
        if not resp:
            return

        for line in resp.splitlines():
            line_s = line.strip()
            if line_s.startswith("{") and line_s.endswith("}"):
                try:
                    data = json.loads(line_s)
                    st = data.get("status", "I")
                    status_map = {
                        "I": "Idle",
                        "P": "Running",
                        "S": "Paused",
                        "C": "Configuring",
                        "B": "Busy",
                        "D": "Decelerating",
                        "H": "Halted",
                        "F": "Flashing",
                    }
                    self.status = status_map.get(st, st)
                    coords = data.get("coords", {})
                    if "xyz" in coords and len(coords["xyz"]) >= 3:
                        self.pos["X"] = float(coords["xyz"][0])
                        self.pos["Y"] = float(coords["xyz"][1])
                        self.pos["Z"] = float(coords["xyz"][2])
                    if "machine" in coords and len(coords["machine"]) >= 3:
                        self.machine_pos["X"] = float(coords["machine"][0])
                        self.machine_pos["Y"] = float(coords["machine"][1])
                        self.machine_pos["Z"] = float(coords["machine"][2])
                    if "extr" in coords:
                        if len(coords["extr"]) >= 1:
                            self.pos["E0"] = float(coords["extr"][0])
                        if len(coords["extr"]) >= 2:
                            self.pos["E1"] = float(coords["extr"][1])
                    if "axesHomed" in coords:
                        self.axes_homed = coords["axesHomed"]

                    params = data.get("params", {})
                    if "speedFactor" in params:
                        self.speed_factor = float(params["speedFactor"])
                    if "fanPercent" in params:
                        self.fan_percent = [float(x) for x in params["fanPercent"][:3]]
                    if "babystep" in params:
                        self.babystep = float(params["babystep"])

                    temps = data.get("temps", {})
                    bed = temps.get("bed", {})
                    if "current" in bed:
                        self.temps["bed"]["current"] = float(bed["current"])
                    if "active" in bed:
                        self.temps["bed"]["active"] = float(bed["active"])

                    heads = temps.get("heads", {})
                    cur_h = heads.get("current", [])
                    act_h = heads.get("active", [])
                    if len(cur_h) >= 1:
                        self.temps["t0"]["current"] = float(cur_h[0])
                    if len(cur_h) >= 2:
                        self.temps["t1"]["current"] = float(cur_h[1])
                    if len(act_h) >= 1:
                        self.temps["t0"]["active"] = float(act_h[0])
                    if len(act_h) >= 2:
                        self.temps["t1"]["active"] = float(act_h[1])

                    for ex in temps.get("extra", []):
                        if ex.get("name") == "MCU":
                            self.temps["mcu"] = float(ex.get("temp", 0.0))

                    if "vin" in data and isinstance(data["vin"], dict):
                        self.v_in = float(data["vin"].get("current", 0.0))
                except Exception:
                    pass

        if "X:" in resp and "Y:" in resp and "Z:" in resp:
            mx = re.search(r"X:\s*([-\d.]+)\s+Y:\s*([-\d.]+)\s+Z:\s*([-\d.]+)", resp)
            if mx:
                self.pos["X"] = float(mx.group(1))
                self.pos["Y"] = float(mx.group(2))
                self.pos["Z"] = float(mx.group(3))

        if "RepRap name:" in resp:
            m = re.search(r"RepRap name:\s*(.+)", resp)
            if m:
                self.machine_name = m.group(1).strip()

        if "FIRMWARE_NAME:" in resp:
            self.firmware = resp.splitlines()[0].strip()

        if "Endstops -" in resp:
            for axis_key, label in [("X", "X"), ("Y", "Y"), ("Z", "Z"), ("Z_probe", "Z probe")]:
                m = re.search(rf"{label}:\s*([^,\n]+)", resp)
                if m:
                    self.endstops[axis_key] = m.group(1).strip()

        if "Supply voltage:" in resp:
            mv = re.search(r"Supply voltage:.*?current\s+([\d.]+)", resp)
            if mv:
                self.v_in = float(mv.group(1))

    def _poll_worker(self):
        tick = 0
        while not self._stop_threads:
            if self.connected and not self.job_running:
                self.send_command("M408 S2", is_poll=True, timeout=0.8)
                if tick % 8 == 0:
                    self.send_command("M119", is_poll=True, timeout=0.8)
                if tick % 25 == 0:
                    self.send_command("M122", is_poll=True, timeout=1.0)
                tick += 1
            time.sleep(0.45)

    def load_job(self, filename, gcode_text):
        raw_lines = gcode_text.splitlines()
        cleaned = []
        for idx, line in enumerate(raw_lines):
            code_part = line.split(";")[0].strip()
            if code_part:
                cleaned.append({"line_no": idx + 1, "code": code_part, "raw": line.strip()})
        self.job_lines = cleaned
        self.job_filename = filename or "untitled.gcode"
        self.job_total = len(cleaned)
        self.job_index = 0
        self.job_running = False
        self.job_paused = False
        self.job_accum_elapsed = 0.0
        self.job_last_resume_time = None
        self.job_current_line = ""
        self.add_log("SYS", f"Loaded '{self.job_filename}' ({self.job_total} executable G-code lines)")

    def start_job(self):
        if not self.connected:
            return False, "Controller is not connected"
        if not self.job_lines:
            return False, "No G-code job loaded"
        if self.job_running and self.job_paused:
            self.job_paused = False
            self.job_last_resume_time = time.time()
            self.add_log("SYS", "Resumed G-code job")
            return True, "Job resumed"
        self.job_index = 0
        self.job_accum_elapsed = 0.0
        self.job_last_resume_time = time.time()
        self.job_paused = False
        self.job_running = True
        self.add_log("SYS", f"Started streaming '{self.job_filename}' ({self.job_total} lines)")
        return True, "Job started"

    def pause_job(self):
        if self.job_running:
            if not self.job_paused:
                self.job_paused = True
                if self.job_last_resume_time:
                    self.job_accum_elapsed += time.time() - self.job_last_resume_time
                    self.job_last_resume_time = None
                self.add_log("SYS", f"Paused G-code job at line {self.job_index}/{self.job_total}")
                return True, "Paused"
            else:
                self.job_paused = False
                self.job_last_resume_time = time.time()
                self.add_log("SYS", f"Resumed G-code job at line {self.job_index}/{self.job_total}")
                return True, "Resumed"
        return False, "No active job"

    def stop_job(self):
        if self.job_running or self.job_paused:
            if not self.job_paused and self.job_last_resume_time:
                self.job_accum_elapsed += time.time() - self.job_last_resume_time
            self.job_last_resume_time = None
            self.job_running = False
            self.job_paused = False
            self.send_command("G90", is_poll=True, timeout=1.0)
            self.add_log("SYS", "Stopped G-code job and restored G90 absolute mode")
            return True, "Job stopped"
        return False, "No active job"

    def step_job(self):
        if not self.connected:
            return False, "Not connected"
        if not self.job_lines or self.job_index >= self.job_total:
            return False, "No remaining lines to step"
        item = self.job_lines[self.job_index]
        self.job_current_line = f"N{item['line_no']}: {item['code']}"
        self.send_command(item["code"], is_poll=False, timeout=5.0)
        self.job_index += 1
        self.send_command("M408 S2", is_poll=True, timeout=0.6)
        return True, f"Stepped line {self.job_index}/{self.job_total}"

    def get_job_elapsed(self):
        total = self.job_accum_elapsed
        if self.job_running and not self.job_paused and self.job_last_resume_time:
            total += time.time() - self.job_last_resume_time
        return total

    def _job_worker(self):
        last_poll = 0.0
        while not self._stop_threads:
            if self.job_running and not self.job_paused:
                if self.job_index < self.job_total:
                    item = self.job_lines[self.job_index]
                    self.job_current_line = f"N{item['line_no']}: {item['code']}"
                    resp = self.send_command(item["code"], is_poll=False, timeout=10.0)
                    if resp.startswith("Error: Not connected") or resp.startswith("Error: Serial"):
                        self.job_running = False
                        self.add_log("ERR", "Job aborted due to connection loss")
                        continue
                    self.job_index += 1
                    now = time.time()
                    if now - last_poll > 0.35:
                        self.send_command("M408 S2", is_poll=True, timeout=0.6)
                        last_poll = now
                else:
                    self.send_command("M400", is_poll=True, timeout=30.0)
                    self.send_command("M408 S2", is_poll=True, timeout=0.8)
                    if self.job_last_resume_time:
                        self.job_accum_elapsed += time.time() - self.job_last_resume_time
                        self.job_last_resume_time = None
                    self.job_running = False
                    self.job_paused = False
                    self.add_log("SYS", f"Job '{self.job_filename}' completed in {self.get_job_elapsed():.1f}s!")
            else:
                time.sleep(0.05)

    def get_state_snapshot(self, since_log=0):
        with self.log_lock:
            new_logs = [entry for entry in self.console_log if entry["id"] > since_log]
        return {
            "connected": self.connected,
            "port": self.port_name,
            "baudrate": self.baudrate,
            "status": self.status if self.connected else "Disconnected",
            "machine_name": self.machine_name,
            "firmware": self.firmware,
            "pos": self.pos,
            "axes_homed": self.axes_homed,
            "endstops": self.endstops,
            "temps": self.temps,
            "v_in": self.v_in,
            "speed_factor": self.speed_factor,
            "fan_percent": self.fan_percent,
            "babystep": self.babystep,
            "job": {
                "filename": self.job_filename,
                "running": self.job_running,
                "paused": self.job_paused,
                "index": self.job_index,
                "total": self.job_total,
                "elapsed": round(self.get_job_elapsed(), 1),
                "current_line": self.job_current_line,
            },
            "logs": new_logs,
        }


controller = CNCController()


class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _send_json(self, obj, status=200):
        payload = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            ui_path = os.path.join(os.path.dirname(__file__), "index.html")
            if os.path.exists(ui_path):
                with open(ui_path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404, "index.html not found")
            return

        if parsed.path == "/api/state":
            qs = parse_qs(parsed.query)
            since = int(qs.get("since", ["0"])[0])
            self._send_json(controller.get_state_snapshot(since_log=since))
            return

        if parsed.path == "/api/ports":
            self._send_json({"ports": controller.list_ports()})
            return

        if parsed.path.startswith("/api/preset/"):
            key = parsed.path.split("/api/preset/")[1]
            preset_map = {
                "sleeve": "Sleeve DXF - Sketch 4.gcode",
                "cf30_front": "CF30_front_in_template.gcode",
                "cf30_back": "CF30_back_in_template.gcode",
            }
            fname = preset_map.get(key, "Sleeve DXF - Sketch 4.gcode")
            fpath = os.path.join(os.path.dirname(__file__), fname)
            if os.path.exists(fpath):
                with open(fpath, "r", encoding="utf-8") as f:
                    gcode = f.read()
                controller.load_job(fname, gcode)
                self._send_json({"ok": True, "filename": fname, "gcode": gcode})
            else:
                self._send_json({"ok": False, "message": f"{fname} not found"}, status=404)
            return

        self.send_error(404, "Not found")

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        try:
            payload = json.loads(body) if body else {}
        except Exception:
            payload = {}

        if parsed.path == "/api/connect":
            port = payload.get("port", "COM6")
            baud = payload.get("baudrate", 115200)
            ok, msg = controller.connect(port, baud)
            self._send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/api/disconnect":
            controller.disconnect()
            self._send_json({"ok": True, "message": "Disconnected"})
            return

        if parsed.path == "/api/estop":
            controller.emergency_stop()
            self._send_json({"ok": True, "message": "Emergency stop executed"})
            return

        if parsed.path == "/api/command":
            cmd = payload.get("command", "")
            responses = []
            for line in cmd.splitlines():
                line_s = line.split(";")[0].strip()
                if line_s:
                    r = controller.send_command(line_s, is_poll=False, timeout=3.0)
                    responses.append(r)
            controller.send_command("M408 S2", is_poll=True, timeout=0.6)
            self._send_json({"ok": True, "response": "\n".join(responses)})
            return

        if parsed.path == "/api/jog":
            feed = float(payload.get("feedrate", 600.0))
            if "axes" in payload and isinstance(payload["axes"], dict):
                parts = []
                for ax, d in payload["axes"].items():
                    parts.append(f"{ax.upper()}{float(d):.3f}")
                feed = min(feed, 12000.0)
                move_cmd = f"G1 {' '.join(parts)} F{feed:.0f}"
            else:
                axis = payload.get("axis", "Z").upper()
                dist = float(payload.get("distance", 1.0))
                if axis == "Z" and feed > 180.0:
                    feed = 180.0
                elif axis == "E" and feed > 300.0:
                    feed = 300.0
                else:
                    feed = min(feed, 12000.0)
                move_cmd = f"G1 {axis}{dist:.3f} F{feed:.0f}"

            for c in ("M120", "G91", move_cmd, "M121"):
                controller.send_command(c, is_poll=(c in ("M120", "M121")), timeout=5.0)
            controller.send_command("M408 S2", is_poll=True, timeout=0.6)
            self._send_json({"ok": True})
            return

        if parsed.path == "/api/job/load":
            filename = payload.get("filename", "program.gcode")
            gcode = payload.get("gcode", "")
            controller.load_job(filename, gcode)
            self._send_json({"ok": True, "total": controller.job_total})
            return

        if parsed.path == "/api/dxf/convert":
            filename = payload.get("filename", "drawing.dxf")
            dxf_text = payload.get("dxf", "")
            origin_mode = payload.get("origin_mode", "min")
            xy_cut_feed = int(float(payload.get("xy_cut_feed", 150)))
            xy_travel_feed = int(float(payload.get("xy_travel_feed", 3600)))
            z_feed = min(180, int(float(payload.get("z_feed", 150))))
            safe_z = float(payload.get("safe_z", 5.0))
            cut_z = float(payload.get("cut_z", 0.0))
            gcode, meta = convert_dxf_to_gcode(
                dxf_text,
                source_name=filename,
                origin_mode=origin_mode,
                xy_cut_feed=xy_cut_feed,
                xy_travel_feed=xy_travel_feed,
                z_feed=z_feed,
                safe_z=safe_z,
                cut_z=cut_z,
            )
            out_name = os.path.splitext(filename)[0] + ".gcode"
            controller.load_job(out_name, gcode)
            self._send_json({"ok": True, "filename": out_name, "gcode": gcode, "meta": meta})
            return

        if parsed.path == "/api/job/start":
            ok, msg = controller.start_job()
            self._send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/api/job/pause":
            ok, msg = controller.pause_job()
            self._send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/api/job/stop":
            ok, msg = controller.stop_job()
            self._send_json({"ok": ok, "message": msg})
            return

        if parsed.path == "/api/job/step":
            ok, msg = controller.step_job()
            self._send_json({"ok": ok, "message": msg})
            return

        self.send_error(404, "Endpoint not found")


if __name__ == "__main__":
    ports = [p["device"] for p in controller.list_ports()]
    if "COM6" in ports:
        controller.connect("COM6", 115200)
    elif ports:
        controller.connect(ports[0], 115200)

    server = ThreadingHTTPServer((PORT_HOST, PORT_HTTP), RequestHandler)
    print(f"UGS / WorkBench CNC Studio running at http://{PORT_HOST}:{PORT_HTTP}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
