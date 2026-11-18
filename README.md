# UGS Studio — WorkBench 200 Series CNC & 3DP Controller

A lightweight, self-contained Universal G-Code Sender (UGS) web UI, live 3D toolpath visualizer, and DXF-to-GCode converter built for the **3DP WorkBench 200 Series (Duet Ethernet 1.0 / RepRapFirmware 1.19.2)**.

## Features
- **Auto-Connect & Live Telemetry (`COM6` @ `115200` baud):** Continuous `M408 S2`, `M119`, and `M122` polling for Digital Readout (`X`, `Y`, `Z`, `E0/E1`), endstop state, BLTouch probe controls, Bed/Volcano extruder temperatures, MCU temperature, and 24V bus voltage.
- **Interactive Jog Keypad & Keyboard Jogging:** 3×3 XY directional pad (including diagonal jogs), `Z+`/`Z-`, `E+`/`E-`, selectable step sizes (`0.01` to `100 mm`), and modal-safe `M120`/`M121` relative jogging.
- **Unlocked Negative Z (`Z < 0`) Travel & Probing:** Automatically sends `M208 X-1000 Y-1000 Z-510 S1` and `M564 S0 H0` so the probe and toolhead can descend below `Z = 0` after zeroing.
- **Native DXF-to-GCode Converter (`dxf_converter.py`):**
  - Supports `SPLINE` (De Boor B-spline / NURBS evaluation), `LWPOLYLINE` (including arc bulges), `POLYLINE`, `LINE`, `ARC`, and `CIRCLE` entities.
  - Deduplicates overlapping segments, chains connected segments into continuous contours, shifts origin to bottom-left `(X0, Y0)`, and orders contours by nearest-neighbor traversal.
- **Configurable Feedrates & Z Heights:** Adjust **Cut Feed (`G1 XY`)**, **Rapid Feed (`G0`)**, **Z Feed (max `180 mm/min`)**, **Work Z**, and **Safe Z** directly in the UI and apply them to any loaded G-code or uploaded `.dxf` file.
- **3D Toolpath & Live Toolhead Visualizer:** Interactive 3D HTML5 Canvas with Orbit, Pan, Zoom, camera presets (`3D Iso`, `Top XY`, `Front XZ`, `Side YZ`, `Fit Path`), and live toolhead tracking.

## Quick Start

1. Install `pyserial`:
   ```bash
   pip install pyserial
   ```
2. Double-click **`Launch CNC Studio.bat`** or run:
   ```bash
   python server.py
   ```
3. Open **`http://127.0.0.1:8765`** in your browser.
