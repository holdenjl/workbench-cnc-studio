import math
import os


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bulge_to_arc_points(p1, p2, bulge, num_steps=8):
    if abs(bulge) < 1e-6:
        return [p1, p2]
    theta = 4.0 * math.atan(bulge)
    chord = dist2d(p1, p2)
    if chord < 1e-9:
        return [p1]
    radius = chord / (2.0 * abs(math.sin(theta / 2.0)))
    mx = 0.5 * (p1[0] + p2[0])
    my = 0.5 * (p1[1] + p2[1])
    dx = (p2[0] - p1[0]) / chord
    dy = (p2[1] - p1[1]) / chord
    nx, ny = -dy, dx
    d = math.sqrt(max(0.0, radius * radius - (chord * 0.5) ** 2))
    sign = 1.0 if bulge > 0 else -1.0
    if abs(theta) > math.pi:
        sign = -sign
    cx = mx + sign * d * nx
    cy = my + sign * d * ny
    start_ang = math.atan2(p1[1] - cy, p1[0] - cx)
    pts = []
    for k in range(num_steps + 1):
        ang = start_ang + theta * (k / num_steps)
        pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    return pts


def eval_bspline_point(t, degree, ctrl_pts, knots, weights=None):
    n = len(ctrl_pts)
    if not weights or len(weights) != n:
        weights = [1.0] * n
    pts_h = [[ctrl_pts[i][0] * weights[i], ctrl_pts[i][1] * weights[i], weights[i]] for i in range(n)]
    t_min = knots[degree]
    t_max = knots[n]
    t = max(t_min, min(t_max, t))
    k = degree
    for i in range(degree, n):
        if knots[i] <= t <= knots[i + 1]:
            k = i
            if t < knots[i + 1]:
                break
    d = [pts_h[j + k - degree][:] for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            denom = knots[j + 1 + k - r] - knots[j + k - degree]
            alpha = (t - knots[j + k - degree]) / denom if abs(denom) > 1e-12 else 0.0
            for c in range(3):
                d[j][c] = (1.0 - alpha) * d[j - 1][c] + alpha * d[j][c]
    w = d[degree][2]
    if abs(w) < 1e-12:
        return (d[degree][0], d[degree][1])
    return (d[degree][0] / w, d[degree][1] / w)


def sample_spline_entity(props):
    deg = next((int(v) for c, v in props if c == 71), 3)
    knots = [float(v) for c, v in props if c == 40]
    weights = [float(v) for c, v in props if c == 41]
    ctrl_x = [float(v) for c, v in props if c == 10]
    ctrl_y = [float(v) for c, v in props if c == 20]
    fit_x = [float(v) for c, v in props if c == 11]
    fit_y = [float(v) for c, v in props if c == 21]

    ctrl_pts = list(zip(ctrl_x, ctrl_y))
    if len(ctrl_pts) >= deg + 1 and len(knots) >= len(ctrl_pts) + deg + 1:
        # Estimate arc length with coarse pass
        t0, t1 = knots[deg], knots[len(ctrl_pts)]
        coarse = [eval_bspline_point(t0 + (t1 - t0) * k / 16.0, deg, ctrl_pts, knots, weights) for k in range(17)]
        est_len = sum(dist2d(coarse[i], coarse[i + 1]) for i in range(len(coarse) - 1))
        n_steps = max(16, min(80, int(math.ceil(est_len / 5.0))))
        return [eval_bspline_point(t0 + (t1 - t0) * k / n_steps, deg, ctrl_pts, knots, weights) for k in range(n_steps + 1)]
    elif len(fit_x) >= 2 and len(fit_x) == len(fit_y):
        return list(zip(fit_x, fit_y))
    elif len(ctrl_pts) >= 2:
        return ctrl_pts
    return []


def parse_dxf_segments(dxf_text):
    raw = [l.strip() for l in dxf_text.splitlines()]
    pairs = [(int(raw[i]), raw[i + 1]) for i in range(0, len(raw) - 1, 2) if raw[i].lstrip("-").isdigit()]

    entities = []
    sec = None
    cur_ent = None
    poly_verts = None
    poly_flag = 0

    for code, val in pairs:
        if code == 0 and val == "SECTION":
            sec = "WAIT"
            continue
        if sec == "WAIT" and code == 2:
            sec = val
            continue
        if code == 0 and val == "ENDSEC":
            if cur_ent:
                entities.append(cur_ent)
                cur_ent = None
            sec = None
            continue
        if sec == "ENTITIES":
            if code == 0:
                if cur_ent:
                    if cur_ent["type"] == "POLYLINE":
                        poly_verts = []
                        poly_flag = int(next((v for c, v in cur_ent["props"] if c == 70), "0"))
                    elif cur_ent["type"] == "VERTEX" and poly_verts is not None:
                        vx = float(next((v for c, v in cur_ent["props"] if c == 10), "0"))
                        vy = float(next((v for c, v in cur_ent["props"] if c == 20), "0"))
                        vb = float(next((v for c, v in cur_ent["props"] if c == 42), "0"))
                        poly_verts.append([vx, vy, vb])
                    elif cur_ent["type"] == "SEQEND" and poly_verts is not None:
                        entities.append({"type": "_OLD_POLYLINE", "props": [], "verts": poly_verts, "flag": poly_flag})
                        poly_verts = None
                    else:
                        entities.append(cur_ent)
                cur_ent = {"type": val, "props": []}
            elif cur_ent is not None:
                cur_ent["props"].append((code, val))

    raw_segments = []
    for e in entities:
        t = e["type"]
        props = e.get("props", [])
        if t == "LINE":
            x1 = float(next(v for c, v in props if c == 10))
            y1 = float(next(v for c, v in props if c == 20))
            x2 = float(next(v for c, v in props if c == 11))
            y2 = float(next(v for c, v in props if c == 21))
            if dist2d((x1, y1), (x2, y2)) > 1e-5:
                raw_segments.append([(x1, y1), (x2, y2)])
        elif t == "SPLINE":
            pts = sample_spline_entity(props)
            if len(pts) >= 2:
                raw_segments.append(pts)
        elif t == "ARC":
            cx = float(next(v for c, v in props if c == 10))
            cy = float(next(v for c, v in props if c == 20))
            r = float(next(v for c, v in props if c == 40))
            a1 = float(next(v for c, v in props if c == 50))
            a2 = float(next(v for c, v in props if c == 51))
            if a2 < a1:
                a2 += 360.0
            span = a2 - a1
            n = max(8, int(math.ceil(span / 8.0)))
            pts = []
            for k in range(n + 1):
                ang = math.radians(a1 + span * (k / n))
                pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
            raw_segments.append(pts)
        elif t == "CIRCLE":
            cx = float(next(v for c, v in props if c == 10))
            cy = float(next(v for c, v in props if c == 20))
            r = float(next(v for c, v in props if c == 40))
            n = 48
            pts = []
            for k in range(n + 1):
                ang = 2.0 * math.pi * (k / n)
                pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
            raw_segments.append(pts)
        elif t in ("LWPOLYLINE", "_OLD_POLYLINE"):
            if t == "_OLD_POLYLINE":
                flag = e["flag"]
                verts = e["verts"]
            else:
                flag = int(next((v for c, v in props if c == 70), "0"))
                verts = []
                cur_v = None
                for c, v in props:
                    if c == 10:
                        if cur_v:
                            verts.append(cur_v)
                        cur_v = [float(v), 0.0, 0.0]
                    elif c == 20 and cur_v:
                        cur_v[1] = float(v)
                    elif c == 42 and cur_v:
                        cur_v[2] = float(v)
                if cur_v:
                    verts.append(cur_v)
            if flag & 1 and len(verts) > 1:
                verts.append([verts[0][0], verts[0][1], 0.0])
            pts = []
            for k in range(len(verts) - 1):
                p1 = (verts[k][0], verts[k][1])
                p2 = (verts[k + 1][0], verts[k + 1][1])
                b = verts[k][2]
                sub = bulge_to_arc_points(p1, p2, b)
                if pts:
                    pts.extend(sub[1:])
                else:
                    pts.extend(sub)
            if len(pts) >= 2:
                raw_segments.append(pts)

    unique_segments = []
    for seg in raw_segments:
        p0, p1 = seg[0], seg[-1]
        length = sum(dist2d(seg[i], seg[i + 1]) for i in range(len(seg) - 1))
        is_dup = False
        for u in unique_segments:
            u0, u1 = u[0], u[-1]
            u_len = sum(dist2d(u[i], u[i + 1]) for i in range(len(u) - 1))
            if abs(length - u_len) < 1e-3:
                if (dist2d(p0, u0) < 1e-3 and dist2d(p1, u1) < 1e-3) or (
                    dist2d(p0, u1) < 1e-3 and dist2d(p1, u0) < 1e-3
                ):
                    is_dup = True
                    break
        if not is_dup:
            unique_segments.append(seg)

    return unique_segments


def chain_segments(segments, tol=0.05):
    remaining = [list(s) for s in segments]
    raw_chains = []
    while remaining:
        chain = remaining.pop(0)
        changed = True
        while changed:
            changed = False
            for i, cand in enumerate(remaining):
                if dist2d(chain[-1], cand[0]) <= tol:
                    chain.extend(cand[1:])
                    remaining.pop(i)
                    changed = True
                    break
                elif dist2d(chain[-1], cand[-1]) <= tol:
                    rev = list(reversed(cand))
                    chain.extend(rev[1:])
                    remaining.pop(i)
                    changed = True
                    break
                elif dist2d(chain[0], cand[-1]) <= tol:
                    chain = cand[:-1] + chain
                    remaining.pop(i)
                    changed = True
                    break
                elif dist2d(chain[0], cand[0]) <= tol:
                    rev = list(reversed(cand))
                    chain = rev[:-1] + chain
                    remaining.pop(i)
                    changed = True
                    break
        raw_chains.append(chain)
    return raw_chains


def order_chains_from_origin(chains, start_pt=(0.0, 0.0)):
    ordered = []
    cur_pos = start_pt
    pool = [list(c) for c in chains]
    while pool:
        best_idx = 0
        best_rev = False
        best_d = float("inf")
        for idx, ch in enumerate(pool):
            d_start = dist2d(cur_pos, ch[0])
            d_end = dist2d(cur_pos, ch[-1])
            if d_start < best_d:
                best_d = d_start
                best_idx = idx
                best_rev = False
            if d_end < best_d:
                best_d = d_end
                best_idx = idx
                best_rev = True
        chosen = pool.pop(best_idx)
        if best_rev:
            chosen = list(reversed(chosen))
        ordered.append(chosen)
        cur_pos = chosen[-1]
    return ordered


def convert_dxf_to_gcode(
    dxf_text,
    source_name="Sleeve DXF - Sketch 4.dxf",
    safe_z=5.0,
    cut_z=0.0,
    xy_travel_feed=3600,
    xy_cut_feed=150,
    z_feed=150,
    origin_mode="min",
    zero_at_start=True,
):
    segments = parse_dxf_segments(dxf_text)
    raw_chains = chain_segments(segments, tol=0.05)

    all_pts = [pt for c in raw_chains for pt in c]
    if not all_pts:
        return "; Empty DXF (no valid entities found)\n", {}

    min_x = min(p[0] for p in all_pts)
    max_x = max(p[0] for p in all_pts)
    min_y = min(p[1] for p in all_pts)
    max_y = max(p[1] for p in all_pts)

    offset_x = 0.0
    offset_y = 0.0
    if origin_mode == "center":
        offset_x = -0.5 * (min_x + max_x)
        offset_y = -0.5 * (min_y + max_y)
    elif origin_mode == "min":
        offset_x = -min_x
        offset_y = -min_y

    shifted_chains = [
        [(pt[0] + offset_x, pt[1] + offset_y) for pt in ch]
        for ch in raw_chains
    ]
    chains = order_chains_from_origin(shifted_chains, start_pt=(0.0, 0.0))

    lines = [
        f"; ===========================================================",
        f"; G-Code Generated for: WorkBench 200 Series - Volcanos",
        f"; Controller: Duet Ethernet 1.0 (RepRapFirmware 1.19.2)",
        f"; Source DXF: {source_name}",
        f"; Origin Mode: Bottom-Left Corner at X0.000 Y0.000",
        f"; Contours: {len(chains)} continuous chained toolpaths ({len(segments)} deduplicated segments)",
        f"; Bounding Box: X[{min_x + offset_x:.3f} .. {max_x + offset_x:.3f}] mm, Y[{min_y + offset_y:.3f} .. {max_y + offset_y:.3f}] mm",
        f"; Overall Span: {(max_x - min_x):.3f} mm (W) x {(max_y - min_y):.3f} mm (H)",
        f"; Safe Z: +{safe_z:.2f} mm | Cut/Trace Z: {cut_z:.2f} mm | Z Feed: {z_feed} mm/min | XY Feed: {xy_cut_feed} mm/min",
        f"; ===========================================================",
        "G21 ; Set units to millimeters",
        "G90 ; Absolute positioning",
        "M208 X-1000 Y-1000 Z-510 S1 ; Allow Z axis and probe to travel below Z=0",
        "M564 S0 H0 ; Disable Z=0 software lower-bound clipping",
    ]
    if zero_at_start:
        lines.append("G92 X0 Y0 Z0 ; Set current toolhead position as Bottom-Left Work Origin (0,0,0)")
    lines.append(f"G1 Z{safe_z:.3f} F{z_feed} ; Lift Z to safe clearance height")
    lines.append("")

    total_cut_dist = 0.0
    for c_idx, chain in enumerate(chains, start=1):
        sx, sy = chain[0][0], chain[0][1]
        c_len = sum(dist2d(chain[i], chain[i + 1]) for i in range(len(chain) - 1))
        total_cut_dist += c_len

        lines.append(f"; --- Contour {c_idx}/{len(chains)} ({len(chain)} pts, {c_len:.2f} mm) ---")
        lines.append(f"G0 X{sx:.4f} Y{sy:.4f} F{xy_travel_feed} ; Rapid to contour {c_idx} start")
        lines.append(f"G1 Z{cut_z:.3f} F{z_feed} ; Plunge Z to work height")
        for pt in chain[1:]:
            px, py = pt[0], pt[1]
            lines.append(f"G1 X{px:.4f} Y{py:.4f} F{xy_cut_feed}")
        lines.append(f"G1 Z{safe_z:.3f} F{z_feed} ; Retract Z to safe clearance height")
        lines.append("")

    lines.append(f"; --- Job Complete ---")
    lines.append(f"G0 X0.0000 Y0.0000 F{xy_travel_feed} ; Return XY to Bottom-Left Work Origin (0,0)")
    lines.append(f"G1 Z0.000 F{z_feed} ; Return Z to work origin")
    lines.append("M400 ; Wait for all moves to complete")

    meta = {
        "contours": len(chains),
        "segments": len(segments),
        "points": len(all_pts),
        "width": round(max_x - min_x, 3),
        "height": round(max_y - min_y, 3),
        "min_x": round(min_x + offset_x, 3),
        "max_x": round(max_x + offset_x, 3),
        "min_y": round(min_y + offset_y, 3),
        "max_y": round(max_y + offset_y, 3),
        "cut_distance_mm": round(total_cut_dist, 2),
    }
    return "\n".join(lines) + "\n", meta
