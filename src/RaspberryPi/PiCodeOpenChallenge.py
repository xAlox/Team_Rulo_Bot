#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Team Rulo Bot — WRO 2025
# Autonomous vision: line color decision, PD lateral centering, front black turn,
# and a final top-stop trigger with brief reverse after the 12th line.
# Authors: Wilmer Reyes, Alfonso Duverge, Emil Velasquez

import cv2, numpy as np, time, signal, json, os
import serial, serial.tools.list_ports

# ===================== Line counting / stop logic =====================
NUM_LINES_TO_STOP = 12
LINE_COOLDOWN_S   = 3
POST_ALIGN_S      = 1.5  # kept for compatibility (not used to stop)

# ===================== Camera =====================
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
FONT = cv2.FONT_HERSHEY_SIMPLEX

def open_cam(idx=0):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("[ERROR] No camera (/dev/videoX).")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    return cap

# ===================== Serial =====================
def open_serial(baud=115200):
    for p in ["/dev/ttyUSB0","/dev/ttyACM0"]:
        try: return serial.Serial(p, baudrate=baud, timeout=0.01)
        except: pass
    for p in serial.tools.list_ports.comports():
        try: return serial.Serial(p.device, baudrate=baud, timeout=0.01)
        except: pass
    print("[WARN] No serial; vision-only.")
    return None

def send(ser, s):
    try:
        if ser: ser.write((s+"\n").encode())
    except Exception as e:
        print(f"[WARN] Serial write error: {e}")

# ===================== ROIs (fractions x0,y0,x1,y1) =====================
ROIS = {
    "ORANGE_BOT":     (0.25, 0.82, 0.75, 0.95),   # color/line counting (bottom)
    "FRONT_SQ_RIGHT": (0.50, 0.59, 0.57, 0.70),   # front black (default turn RIGHT)
    "FRONT_SQ_LEFT":  (0.43, 0.59, 0.50, 0.70),   # front black (turn LEFT)
    "FRONT_SIDE_L":   (0.00, 0.60, 0.20, 0.67),   # side PD (left)
    "FRONT_SIDE_R":   (0.80, 0.60, 1.00, 0.67),   # side PD (right)
    "TOP_STOP":       (0.43, 0.40, 0.57, 0.70),   # final stop: black ahead, upper ROI
}

# ===================== Servo / control =====================
SERVO_CENTER    = 84
SERVO_LEFT_LIM  = 24
SERVO_RIGHT_LIM = 144

TURN_BIAS_DEG   = 30.0       # constant bias during front-black turning (R:+ / L:-)
SIDE_BLEND_IN_TURN = 0.5     # mix some side PD while turning
FRONT_BLACK_HYST_FRACT = 0.6 # hysteresis to exit the front-black turn
TX_PERIOD = 0.015            # ~66 Hz

# Lateral PD (area difference of side black masks)
Kp_side = 70.0
Kd_side = 190.0
SIDE_MAX_DEG = 13.0

# ===================== Utils =====================
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def frac_to_rect(img, f):
    h,w = img.shape[:2]
    return (int(f[0]*w), int(f[1]*h), int(f[2]*w), int(f[3]*h))

def draw_roi(frame, rect, label, color=(255,200,0)):
    x0,y0,x1,y1 = rect
    cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
    cv2.putText(frame,label,(x0+4,y0+18),FONT,0.55,color,2,cv2.LINE_AA)

def largest_contour(mask):
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not cnts: return None, 0
    best = max(cnts, key=cv2.contourArea)
    return best, cv2.contourArea(best)

# ===================== Tuning (sliders + persistence) =====================
CONFIG_PATH = os.path.expanduser("~/.rulobot_vision_tuning.json")

DEFAULTS = {
    # Global gains (applied to whole frame)
    "GainS_global": 120, "GainV_global": 90, "Gamma_x100": 100,
    # Orange / Blue HSV and white suppression
    "O_Hmin": 5, "O_Hmax": 30, "O_Smin": 80, "O_Vmin": 80,
    "B_Hmin": 100, "B_Hmax": 130, "B_Smin": 110, "B_Vmin": 40,
    "W_Smax": 70, "W_Vmin": 190,
    # Black (LAB + HSV)
    "L_min": 0, "L_max": 95, "A_tol": 25, "B_tol": 25,
    "Blk_Vmax": 120, "Blk_Smax": 120,
    # Morphology
    "Kernel": 5,
    # Front black area threshold (enter/exit with hysteresis)
    "FrontA_T": 1600,
    # PD for front turn (x100)
    "Kp_turn": 10, "Kd_turn": 25,
    # Line color sensitivity / stability
    "O_minFrac_x1000": 15, "O_minH": 7,  "O_stable": 1,
    "B_minFrac_x1000": 15, "B_minH": 10, "B_stable": 2,
    "ColorMargin_x100": 135,
    # Delayed side PD after GO (ms)
    "SidePD_delay_ms": 600,
    # Final stop (top ROI) after 12th line
    "TopStop_delay_ms": 1500,
    "TopStop_back_ms":  250,
    "TopStop_area_T":   1200
}

def build_tuner():
    cv2.namedWindow("TUNING", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("TUNING", 580, 840)

    # Global
    cv2.createTrackbar("GainS_gl x100", "TUNING", 0, 300, lambda x: None)
    cv2.createTrackbar("GainV_gl x100", "TUNING", 0, 300, lambda x: None)
    cv2.createTrackbar("Gamma x100",    "TUNING", 30, 300, lambda x: None)

    # Orange / Blue
    cv2.createTrackbar("O Hmin", "TUNING", 0, 179, lambda x: None)
    cv2.createTrackbar("O Hmax", "TUNING", 0, 179, lambda x: None)
    cv2.createTrackbar("O Smin", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("O Vmin", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("B Hmin", "TUNING", 0, 179, lambda x: None)
    cv2.createTrackbar("B Hmax", "TUNING", 0, 179, lambda x: None)
    cv2.createTrackbar("B Smin", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("B Vmin", "TUNING", 0, 255, lambda x: None)

    # White suppression
    cv2.createTrackbar("W Smax", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("W Vmin", "TUNING", 0, 255, lambda x: None)

    # Black (LAB + HSV)
    cv2.createTrackbar("L min", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("L max", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("A tol", "TUNING", 0, 64,  lambda x: None)
    cv2.createTrackbar("B tol", "TUNING", 0, 64,  lambda x: None)
    cv2.createTrackbar("Blk Vmax", "TUNING", 0, 255, lambda x: None)
    cv2.createTrackbar("Blk Smax", "TUNING", 0, 255, lambda x: None)

    # Morph / Front / PD turn
    cv2.createTrackbar("Kernel", "TUNING", 1, 15, lambda x: None)
    cv2.createTrackbar("FrontArea T", "TUNING", 0, 10000, lambda x: None)
    cv2.createTrackbar("Kp_turn x100", "TUNING", 0, 400, lambda x: None)
    cv2.createTrackbar("Kd_turn x100", "TUNING", 0, 400, lambda x: None)

    # Color sensitivity / stability
    cv2.createTrackbar("O frac x1000", "TUNING", 0, 200, lambda x: None)
    cv2.createTrackbar("O hMin", "TUNING", 0, 100, lambda x: None)
    cv2.createTrackbar("O stable", "TUNING", 0, 5, lambda x: None)
    cv2.createTrackbar("B frac x1000", "TUNING", 0, 200, lambda x: None)
    cv2.createTrackbar("B hMin", "TUNING", 0, 100, lambda x: None)
    cv2.createTrackbar("B stable", "TUNING", 0, 5, lambda x: None)
    cv2.createTrackbar("Margin x100", "TUNING", 100, 300, lambda x: None)

    # Delays and top-stop
    cv2.createTrackbar("SidePD delay ms", "TUNING", 0, 3000, lambda x: None)
    cv2.createTrackbar("TopStop delay ms", "TUNING", 0, 4000, lambda x: None)
    cv2.createTrackbar("TopStop back ms",  "TUNING", 0, 2000, lambda x: None)
    cv2.createTrackbar("TopStop area T",   "TUNING", 0, 20000, lambda x: None)

def set_trackbar_positions(cfg):
    c = (DEFAULTS if cfg is None else {**DEFAULTS, **cfg})
    setb = lambda n,v: cv2.setTrackbarPos(n, "TUNING", int(v))
    for k,v in [
        ("GainS_gl x100",c["GainS_global"]), ("GainV_gl x100",c["GainV_global"]), ("Gamma x100",c["Gamma_x100"]),
        ("O Hmin",c["O_Hmin"]), ("O Hmax",c["O_Hmax"]), ("O Smin",c["O_Smin"]), ("O Vmin",c["O_Vmin"]),
        ("B Hmin",c["B_Hmin"]), ("B Hmax",c["B_Hmax"]), ("B Smin",c["B_Smin"]), ("B Vmin",c["B_Vmin"]),
        ("W Smax",c["W_Smax"]), ("W Vmin",c["W_Vmin"]),
        ("L min",c["L_min"]), ("L max",c["L_max"]), ("A tol",c["A_tol"]), ("B tol",c["B_tol"]),
        ("Blk Vmax",c["Blk_Vmax"]), ("Blk Smax",c["Blk_Smax"]),
        ("Kernel",c["Kernel"]), ("FrontArea T",c["FrontA_T"]),
        ("Kp_turn x100",c["Kp_turn"]), ("Kd_turn x100",c["Kd_turn"]),
        ("O frac x1000",c["O_minFrac_x1000"]), ("O hMin",c["O_minH"]), ("O stable",c["O_stable"]),
        ("B frac x1000",c["B_minFrac_x1000"]), ("B hMin",c["B_minH"]), ("B stable",c["B_stable"]),
        ("Margin x100",c["ColorMargin_x100"]),
        ("SidePD delay ms",c["SidePD_delay_ms"]),
        ("TopStop delay ms",c["TopStop_delay_ms"]),
        ("TopStop back ms",c["TopStop_back_ms"]),
        ("TopStop area T",c["TopStop_area_T"]),
    ]: setb(k,v)

def read_tuner():
    g = lambda n: cv2.getTrackbarPos(n,"TUNING")
    return dict(
        GainS_global=g("GainS_gl x100"), GainV_global=g("GainV_gl x100"), Gamma_x100=g("Gamma x100"),
        O_Hmin=g("O Hmin"), O_Hmax=g("O Hmax"), O_Smin=g("O Smin"), O_Vmin=g("O Vmin"),
        B_Hmin=g("B Hmin"), B_Hmax=g("B Hmax"), B_Smin=g("B Smin"), B_Vmin=g("B Vmin"),
        W_Smax=g("W Smax"), W_Vmin=g("W Vmin"),
        L_min=g("L min"), L_max=g("L max"), A_tol=g("A tol"), B_tol=g("B tol"),
        Blk_Vmax=g("Blk Vmax"), Blk_Smax=g("Blk Smax"),
        Kernel=max(1, g("Kernel")|1),
        FrontA_T=g("FrontArea T"),
        Kp_turn=g("Kp_turn x100"), Kd_turn=g("Kd_turn x100"),
        O_minFrac_x1000=g("O frac x1000"), O_minH=g("O hMin"), O_stable=g("O stable"),
        B_minFrac_x1000=g("B frac x1000"), B_minH=g("B hMin"), B_stable=g("B stable"),
        ColorMargin_x100=g("Margin x100"),
        SidePD_delay_ms=g("SidePD delay ms"),
        TopStop_delay_ms=g("TopStop delay ms"),
        TopStop_back_ms=g("TopStop back ms"),
        TopStop_area_T=g("TopStop area T"),
    )

def save_config(vals, path=CONFIG_PATH):
    try:
        with open(path, "w") as f: json.dump(vals, f, indent=2)
        print(f"[CFG] Guardado: {path}")
    except Exception as e:
        print(f"[CFG] Error guardando: {e}")

def load_config(path=CONFIG_PATH):
    try:
        with open(path, "r") as f:
            data = json.load(f)
        print(f"[CFG] Cargado: {path}")
        return data
    except Exception:
        return None

# ===================== Image-level gains =====================
def apply_global_gains(bgr, gs_x100, gv_x100, gamma_x100):
    """Simple global S/V gain + gamma in HSV; keeps tuning visible/consistent."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    s = np.clip((s.astype(np.float32) * (gs_x100/100.0)), 0, 255)
    v = np.clip(v.astype(np.float32) * (gv_x100/100.0), 0, 255)
    gamma = max(0.3, gamma_x100/100.0)
    v = (np.clip(v/255.0, 0, 1)**gamma)*255.0
    hsv_adj = cv2.merge([h, s.astype(np.uint8), v.astype(np.uint8)])
    return cv2.cvtColor(hsv_adj, cv2.COLOR_HSV2BGR)

# ===================== Color / Black detections =====================
def detect_orange(proc_bgr, rect, P):
    """Return (area, bbox_h) of strongest orange blob in ROI; suppress white glare."""
    x0,y0,x1,y1 = rect
    roi = proc_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (P["O_Hmin"], P["O_Smin"], P["O_Vmin"]), (P["O_Hmax"],255,255))
    mW1 = cv2.inRange(hsv, (0,0,P["W_Vmin"]), (179,255,255))
    mW2 = cv2.inRange(hsv, (0,0,0), (179,P["W_Smax"],255))
    m = cv2.bitwise_and(m1, cv2.bitwise_not(mW1|mW2))
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(P["Kernel"],P["Kernel"]))
    m = cv2.GaussianBlur(m,(5,5),0); m = cv2.erode(m,k,1); m = cv2.dilate(m,k,1)
    cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best=None
    if cnts:
        best = max(cnts, key=cv2.contourArea)
        cv2.drawContours(roi, [best], -1, (0,165,255), 2)
    area = (cv2.contourArea(best) if best is not None else 0.0)
    hbox = 0
    if best is not None:
        x,y,w,h = cv2.boundingRect(best); hbox = h
        cv2.rectangle(roi, (x,y), (x+w,y+h), (0,140,255), 2)
    return area, hbox

def detect_blue(proc_bgr, rect, P):
    """Return (area, bbox_h) of strongest blue blob in ROI; suppress white glare."""
    x0,y0,x1,y1 = rect
    roi = proc_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (P["B_Hmin"], P["B_Smin"], P["B_Vmin"]), (P["B_Hmax"],255,255))
    mW1 = cv2.inRange(hsv, (0,0,P["W_Vmin"]), (179,255,255))
    mW2 = cv2.inRange(hsv, (0,0,0), (179,P["W_Smax"],255))
    m = cv2.bitwise_and(m1, cv2.bitwise_not(mW1|mW2))
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(P["Kernel"],P["Kernel"]))
    m = cv2.GaussianBlur(m,(5,5),0); m = cv2.erode(m,k,1); m = cv2.dilate(m,k,1)
    cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best=None
    if cnts:
        best = max(cnts, key=cv2.contourArea)
        cv2.drawContours(roi, [best], -1, (255,120,0), 2)
    area = (cv2.contourArea(best) if best is not None else 0.0)
    hbox = 0
    if best is not None:
        x,y,w,h = cv2.boundingRect(best); hbox = h
        cv2.rectangle(roi, (x,y), (x+w,y+h), (255,120,0), 2)
    return area, hbox

def black_mask_combined(proc_bgr, P):
    """Robust black = (LAB in-range) AND (HSV low V,S)."""
    lab = cv2.cvtColor(proc_bgr, cv2.COLOR_BGR2LAB)
    L = lab[:,:,0]; A = lab[:,:,1]; B = lab[:,:,2]
    mL = cv2.inRange(L, P["L_min"], P["L_max"])
    mA = cv2.inRange(A, 128-P["A_tol"], 128+P["A_tol"])
    mB = cv2.inRange(B, 128-P["B_tol"], 128+P["B_tol"])
    mLAB = cv2.bitwise_and(cv2.bitwise_and(mL,mA), mB)
    hsv = cv2.cvtColor(proc_bgr, cv2.COLOR_BGR2HSV)
    H,S,V = cv2.split(hsv)
    mHSV = cv2.inRange(S, 0, P["Blk_Smax"]) & cv2.inRange(V, 0, P["Blk_Vmax"])
    m = cv2.bitwise_and(mLAB, mHSV)
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(P["Kernel"],P["Kernel"]))
    m = cv2.GaussianBlur(m,(5,5),0); m = cv2.erode(m,k,1); m = cv2.dilate(m,k,1)
    return m

# ===================== Main =====================
_stop = False
def _sigint(_s,_f):
    global _stop
    _stop = True

def main():
    signal.signal(signal.SIGINT, _sigint)
    cap = open_cam(CAM_INDEX)
    ser = open_serial(115200)

    win = "Team Rulo Bot — PD Turn + Side Centering"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 960, 720)
    build_tuner()
    cfg = load_config()
    set_trackbar_positions(cfg)

    # State
    armed=False
    selected_turn=None  # 'R' / 'L' (chosen by bottom color)
    orange_hits=0; blue_hits=0
    turning=False
    prev_err_turn=0.0
    prev_err_side = 0.0
    last_tx_ts=0.0

    line_count = 0
    last_line_ts = 0.0

    # Side PD enable delay after GO
    sidepd_enable_at = 0.0

    # Final stop after 12th line (top ROI)
    topstop_arm_ts = 0.0
    topstop_armed = False
    topstop_done = False

    rx_buf=b""
    t0=time.time(); n=0; fps=0.0

    while not _stop:
        ok, frame_raw = cap.read()
        if not ok: continue
        n+=1; t1=time.time()
        if (t1-t0)>=1.0: fps=n/(t1-t0); n=0; t0=t1

        P = read_tuner()
        Kp_turn = P["Kp_turn"]/100.0
        Kd_turn = P["Kd_turn"]/100.0
        O_minFrac = P["O_minFrac_x1000"]/1000.0
        B_minFrac = P["B_minFrac_x1000"]/1000.0
        O_minH    = int(P["O_minH"]); B_minH = int(P["B_minH"])
        O_stable  = max(1,int(P["O_stable"])); B_stable = max(1,int(P["B_stable"]))
        COLOR_DECISION_MARGIN = max(1.0, P["ColorMargin_x100"]/100.0)

        frame = apply_global_gains(frame_raw, P["GainS_global"], P["GainV_global"], P["Gamma_x100"])

        # ROIs
        orangeR = frac_to_rect(frame, ROIS["ORANGE_BOT"])
        if selected_turn == 'L':
            frontR = frac_to_rect(frame, ROIS["FRONT_SQ_LEFT"]);  front_label = "FRONT (LEFT ROI)"
        else:
            frontR = frac_to_rect(frame, ROIS["FRONT_SQ_RIGHT"]); front_label = "FRONT (RIGHT ROI)"
        sideL   = frac_to_rect(frame, ROIS["FRONT_SIDE_L"])
        sideR   = frac_to_rect(frame, ROIS["FRONT_SIDE_R"])
        topStopR= frac_to_rect(frame, ROIS["TOP_STOP"])

        # Draw ROIs (debug)
        draw_roi(frame, orangeR, "LINE/DECISION")
        draw_roi(frame, frontR,  front_label, (0,255,255))
        draw_roi(frame, sideL,   "SIDE_L", (0,255,0))
        draw_roi(frame, sideR,   "SIDE_R", (0,255,0))
        draw_roi(frame, topStopR,"TOP_STOP", (255,0,255))

        # GO/STOP from MCU
        if ser and ser.in_waiting:
            try:
                rx_buf += ser.read(ser.in_waiting)
                while b"\n" in rx_buf:
                    line, rx_buf = rx_buf.split(b"\n",1)
                    s=line.decode(errors='ignore').strip().upper()
                    if s=="GO":
                        armed=True
                        selected_turn=None
                        orange_hits=blue_hits=0
                        turning=False
                        prev_err_turn=0.0
                        prev_err_side=0.0
                        line_count = 0
                        topstop_arm_ts = 0.0
                        topstop_armed = False
                        topstop_done  = False
                        send(ser, f"STEER:{SERVO_CENTER}")
                        sidepd_enable_at = time.time() + (P["SidePD_delay_ms"]/1000.0)
                        print("[INFO] ARMED by GO")
                    elif s=="STOP":
                        armed=False
                        turning=False
                        print("[INFO] DISARMED by STOP")
            except Exception as e:
                print(f"[WARN] Serial read error: {e}")

        now = time.time()

        # ---------- Bottom color: choose turn direction (once) ----------
        x0,y0,x1,y1 = orangeR
        roi_area_color = max(1,(x1-x0)*(y1-y0))
        aO,hO = detect_orange(frame, orangeR, P)
        aB,hB = detect_blue(frame,  orangeR, P)
        fracO = aO/roi_area_color; fracB = aB/roi_area_color

        if armed and (selected_turn is None):
            orange_hits = (orange_hits+1) if (fracO>=O_minFrac and hO>=O_minH) else 0
            blue_hits   = (blue_hits+1)   if (fracB>=B_minFrac and hB>=B_minH) else 0
            if orange_hits>=O_stable and (blue_hits==0 or fracO>=fracB*COLOR_DECISION_MARGIN):
                selected_turn='R'; print("[MODE] TURN RIGHT (orange)")
            elif blue_hits>=B_stable and (orange_hits==0 or fracB>=fracO*COLOR_DECISION_MARGIN):
                selected_turn='L'; print("[MODE] TURN LEFT (blue)")

        # ---------- Line counting (no hard stop) ----------
        saw_line = (fracO>=O_minFrac) or (fracB>=B_minFrac)
        if armed and saw_line:
            if (now - last_line_ts) >= LINE_COOLDOWN_S:
                line_count += 1
                last_line_ts = now
                print(f"[LINE] count = {line_count}")
                if (line_count == NUM_LINES_TO_STOP):
                    # Arm top-stop after adjustable delay; vehicle keeps running normally
                    topstop_arm_ts = now + (P["TopStop_delay_ms"]/1000.0)
                    topstop_armed = False
                    topstop_done  = False
                    print(f"[TOPSTOP] armed to activate at +{P['TopStop_delay_ms']} ms")

        if (not topstop_armed) and (topstop_arm_ts>0.0) and (now >= topstop_arm_ts):
            topstop_armed = True
            print("[TOPSTOP] ARMED: watching TOP_STOP ROI for black")

        # ---------- Black masks (front + sides) ----------
        fx0,fy0,fx1,fy1 = frontR
        roi_front = frame[fy0:fy1, fx0:fx1]
        m_front = black_mask_combined(roi_front, P)
        cntF, area_front = largest_contour(m_front)
        if cntF is not None: cv2.drawContours(roi_front, [cntF], -1, (0,255,255), 2)
        cv2.addWeighted(roi_front,0.65,cv2.cvtColor(m_front,cv2.COLOR_GRAY2BGR),0.35,0,roi_front)

        slx0,sly0,slx1,sly1 = sideL; srx0,sry0,srx1,sry1 = sideR
        roi_l = frame[sly0:sly1, slx0:slx1]; roi_r = frame[sry0:sry1, srx0:srx1]
        m_l = black_mask_combined(roi_l, P); m_r = black_mask_combined(roi_r, P)
        cv2.addWeighted(roi_l,0.65,cv2.cvtColor(m_l,cv2.COLOR_GRAY2BGR),0.35,0,roi_l)
        cv2.addWeighted(roi_r,0.65,cv2.cvtColor(m_r,cv2.COLOR_GRAY2BGR),0.35,0,roi_r)
        area_l = float(cv2.countNonZero(m_l)); area_r = float(cv2.countNonZero(m_r))

        # ---------- Front black turning state machine ----------
        if armed and (selected_turn is not None):
            enter = (area_front >= float(P["FrontA_T"]))
            exit_ = (area_front <= float(P["FrontA_T"])*FRONT_BLACK_HYST_FRACT)
            if (not turning) and enter:
                turning=True; prev_err_turn=0.0; send(ser,"TURN_ON"); print("[MODE] TURNING=ON")
            elif turning and exit_:
                turning=False; send(ser,"TURN_OFF"); print("[MODE] TURNING=OFF")

        # ---------- Side PD (centering on black area difference) ----------
        err_side = area_l - area_r
        d_err_side = err_side - prev_err_side
        prev_err_side = err_side
        side_pd = (Kp_side * err_side) + (Kd_side * d_err_side)
        side_pd = float(clamp(side_pd, -SIDE_MAX_DEG, SIDE_MAX_DEG))
        if now < sidepd_enable_at:
            side_pd = 0.0  # delay at start to avoid early wobble

        # ---------- PD turn (front contour centroid) ----------
        effective_turning = turning
        turn_pd = 0.0
        bias = 0.0
        if effective_turning and (cntF is not None) and (area_front > 0):
            M = cv2.moments(cntF)
            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                roi_cx = (fx1-fx0)//2
                err_turn = float(cx - roi_cx)
                d_err_turn = err_turn - prev_err_turn
                turn_pd = (P["Kp_turn"]/100.0 * err_turn) + (P["Kd_turn"]/100.0 * d_err_turn)
                prev_err_turn = err_turn
        if selected_turn is not None and effective_turning:
            bias = TURN_BIAS_DEG if (selected_turn == 'R') else -TURN_BIAS_DEG

        # ---------- Final Top-Stop (after 12th line, upper ROI sees black) ----------
        if topstop_armed and (not topstop_done):
            tx0,ty0,tx1,ty1 = topStopR
            roi_top = frame[ty0:ty1, tx0:tx1]
            m_top = black_mask_combined(roi_top, P)
            area_top = float(cv2.countNonZero(m_top))
            cv2.addWeighted(roi_top,0.65,cv2.cvtColor(m_top,cv2.COLOR_GRAY2BGR),0.35,0,roi_top)

            if area_top >= float(P["TopStop_area_T"]):
                # Issue: center steer, brief reverse, then STOP
                steer_cmd = SERVO_CENTER
                send(ser, f"STEER:{steer_cmd}")
                send(ser, f"REV:{int(P['TopStop_back_ms'])}")
                time.sleep(max(0.0, P["TopStop_back_ms"]/1000.0))
                send(ser, "STOP")
                armed=False
                topstop_done=True
                print(f"[TOPSTOP] Triggered (area={int(area_top)}). REV {P['TopStop_back_ms']} ms + STOP.")

        # ---------- Output steering (normal running) ----------
        if armed:
            if effective_turning:
                steer = SERVO_CENTER + (bias + turn_pd + SIDE_BLEND_IN_TURN*side_pd)
            else:
                steer = SERVO_CENTER + side_pd
            steer = int(round(clamp(steer, SERVO_LEFT_LIM, SERVO_RIGHT_LIM)))
            if (now - last_tx_ts) >= TX_PERIOD:
                send(ser, f"STEER:{steer}")
                last_tx_ts = now

        # HUD (debug only)
        dir_txt  = {None:"WAIT_COLOR", 'R':"RIGHT", 'L':"LEFT"}[selected_turn]
        mode_txt = ("TURN" if turning else ("RUN" if armed else ("HALT" if topstop_done else "IDLE")))
        hud = f"{FRAME_W}x{FRAME_H}  ~{fps:.1f}fps  armed:{int(armed)}  DIR:{dir_txt}  mode:{mode_txt}  lines:{line_count}/{NUM_LINES_TO_STOP}"
        if topstop_armed and not topstop_done: hud += "  [TOPSTOP ARMED]"
        cv2.putText(frame, hud, (10,20), FONT, 0.6, (255,255,255), 2, cv2.LINE_AA)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            save_config(read_tuner()); break
        elif key == ord('s'):
            save_config(read_tuner()); print("[CFG] Saved (s).")
        elif key == ord('r'):
            set_trackbar_positions(DEFAULTS); print("[CFG] Reset to defaults.")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Bye.")

if __name__ == "__main__":
    main()
