#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Team Rulo Bot — Vision (GO-armed) — PD Turn + Side Centering
LAB ColorCalibrator + fallback fijo (sin sliders de color).

- Usa modelo LAB en ~/rulobot_lab_color_model.json (Mahalanobis).
- Si no existe, usa umbrales fijos de respaldo (no tunables).
- El panel TUNING NO tiene nada de color: solo cámara, morfología, PDs, y tiempos.
"""

import cv2, numpy as np, time, signal, json, os
import serial, serial.tools.list_ports

# ===================== Parámetros de parada por líneas =====================
NUM_LINES_TO_STOP = 12
LINE_COOLDOWN_S   = 2
POST_ALIGN_S      = 3.3  # compatibilidad; no se usa para parar

# ===================== Cámara =====================
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

# ===================== ROIs (fracciones x0,y0,x1,y1) =====================
ROIS = {
    "ORANGE_BOT":     (0.20, 0.82, 0.80, 0.95),  # abajo: decisión/lineas naranja-azul

    # FRONT: por defecto RIGHT / alternativo LEFT
    "FRONT_SQ_RIGHT": (0.50, 0.60, 0.57, 0.75),
    "FRONT_SQ_LEFT":  (0.43, 0.60, 0.50, 0.75),

    # Laterales (PD lateral)
    "FRONT_SIDE_L":   (0.00, 0.63, 0.20, 0.80),
    "FRONT_SIDE_R":   (0.80, 0.63, 1.00, 0.80),

    # ROI superior para paro final por negro
    "TOP_STOP":       (0.43, 0.43, 0.57, 0.70),
}

# ===================== Servo & control =====================
SERVO_CENTER    = 84
SERVO_LEFT_LIM  = 28
SERVO_RIGHT_LIM = 140

TURN_BIAS_DEG   = 32.0
SIDE_BLEND_IN_TURN = 0.5
FRONT_BLACK_HYST_FRACT = 0.6
TX_PERIOD = 0.015

# === Lateral PD clásico (constantes) ===
Kp_side = 30.0
Kd_side = 120.0
SIDE_MAX_DEG = 13.0

# === Lógica decisión de color (constantes SIN sliders) ===
O_MIN_FRAC = 0.015   # fracción mínima de área naranja en ORANGE_BOT
O_MIN_H    = 7       # altura mínima bbox
O_STABLE   = 1       # frames consecutivos
B_MIN_FRAC = 0.015
B_MIN_H    = 10
B_STABLE   = 2
COLOR_DECISION_MARGIN = 1.35  # ventaja mínima de un color sobre el otro

# ===================== Helpers =====================
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

# ===================== Archivos / modelos =====================
LAB_MODEL_PATH    = os.path.expanduser("~/rulobot_lab_color_model.json")  # ColorCalibrator
HSV_TUNING_PATH   = os.path.expanduser("~/.rulobot_vision_tuning.json")   # solo para NO-color (cámara/PD/timers)

def load_json(path):
    try:
        with open(path, "r") as f: return json.load(f)
    except Exception: return None

LAB = load_json(LAB_MODEL_PATH)  # dict con claves: orange, blue, black, green, red

def get_lab_model(name: str):
    if not LAB: return None
    aliases = {
        "orange": ["orange","naranja","line_orange","O"],
        "blue":   ["blue","azul","line_blue","B"],
        "black":  ["black","negro","wall","floor_black"],
        "green":  ["green","verde","pillar_green","G"],
        "red":    ["red","rojo","pillar_red","R"],
    }
    for key, arr in aliases.items():
        if name.lower() in arr: return LAB.get(key, None)
    return LAB.get(name, None)

# ===================== TUNING (sin color) =====================
CONFIG_PATH = HSV_TUNING_PATH  # reusamos archivo, pero solo para NO-color

DEFAULTS = {
    # --- Cámara / imagen ---
    "GainS_global": 120, "GainV_global": 90, "Gamma_x100": 100,

    # --- Morfología ---
    "Kernel": 5,

    # --- Umbral área frontal para giro ---
    "FrontA_T":  1600,

    # --- PD giro (x100) ---
    "Kp_turn": 10, "Kd_turn": 25,

    # --- Delay PD lateral / Timers TOP STOP ---
    "SidePD_delay_ms": 600,
    "TopStop_delay_ms": 1500,
    "TopStop_back_ms":  250,
    "TopStop_area_T":   1200
}

def build_tuner():
    cv2.namedWindow("TUNING", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("TUNING", 520, 460)

    # Cámara
    cv2.createTrackbar("GainS_gl x100", "TUNING", 0, 300, lambda x: None)
    cv2.createTrackbar("GainV_gl x100", "TUNING", 0, 300, lambda x: None)
    cv2.createTrackbar("Gamma x100",    "TUNING", 30, 300, lambda x: None)

    # Morfología / Giro frontal / PDs
    cv2.createTrackbar("Kernel",        "TUNING", 1, 15,  lambda x: None)
    cv2.createTrackbar("FrontArea T",   "TUNING", 0, 10000, lambda x: None)
    cv2.createTrackbar("Kp_turn x100",  "TUNING", 0, 400, lambda x: None)
    cv2.createTrackbar("Kd_turn x100",  "TUNING", 0, 400, lambda x: None)

    # Delays
    cv2.createTrackbar("SidePD delay ms","TUNING", 0, 3000, lambda x: None)

    # TOP STOP
    cv2.createTrackbar("TopStop delay ms","TUNING", 0, 4000, lambda x: None)
    cv2.createTrackbar("TopStop back ms", "TUNING", 0, 2000, lambda x: None)
    cv2.createTrackbar("TopStop area T",  "TUNING", 0, 20000, lambda x: None)

def set_trackbar_positions(cfg):
    c = (DEFAULTS if cfg is None else {**DEFAULTS, **cfg})
    setb = lambda name, val: cv2.setTrackbarPos(name, "TUNING", int(val))
    for k,v in [
        ("GainS_gl x100",c["GainS_global"]),
        ("GainV_gl x100",c["GainV_global"]),
        ("Gamma x100",   c["Gamma_x100"]),
        ("Kernel",       c["Kernel"]),
        ("FrontArea T",  c["FrontA_T"]),
        ("Kp_turn x100", c["Kp_turn"]),
        ("Kd_turn x100", c["Kd_turn"]),
        ("SidePD delay ms",c["SidePD_delay_ms"]),
        ("TopStop delay ms",c["TopStop_delay_ms"]),
        ("TopStop back ms", c["TopStop_back_ms"]),
        ("TopStop area T",  c["TopStop_area_T"]),
    ]: setb(k,v)

def read_tuner():
    g = lambda n: cv2.getTrackbarPos(n,"TUNING")
    return dict(
        GainS_global=g("GainS_gl x100"),
        GainV_global=g("GainV_gl x100"),
        Gamma_x100=g("Gamma x100"),
        Kernel=max(1, g("Kernel")|1),
        FrontA_T=g("FrontArea T"),
        Kp_turn=g("Kp_turn x100"),
        Kd_turn=g("Kd_turn x100"),
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

# ===================== Procesado global imagen =====================
def apply_global_gains(bgr, gs_x100, gv_x100, gamma_x100):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    s = np.clip((s.astype(np.float32) * (gs_x100/100.0)), 0, 255)
    v = np.clip(v.astype(np.float32) * (gv_x100/100.0), 0, 255)
    gamma = max(0.3, gamma_x100/100.0)
    v = (np.clip(v/255.0, 0, 1)**gamma)*255.0
    hsv_adj = cv2.merge([h, s.astype(np.uint8), v.astype(np.uint8)])
    return cv2.cvtColor(hsv_adj, cv2.COLOR_HSV2BGR)

# ===================== Color: LAB + Mahalanobis / Fallback fijo =====================
# Fallback HSV fijo (solo si no hay modelo LAB); NO es tunable.
FALLBACK_ORANGE_HSV = (5, 30, 80, 80)     # (hmin,hmax,smin,vmin)
FALLBACK_BLUE_HSV   = (100, 130, 110, 40)

# Fallback negro (LAB+HSV) fijo
FALLBACK_BLACK = dict(L_min=0, L_max=95, A_tol=25, B_tol=25, Blk_Vmax=120, Blk_Smax=120)

def mahalanobis_mask(bgr, model):
    """model: {'mean':[a,b], 'cov_inv':[[..],[..]], 'tau2':float} en (A,B) de LAB."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    A = lab[:,:,1].astype(np.float32)
    B = lab[:,:,2].astype(np.float32)
    mean = np.array(model["mean"], dtype=np.float32)
    cov_inv = np.array(model["cov_inv"], dtype=np.float32)
    tau2 = float(model["tau2"])
    X = np.stack([A - mean[0], B - mean[1]], axis=-1)
    dist2 = np.einsum('...i,ij,...j->...', X, cov_inv, X)
    return (dist2 <= tau2).astype(np.uint8) * 255

def mask_color(bgr, name):
    """Color por LAB si hay modelo; si no, HSV fijo."""
    m = get_lab_model(name)
    if m is not None:
        return mahalanobis_mask(bgr, m)
    if name == "orange":
        hmin,hmax,smin,vmin = FALLBACK_ORANGE_HSV
    elif name == "blue":
        hmin,hmax,smin,vmin = FALLBACK_BLUE_HSV
    else:
        # para otros colores no usados en este script
        hmin,hmax,smin,vmin = 0,0,0,255
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (hmin,smin,vmin), (hmax,255,255))

def mask_black(bgr):
    """Negro por LAB si hay modelo; si no, fallback fijo combinado."""
    m = get_lab_model("black")
    if m is not None:
        return mahalanobis_mask(bgr, m)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L = lab[:,:,0]; A = lab[:,:,1]; B = lab[:,:,2]
    mL = cv2.inRange(L, FALLBACK_BLACK["L_min"], FALLBACK_BLACK["L_max"])
    mA = cv2.inRange(A, 128-FALLBACK_BLACK["A_tol"], 128+FALLBACK_BLACK["A_tol"])
    mB = cv2.inRange(B, 128-FALLBACK_BLACK["B_tol"], 128+FALLBACK_BLACK["B_tol"])
    mLAB = cv2.bitwise_and(cv2.bitwise_and(mL,mA), mB)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    S = hsv[:,:,1]; V = hsv[:,:,2]
    mHSV = cv2.inRange(S, 0, FALLBACK_BLACK["Blk_Smax"]) & cv2.inRange(V, 0, FALLBACK_BLACK["Blk_Vmax"])
    m = cv2.bitwise_and(mLAB, mHSV)
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    m = cv2.GaussianBlur(m,(5,5),0); m = cv2.erode(m,k,1); m = cv2.dilate(m,k,1)
    return m

# ===================== Detecciones =====================
def detect_orange(proc_bgr, rect, kernel):
    x0,y0,x1,y1 = rect
    roi = proc_bgr[y0:y1, x0:x1]
    m = mask_color(roi, "orange")
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(kernel,kernel))
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

def detect_blue(proc_bgr, rect, kernel):
    x0,y0,x1,y1 = rect
    roi = proc_bgr[y0:y1, x0:x1]
    m = mask_color(roi, "blue")
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(kernel,kernel))
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

def black_mask_combined(proc_bgr, kernel):
    m = mask_black(proc_bgr)
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(kernel,kernel))
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

    win = "Team Rulo Bot — PD Turn + Side Centering (LAB compatible; sin sliders de color)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 960, 720)
    build_tuner()
    cfg = load_config()
    set_trackbar_positions(cfg)

    # Estados
    armed=False
    selected_turn=None
    orange_hits=0; blue_hits=0
    turning=False
    prev_err_turn=0.0
    prev_err_side = 0.0
    last_tx_ts=0.0

    # Conteo de líneas
    line_count = 0
    last_line_ts = 0.0

    # Side PD delay
    sidepd_enable_at = 0.0

    # Paro final
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

        # Sliders NO-color
        P = read_tuner()
        Kp_turn = P["Kp_turn"]/100.0
        Kd_turn = P["Kd_turn"]/100.0
        kernel   = int(P["Kernel"])

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

        # Dibujar ROIs
        draw_roi(frame, orangeR, "LINE/DECISION")
        draw_roi(frame, frontR,  front_label, (0,255,255))
        draw_roi(frame, sideL,   "SIDE_L", (0,255,0))
        draw_roi(frame, sideR,   "SIDE_R", (0,255,0))
        draw_roi(frame, topStopR,"TOP_STOP", (255,0,255))

        # RX GO/STOP
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

        # --- Detección en ROI inferior (naranja/azul) ---
        x0,y0,x1,y1 = orangeR
        roi_area_color = max(1,(x1-x0)*(y1-y0))
        aO,hO = detect_orange(frame, orangeR, kernel)
        aB,hB = detect_blue(frame,  orangeR, kernel)
        fracO = aO/roi_area_color; fracB = aB/roi_area_color

        # Decisión de color (sin sliders)
        if armed and (selected_turn is None):
            orange_hits = (orange_hits+1) if (fracO>=O_MIN_FRAC and hO>=O_MIN_H) else 0
            blue_hits   = (blue_hits+1)   if (fracB>=B_MIN_FRAC and hB>=B_MIN_H) else 0
            if orange_hits>=O_STABLE and (blue_hits==0 or fracO>=fracB*COLOR_DECISION_MARGIN):
                selected_turn='R'; print("[MODE] TURN RIGHT (orange)")
            elif blue_hits>=B_STABLE and (orange_hits==0 or fracB>=fracO*COLOR_DECISION_MARGIN):
                selected_turn='L'; print("[MODE] TURN LEFT (blue)")

        # Conteo de líneas
        saw_line = (fracO>=O_MIN_FRAC) or (fracB>=B_MIN_FRAC)
        if armed and saw_line:
            if (now - last_line_ts) >= LINE_COOLDOWN_S:
                line_count += 1
                last_line_ts = now
                print(f"[LINE] count = {line_count}")
                if (line_count == NUM_LINES_TO_STOP):
                    topstop_arm_ts = now + (P["TopStop_delay_ms"]/1000.0)
                    topstop_armed = False
                    topstop_done  = False
                    print(f"[TOPSTOP] armed to activate at +{P['TopStop_delay_ms']} ms")

        # Armar top-stop tras el delay
        if (not topstop_armed) and (topstop_arm_ts>0.0) and (now >= topstop_arm_ts):
            topstop_armed = True
            print("[TOPSTOP] ARMED: watching TOP_STOP ROI for black")

        # Negro frontal y laterales
        fx0,fy0,fx1,fy1 = frontR
        roi_front = frame[fy0:fy1, fx0:fx1]
        m_front = black_mask_combined(roi_front, kernel)
        cntF, area_front = largest_contour(m_front)
        if cntF is not None: cv2.drawContours(roi_front, [cntF], -1, (0,255,255), 2)
        cv2.addWeighted(roi_front,0.65,cv2.cvtColor(m_front,cv2.COLOR_GRAY2BGR),0.35,0,roi_front)

        slx0,sly0,slx1,sly1 = sideL; srx0,sry0,srx1,sry1 = sideR
        roi_l = frame[sly0:sly1, slx0:slx1]; roi_r = frame[sry0:sry1, srx0:srx1]
        m_l = black_mask_combined(roi_l, kernel); m_r = black_mask_combined(roi_r, kernel)
        cv2.addWeighted(roi_l,0.65,cv2.cvtColor(m_l,cv2.COLOR_GRAY2BGR),0.35,0,roi_l)
        cv2.addWeighted(roi_r,0.65,cv2.cvtColor(m_r,cv2.COLOR_GRAY2BGR),0.35,0,roi_r)
        area_l = float(cv2.countNonZero(m_l)); area_r = float(cv2.countNonZero(m_r))

        # ===== Gestión de giro =====
        if armed and (selected_turn is not None):
            enter = (area_front >= float(P["FrontA_T"]))
            exit_ = (area_front <= float(P["FrontA_T"])*FRONT_BLACK_HYST_FRACT)
            if (not turning) and enter:
                turning=True; prev_err_turn=0.0; send(ser,"TURN_ON"); print("[MODE] TURNING=ON")
            elif turning and exit_:
                turning=False; send(ser,"TURN_OFF"); print("[MODE] TURNING=OFF")

        # ===== PD lateral =====
        err_side = area_l - area_r
        d_err_side = err_side - prev_err_side
        prev_err_side = err_side
        side_pd = (Kp_side * err_side) + (Kd_side * d_err_side)
        side_pd = float(clamp(side_pd, -SIDE_MAX_DEG, SIDE_MAX_DEG))
        if now < sidepd_enable_at:
            side_pd = 0.0

        # ===== PD de giro (frontal) =====
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
                turn_pd = (Kp_turn * err_turn) + (Kd_turn * d_err_turn)
                prev_err_turn = err_turn
        if selected_turn is not None and effective_turning:
            bias = TURN_BIAS_DEG if (selected_turn == 'R') else -TURN_BIAS_DEG

        # ===== TOP STOP =====
        if topstop_armed and (not topstop_done):
            tx0,ty0,tx1,ty1 = topStopR
            roi_top = frame[ty0:ty1, tx0:tx1]
            m_top = black_mask_combined(roi_top, kernel)
            area_top = float(cv2.countNonZero(m_top))
            cv2.addWeighted(roi_top,0.65,cv2.cvtColor(m_top,cv2.COLOR_GRAY2BGR),0.35,0,roi_top)

            if area_top >= float(P["TopStop_area_T"]):
                steer_cmd = SERVO_CENTER
                send(ser, f"STEER:{steer_cmd}")
                send(ser, f"REV:{int(P['TopStop_back_ms'])}")
                time.sleep(max(0.0, P["TopStop_back_ms"]/1000.0))
                send(ser, "STOP")
                armed=False
                topstop_done=True
                print(f"[TOPSTOP] Triggered (area={int(area_top)}). REV {P['TopStop_back_ms']} ms + STOP.")

        # ===== Mezcla y envío =====
        if armed:
            if effective_turning:
                steer = SERVO_CENTER + (bias + turn_pd + SIDE_BLEND_IN_TURN*side_pd)
            else:
                steer = SERVO_CENTER + side_pd
            steer = int(round(clamp(steer, SERVO_LEFT_LIM, SERVO_RIGHT_LIM)))
            if (now - last_tx_ts) >= TX_PERIOD:
                send(ser, f"STEER:{steer}")
                last_tx_ts = now

        # HUD
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
            save_config(read_tuner()); print("[CFG] Guardado manual (s).")
        elif key == ord('r'):
            set_trackbar_positions(DEFAULTS); print("[CFG] Reset a defaults.")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Bye.")

if __name__ == "__main__":
    main()
