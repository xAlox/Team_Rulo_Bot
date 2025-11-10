#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Team Rulo Bot — Runtime (LAB models + HSV fallback, sin sliders de color)
- Evasión AVOID (PD, contornos, ROIs).
- Giro de esquina con ROI central de negro (TURN_CENTER) con histéresis.
- Prioridad: AVOID > TURN > RUN.
- WALL_L / WALL_R (solo AVOID): bloquean giro hacia la pared negra.
- ANTIBLACK: ROI fijo definido en el código (no aparece en TUNING).
- Color: usa LAB (Mahalanobis) si hay modelo en ~/rulobot_lab_color_model.json.
         Si no, fallback HSV LEÍDO de ~/.rulobot_vision_tuning.json (retrocompatible), sin sliders de color.
- Tuning compartido (sin color) se guarda con 's' en ~/.rulobot_vision_tuning.json y se comparte con Open.
"""

import cv2, numpy as np, time, signal, json, os
import serial, serial.tools.list_ports

# ===================== Camera / UI =====================
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
FONT = cv2.FONT_HERSHEY_SIMPLEX

# ===================== Servo & TX =====================
SERVO_CENTER    = 84
SERVO_LEFT_LIM  = 24
SERVO_RIGHT_LIM = 144
TX_PERIOD       = 0.015  # ~66 Hz

# ===================== Paths =====================
LAB_MODEL_PATH  = os.path.expanduser("~/rulobot_lab_color_model.json")
HSV_TUNING_PATH = os.path.expanduser("~/.rulobot_vision_tuning.json")  # archivo compartido entre códigos

# ===================== ROIs (fracciones 0..1) =====================
ROIS = {
    "LINE_BOT":    (0.25, 0.87, 0.75, 0.95),  # detección de línea (fija sentido R/L)
    "CENTER_BAND": (0.00, 0.42, 1.00, 0.95),  # evasión de pilares
    "TURN_CENTER": (0.45, 0.50, 0.55, 0.85),  # negro central para giro de esquina
}
# LATERALES pared (solo AVOID)
WALL_L_ROI  = (0.13, 0.85, 0.24, 1.00)
WALL_R_ROI  = (0.76, 0.85, 0.87, 1.00)

# ===== ANTIBLACK (fijo) =====
ANTIBLACK_ROI = (0.37, 0.78, 0.63, 0.87)
AB_AREA_T     = 1200
AB_CORR_DEG   = 3

# ===================== Lógicas de color para decisión de línea (constantes) =====================
O_MIN_FRAC = 0.015   # fracción mínima naranja en LINE_BOT
O_MIN_H    = 7       # altura mínima bbox
O_STABLE   = 1       # frames consecutivos
B_MIN_FRAC = 0.015   # fracción mínima azul
B_MIN_H    = 7
B_STABLE   = 2
COLOR_DECISION_MARGIN = 1.35
LINE_COOLDOWN_S = 3.0
NUM_LINES_TO_STOP = 13

# ===================== Defaults (TUNING sin color) =====================
DEFAULTS = {
    # ------- Global image -------
    "GainS_global": 120, "GainV_global": 90, "Gamma_x100": 100,

    # ------- Negro (solo fallback mixto en mask_black) -------
    "L_min": 0, "L_max": 95, "A_tol": 25, "B_tol": 25,
    "Blk_Vmax": 120, "Blk_Smax": 120,

    # ------- Morphology -------
    "Kernel": 5,

    # ------- Pilares -------
    "PillarMinArea": 600,

    # ------- Target ROIs para pilares (en % 0..100) -------
    "TRED_x0":  5, "TRED_y0": 46, "TRED_x1": 16, "TRED_y1": 78,
    "TGRN_x0": 84, "TGRN_y0": 46, "TGRN_x1": 97, "TGRN_y1": 78,

    # ------- PD Avoidance (target tracking) -------
    "Kp_target_x100": 30, "Kd_target_x100": 40, "TargetMax_deg": 32,

    # ------- TURN CENTER (giro con negro central) -------
    "TurnBias_deg": 28,
    "TurnCenter_area_T": 1400,
    "TurnCenter_hyst_x100": 60,

    # ------- Wall guards (solo AVOID) -------
    "WALL_area_T":   700,
    "WALL_corr_deg": 8,
}

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

# ===================== Helpers =====================
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def frac_to_rect(img, f):
    h,w = img.shape[:2]
    return (int(f[0]*w), int(f[1]*h), int(f[2]*w), int(f[3]*h))

def pct_rect_to_abs(px0,py0,px1,py1):
    x0 = int(np.clip(px0,0,100)/100.0 * FRAME_W)
    y0 = int(np.clip(py0,0,100)/100.0 * FRAME_H)
    x1 = int(np.clip(px1,0,100)/100.0 * FRAME_W)
    y1 = int(np.clip(py1,0,100)/100.0 * FRAME_H)
    x0,x1 = min(x0,x1), max(x0,x1)
    y0,y1 = min(y0,y1), max(y0,y1)
    return (x0,y0,x1,y1)

def draw_roi(frame, rect, label, color=(255,200,0)):
    x0,y0,x1,y1 = rect
    cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
    cv2.putText(frame,label,(x0+4,y0+18),FONT,0.55,color,2,cv2.LINE_AA)

def largest_contour(mask):
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not cnts: return None, 0
    best = max(cnts, key=cv2.contourArea)
    return best, cv2.contourArea(best)

# ===================== Carga modelos / tuning compartido =====================
def load_json(path):
    try:
        with open(path,"r") as f:
            return json.load(f)
    except:
        return None

LAB = load_json(LAB_MODEL_PATH)      # dict o None
HSV_FALLBACK = load_json(HSV_TUNING_PATH) or {}  # solo lectura para HSV si falta LAB

def get_lab_model(name):
    if not LAB: return None
    aliases = {
        "green": ["green","verde","pillar_green","G"],
        "red":   ["red","rojo","pillar_red","R"],
        "blue":  ["blue","azul","line_blue","B"],
        "orange":["orange","naranja","line_orange","O"],
        "black": ["black","negro","wall","floor_black"],
    }
    for key, arr in aliases.items():
        if name.lower() in arr:
            return LAB.get(key, None)
    return LAB.get(name, None)

# ========= persistencia (compartida con Open) =========
def save_config(vals, path=HSV_TUNING_PATH):
    """Guarda SOLO claves de no-color para compartir tuning entre códigos."""
    try:
        # Mezclamos con lo previo para no borrar otras claves que existan en el archivo (ej. HSV viejos)
        existing = load_json(path) or {}
        # Escribimos por encima las no-color actuales:
        for k in DEFAULTS.keys():
            existing[k] = vals.get(k, DEFAULTS[k])
        with open(path, "w") as f:
            json.dump(existing, f, indent=2)
        print("[CFG] Guardado:", path)
    except Exception as e:
        print("[CFG] Error guardando:", e)

def load_config(path=HSV_TUNING_PATH):
    try:
        data = load_json(path)
        if not data: return None
        print("[CFG] Cargado:", path)
        # devolvemos solo las claves relevantes (no-color)
        return {**DEFAULTS, **{k:v for k,v in data.items() if k in DEFAULTS}}
    except Exception:
        return None

# ===================== Operaciones de imagen =====================
def apply_global_gains(bgr, gs_x100, gv_x100, gamma_x100):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    s = np.clip((s.astype(np.float32) * (gs_x100/100.0)), 0, 255)
    v = np.clip(v.astype(np.float32) * (gv_x100/100.0), 0, 255)
    gamma = max(0.3, gamma_x100/100.0)
    v = (np.clip(v/255.0, 0, 1)**gamma)*255.0
    hsv_adj = cv2.merge([h, s.astype(np.uint8), v.astype(np.uint8)])
    return cv2.cvtColor(hsv_adj, cv2.COLOR_HSV2BGR)

def mahalanobis_mask(bgr, model):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    A = lab[:,:,1].astype(np.float32)
    B = lab[:,:,2].astype(np.float32)
    mean = np.array(model["mean"], dtype=np.float32)
    cov_inv = np.array(model["cov_inv"], dtype=np.float32)
    tau2 = float(model["tau2"])
    X = np.stack([A - mean[0], B - mean[1]], axis=-1)
    dist2 = np.einsum('...i,ij,...j->...', X, cov_inv, X)
    return (dist2 <= tau2).astype(np.uint8) * 255

def mask_black(bgr, P):
    m = get_lab_model("black")
    if m is not None:
        return mahalanobis_mask(bgr, m)

    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L = lab[:,:,0]; A = lab[:,:,1]; B = lab[:,:,2]
    mL = cv2.inRange(L, P["L_min"], P["L_max"])
    mA = cv2.inRange(A, 128-P["A_tol"], 128+P["A_tol"])
    mB = cv2.inRange(B, 128-P["B_tol"], 128+P["B_tol"])
    mLAB = cv2.bitwise_and(cv2.bitwise_and(mL,mA), mB)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    S = hsv[:,:,1]; V = hsv[:,:,2]
    mHSV = cv2.inRange(S, 0, P["Blk_Smax"]) & cv2.inRange(V, 0, P["Blk_Vmax"])

    k = cv2.getStructuringElement(cv2.MORPH_RECT,(P["Kernel"],P["Kernel"]))
    m = cv2.bitwise_and(mLAB, mHSV)
    m = cv2.GaussianBlur(m,(5,5),0); m = cv2.erode(m,k,1); m = cv2.dilate(m,k,1)
    return m

def _hsv_range_from_fallback(name):
    if name == "orange":
        return (HSV_FALLBACK.get("O_Hmin",5),  HSV_FALLBACK.get("O_Hmax",30),
                HSV_FALLBACK.get("O_Smin",80), HSV_FALLBACK.get("O_Vmin",60))
    if name == "blue":
        return (HSV_FALLBACK.get("B_Hmin",100), HSV_FALLBACK.get("B_Hmax",130),
                HSV_FALLBACK.get("B_Smin",110),  HSV_FALLBACK.get("B_Vmin",40))
    if name == "green":
        return (HSV_FALLBACK.get("G_Hmin",40), HSV_FALLBACK.get("G_Hmax",85),
                HSV_FALLBACK.get("G_Smin",70), HSV_FALLBACK.get("G_Vmin",70))
    if name == "red1":
        return (HSV_FALLBACK.get("R1_Hmin",0),  HSV_FALLBACK.get("R1_Hmax",10),
                HSV_FALLBACK.get("R_Smin",120),  HSV_FALLBACK.get("R_Vmin",70))
    if name == "red2":
        return (HSV_FALLBACK.get("R2_Hmin",170), HSV_FALLBACK.get("R2_Hmax",180),
                HSV_FALLBACK.get("R_Smin",120),    HSV_FALLBACK.get("R_Vmin",70))
    return (0,0,0,0)

def mask_color(bgr, name):
    m = get_lab_model(name)
    if m is not None:
        return mahalanobis_mask(bgr, m)
    if name == "red":
        h1min,h1max,smin,vmin = _hsv_range_from_fallback("red1")
        h2min,h2max,_,_        = _hsv_range_from_fallback("red2")
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (h1min,smin,vmin), (h1max,255,255))
        m2 = cv2.inRange(hsv, (h2min,smin,vmin), (h2max,255,255))
        return cv2.bitwise_or(m1,m2)
    else:
        hmin,hmax,smin,vmin = _hsv_range_from_fallback(name)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, (hmin,smin,vmin), (hmax,255,255))

# ---- Detecciones ----
def detect_orange(proc_bgr, rect, kernel_size):
    x0,y0,x1,y1 = rect
    roi = proc_bgr[y0:y1, x0:x1]
    mask = mask_color(roi, "orange")
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(kernel_size,kernel_size))
    mask = cv2.GaussianBlur(mask,(5,5),0); mask = cv2.erode(mask,k,1); mask = cv2.dilate(mask,k,1)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best=None
    if cnts:
        best = max(cnts, key=cv2.contourArea); cv2.drawContours(roi, [best], -1, (0,165,255), 2)
    area = (cv2.contourArea(best) if best is not None else 0.0)
    hbox = 0
    if best is not None:
        x,y,w,h = cv2.boundingRect(best); hbox=h
        cv2.rectangle(roi,(x,y),(x+w,y+h),(0,140,255),2)
    return area, hbox

def detect_blue(proc_bgr, rect, kernel_size):
    x0,y0,x1,y1 = rect
    roi = proc_bgr[y0:y1, x0:x1]
    mask = mask_color(roi, "blue")
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(kernel_size,kernel_size))
    mask = cv2.GaussianBlur(mask,(5,5),0); mask = cv2.erode(mask,k,1); mask = cv2.dilate(mask,k,1)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best=None
    if cnts:
        best = max(cnts, key=cv2.contourArea); cv2.drawContours(roi, [best], -1, (255,120,0), 2)
    area = (cv2.contourArea(best) if best is not None else 0.0)
    hbox = 0
    if best is not None:
        x,y,w,h = cv2.boundingRect(best); hbox=h
        cv2.rectangle(roi,(x,y),(x+w,y+h),(255,120,0),2)
    return area, hbox

def detect_pillars_both(proc_bgr, det_rect, kernel_size, min_area):
    x0,y0,x1,y1 = det_rect
    roi = proc_bgr[y0:y1, x0:x1].copy()

    mG = mask_color(roi, "green")
    mR = mask_color(roi, "red")

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    def refine(m):
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
        return m
    mG = refine(mG); mR = refine(mR)

    cG,aG = largest_contour(mG)
    cR,aR = largest_contour(mR)

    out = {'R':(None,0.0), 'G':(None,0.0)}
    if cG is not None and aG>=min_area:
        x,y,w,h = cv2.boundingRect(cG); cx = x+w//2
        cv2.rectangle(roi,(x,y),(x+w,y+h),(0,255,0),2); cv2.circle(roi,(cx,y+h//2),4,(255,255,255),-1)
        out['G'] = (cx+x0, float(aG))
    if cR is not None and aR>=min_area:
        x,y,w,h = cv2.boundingRect(cR); cx = x+w//2
        cv2.rectangle(roi,(x,y),(x+w,y+h),(0,0,255),2); cv2.circle(roi,(cx,y+h//2),4,(255,255,255),-1)
        out['R'] = (cx+x0, float(aR))

    proc_bgr[y0:y1, x0:x1] = roi
    return out

# ===================== TUNING UI (solo no-color) =====================
def build_tuner():
    cv2.namedWindow("TUNING", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("TUNING", 560, 560)
    # Global
    cv2.createTrackbar("GainS_gl x100", "TUNING", DEFAULTS["GainS_global"], 300, lambda x: None)
    cv2.createTrackbar("GainV_gl x100", "TUNING", DEFAULTS["GainV_global"], 300, lambda x: None)
    cv2.createTrackbar("Gamma x100",    "TUNING", DEFAULTS["Gamma_x100"],  300, lambda x: None)
    # Kernel / pilares / PD
    cv2.createTrackbar("Kernel",         "TUNING", DEFAULTS["Kernel"],         15,   lambda x: None)
    cv2.createTrackbar("PillarMinArea",  "TUNING", DEFAULTS["PillarMinArea"],  60000,lambda x: None)
    cv2.createTrackbar("Kp_tgt x100",    "TUNING", DEFAULTS["Kp_target_x100"], 400,  lambda x: None)
    cv2.createTrackbar("Kd_tgt x100",    "TUNING", DEFAULTS["Kd_target_x100"], 400,  lambda x: None)
    cv2.createTrackbar("TargetMax deg",  "TUNING", DEFAULTS["TargetMax_deg"],  60,   lambda x: None)
    # Turn center
    cv2.createTrackbar("TurnBias deg",        "TUNING", DEFAULTS["TurnBias_deg"],        60,    lambda x: None)
    cv2.createTrackbar("TurnCenter area",     "TUNING", DEFAULTS["TurnCenter_area_T"],   20000, lambda x: None)
    cv2.createTrackbar("TurnCenter hyst x100","TUNING", DEFAULTS["TurnCenter_hyst_x100"],100,   lambda x: None)

def read_tuner():
    g = lambda n: cv2.getTrackbarPos(n,"TUNING")
    return dict(
        GainS_global=g("GainS_gl x100"),
        GainV_global=g("GainV_gl x100"),
        Gamma_x100=g("Gamma x100"),
        Kernel=max(1, g("Kernel")|1),
        PillarMinArea=g("PillarMinArea"),
        Kp_target_x100=g("Kp_tgt x100"),
        Kd_target_x100=g("Kd_tgt x100"),
        TargetMax_deg=g("TargetMax deg"),
        TurnBias_deg=g("TurnBias deg"),
        TurnCenter_area_T=g("TurnCenter area"),
        TurnCenter_hyst_x100=g("TurnCenter hyst x100"),
    )

def set_trackbar_positions(cfg):
    """Coloca sliders a partir del dict cfg (solo no-color)."""
    c = (DEFAULTS if cfg is None else {**DEFAULTS, **cfg})
    def sb(name, val): cv2.setTrackbarPos(name, "TUNING", int(val))
    sb("GainS_gl x100", c["GainS_global"])
    sb("GainV_gl x100", c["GainV_global"])
    sb("Gamma x100",    c["Gamma_x100"])
    sb("Kernel",        c["Kernel"])
    sb("PillarMinArea", c["PillarMinArea"])
    sb("Kp_tgt x100",   c["Kp_target_x100"])
    sb("Kd_tgt x100",   c["Kd_target_x100"])
    sb("TargetMax deg", c["TargetMax_deg"])
    sb("TurnBias deg",  c["TurnBias_deg"])
    sb("TurnCenter area",      c["TurnCenter_area_T"])
    sb("TurnCenter hyst x100", c["TurnCenter_hyst_x100"])

# ===================== Main =====================
_stop=False
def _sigint(_s,_f):
    global _stop; _stop=True

def main():
    signal.signal(signal.SIGINT, _sigint)

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened(): raise SystemExit("[ERROR] No camera.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    ser = open_serial()
    win = "RuloBot — LAB Runtime + HSV fallback (no color sliders)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 960, 720)

    # Build tuners y cargar config compartida
    build_tuner()
    cfg_loaded = load_config()
    set_trackbar_positions(cfg_loaded)

    # State
    armed=False
    selected_turn=None  # None / 'R' / 'L'
    mode = "RUN"        # RUN | TURN | AVOID
    line_count = 0
    last_line_ts = 0.0

    prev_err_target = 0.0
    last_tx = 0.0

    rx_buf=b""
    t0=time.time(); n=0; fps=0.0

    while not _stop:
        ok, frame_raw = cap.read()
        if not ok: continue
        n+=1; t1=time.time()
        if (t1-t0)>=1.0: fps=n/(t1-t0); n=0; t0=t1

        # RX GO/STOP
        if ser and ser.in_waiting:
            try:
                rx_buf += ser.read(ser.in_waiting)
                while b"\n" in rx_buf:
                    line, rx_buf = rx_buf.split(b"\n",1)
                    s=line.decode(errors='ignore').strip().upper()
                    if s=="GO":
                        armed=True
                        line_count = 0
                        selected_turn=None
                        mode="RUN"
                        send(ser, f"STEER:{SERVO_CENTER}")
                        print("[INFO] ARMED by GO")
                    elif s=="STOP":
                        armed=False
                        mode="RUN"
                        print("[INFO] DISARMED by STOP")
            except Exception as e:
                print(f"[WARN] Serial read error: {e}")

        # leer tuning (dinámico) no-color
        P = {**DEFAULTS, **read_tuner()}

        frame = apply_global_gains(frame_raw, P["GainS_global"], P["GainV_global"], P["Gamma_x100"])

        # ROIs
        lineR = frac_to_rect(frame, ROIS["LINE_BOT"])
        detR  = frac_to_rect(frame, ROIS["CENTER_BAND"])
        turnR = frac_to_rect(frame, ROIS["TURN_CENTER"])
        ABR   = frac_to_rect(frame, ANTIBLACK_ROI)
        wallL = frac_to_rect(frame, WALL_L_ROI)
        wallR = frac_to_rect(frame, WALL_R_ROI)

        # Draw ROIs
        draw_roi(frame, lineR, "LINE_BOT")
        draw_roi(frame, detR,  "CENTER_BAND", (0,255,255))
        draw_roi(frame, turnR, "TURN_CENTER", (0,200,255))
        draw_roi(frame, ABR,   "ANTIBLACK (fixed)", (255,0,255))
        draw_roi(frame, wallL, "WALL_L", (0,255,0))
        draw_roi(frame, wallR, "WALL_R", (0,255,0))

        now = time.time()

        # ---- LINE lock (naranja/azul) + conteo ----
        x0,y0,x1,y1 = lineR
        roi_area = max(1,(x1-x0)*(y1-y0))
        aO,hO = detect_orange(frame, lineR, P["Kernel"])
        aB,hB = detect_blue(frame,  lineR, P["Kernel"])
        fracO = aO/roi_area; fracB = aB/roi_area

        saw_orange = (fracO >= O_MIN_FRAC and hO >= O_MIN_H)
        saw_blue   = (fracB >= B_MIN_FRAC and hB >= B_MIN_H)

        if armed and (selected_turn is None):
            if not hasattr(main, "_o_hits"): main._o_hits=0
            if not hasattr(main, "_b_hits"): main._b_hits=0
            accept_orange = saw_orange and (fracO >= fracB*COLOR_DECISION_MARGIN)
            accept_blue   = saw_blue   and (fracB >= fracO*COLOR_DECISION_MARGIN)
            main._o_hits = (main._o_hits+1) if accept_orange else 0
            main._b_hits = (main._b_hits+1) if accept_blue else 0
            if main._o_hits >= O_STABLE: selected_turn='R'; print("[DIR] LOCKED: ORANGE → RIGHT")
            elif main._b_hits >= B_STABLE: selected_turn='L'; print("[DIR] LOCKED: BLUE → LEFT")

        if armed and (saw_orange or saw_blue):
            if (now - last_line_ts) >= LINE_COOLDOWN_S:
                line_count += 1
                last_line_ts = now
                print(f"[LINE] count = {line_count}")
                if line_count >= NUM_LINES_TO_STOP:
                    send(ser, "STOP"); armed=False; print("[INFO] 12 lines reached — STOP")

        # ---- Pillars (EVASIÓN) ----
        pillars = detect_pillars_both(frame, detR, P["Kernel"], P["PillarMinArea"])
        R_cx,R_area = pillars['R']
        G_cx,G_area = pillars['G']
        pillar_active = ((R_area>=P["PillarMinArea"]) or (G_area>=P["PillarMinArea"]))

        # ---- TURN_CENTER negro con histéresis ----
        tx0,ty0,tx1,ty1 = turnR
        roi_turn = frame[ty0:ty1, tx0:tx1]
        m_turn = mask_black(roi_turn, P)
        cntT, area_turn = largest_contour(m_turn)
        if cntT is not None: cv2.drawContours(roi_turn, [cntT], -1, (0,200,255), 2)
        cv2.addWeighted(roi_turn,0.65,cv2.cvtColor(m_turn,cv2.COLOR_GRAY2BGR),0.35,0,roi_turn)

        if pillar_active:
            mode = "AVOID"
        elif (selected_turn is not None):
            enter = (area_turn >= float(P["TurnCenter_area_T"]))
            exit_ = (area_turn <= float(P["TurnCenter_area_T"])*(max(10,P["TurnCenter_hyst_x100"])/100.0))
            if not hasattr(main, "_turning"): main._turning = False
            if (not main._turning) and enter: main._turning = True
            elif main._turning and exit_:     main._turning = False
            mode = "TURN" if main._turning else "RUN"
        else:
            mode = "RUN"

        # ======= Compute steer =======
        steer_cmd = SERVO_CENTER

        def area_black_rect(rect):
            x0,y0,x1,y1 = rect
            roi = frame[y0:y1, x0:x1]
            m = mask_black(roi, P)
            cv2.addWeighted(roi,0.65,cv2.cvtColor(m,cv2.COLOR_GRAY2BGR),0.35,0,roi)
            return float(cv2.countNonZero(m))

        if armed:
            if mode == "AVOID":
                # PD evasión (target tracking por cx)
                ctrl_color = 'R' if (R_area >= G_area and R_area>0) else ('G' if G_area>0 else None)
                err = 0.0
                if ctrl_color == 'R' and R_cx is not None:
                    TRED = pct_rect_to_abs(P["TRED_x0"],P["TRED_y0"],P["TRED_x1"],P["TRED_y1"])
                    tgt_cx = (TRED[0] + TRED[2])//2
                    err = float(R_cx - tgt_cx)
                elif ctrl_color == 'G' and G_cx is not None:
                    TGRN = pct_rect_to_abs(P["TGRN_x0"],P["TGRN_y0"],P["TGRN_x1"],P["TGRN_y1"])
                    tgt_cx = (TGRN[0] + TGRN[2])//2
                    err = float(G_cx - tgt_cx)

                d_err = err - prev_err_target
                prev_err_target = err
                Kp = P["Kp_target_x100"]/100.0
                Kd = P["Kd_target_x100"]/100.0
                u = (Kp*err) + (Kd*d_err)
                u = float(clamp(u, -P["TargetMax_deg"], P["TargetMax_deg"]))

                # AntiBlack (central) — ROI y parámetros fijos
                ax0,ay0,ax1,ay1 = ABR
                ab_roi = frame[ay0:ay1, ax0:ax1]
                m_black = mask_black(ab_roi, P)
                ab_area = float(cv2.countNonZero(m_black))
                cv2.addWeighted(ab_roi,0.65,cv2.cvtColor(m_black,cv2.COLOR_GRAY2BGR),0.35,0,ab_roi)
                if ab_area >= float(AB_AREA_T) and ctrl_color is not None:
                    corr = float(AB_CORR_DEG)
                    u += (-corr if ctrl_color=='R' else +corr)
                    u = float(clamp(u, -P["TargetMax_deg"], P["TargetMax_deg"]))
                    cv2.putText(frame, f"ANTIBLACK(FIXED) ({ctrl_color}) a={int(ab_area)}",
                                (10, 66), FONT, 0.55, (255,0,255), 2, cv2.LINE_AA)

                # Wall guards (solo AVOID)
                area_wall_l = area_black_rect(wallL)
                area_wall_r = area_black_rect(wallR)
                wallL_hit = (area_wall_l >= DEFAULTS["WALL_area_T"])
                wallR_hit = (area_wall_r >= DEFAULTS["WALL_area_T"])
                wall_corr = float(DEFAULTS["WALL_corr_deg"])

                if wallL_hit:
                    if u < 0: u = 0.0
                    u += wall_corr
                    cv2.putText(frame, "WALL_L BLOCK → RIGHT", (10, 88), FONT, 0.55, (0,255,0), 2, cv2.LINE_AA)
                if wallR_hit:
                    if u > 0: u = 0.0
                    u -= wall_corr
                    cv2.putText(frame, "WALL_R BLOCK → LEFT",  (10,110), FONT, 0.55, (0,255,0), 2, cv2.LINE_AA)

                steer_cmd = SERVO_CENTER + int(round(u))
                cv2.putText(frame, f"MODE:AVOID  u={u:.1f}",
                            (10, 44), FONT, 0.55, (0,255,255), 2, cv2.LINE_AA)

            elif mode == "TURN":
                bias = float(P["TurnBias_deg"]) if (selected_turn=='R') else -float(P["TurnBias_deg"])
                steer_cmd = SERVO_CENTER + int(round(bias))
                cv2.putText(frame, f"MODE:TURN  dir={selected_turn} area={int(area_turn)} bias={bias:.1f}",
                            (10, 44), FONT, 0.55, (0,200,255), 2, cv2.LINE_AA)

            else:
                steer_cmd = SERVO_CENTER
                cv2.putText(frame, "MODE:RUN",
                            (10, 44), FONT, 0.55, (0,255,255), 2, cv2.LINE_AA)

            # Clamp + TX
            steer_cmd = int(round(clamp(steer_cmd, SERVO_LEFT_LIM, SERVO_RIGHT_LIM)))
            if (time.time()-last_tx) >= TX_PERIOD:
                send(ser, f"STEER:{steer_cmd}")
                last_tx = time.time()

        # HUD
        dir_txt = {None:"WAIT_LINE", 'R':"RIGHT", 'L':"LEFT"}[selected_turn]
        hud = f"{FRAME_W}x{FRAME_H}  ~{fps:.1f}fps  armed:{int(armed)}  DIR:{dir_txt}  mode:{mode}  lines:{line_count}/{NUM_LINES_TO_STOP}"
        cv2.putText(frame, hud, (10,20), FONT, 0.58, (255,255,255), 2, cv2.LINE_AA)
        cv2.imshow(win, frame)

        key = cv2.waitKey(1) & 0xFF
        if key==ord('q'):
            break
        elif key==ord('s'):
            # Guardar tuning compartido
            current = read_tuner()
            save_config({**DEFAULTS, **current})
            print("[CFG] Saved (s).")
        elif key==ord('r'):
            set_trackbar_positions(DEFAULTS)
            print("[CFG] Reset defaults.")

    cap.release(); cv2.destroyAllWindows()
    if ser:
        try: ser.close()
        except: pass
    print("[INFO] Bye.")

if __name__ == "__main__":
    main()
