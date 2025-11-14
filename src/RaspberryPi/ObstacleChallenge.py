#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Team Rulo Bot — Exit (WHITE L/R) -> Obstacle FULL (RGB/M + TURN + RUN)

Fase 1 (Salida):
  - Decide EXT (LEFT/RIGHT) por blancos laterales y envía EXT:* al Nano.
  - Espera "[SEQ] END" del Nano (o tecla X para saltar).
  - La decisión de sentido (LEFT/RIGHT) se CONGELA cuando el Nano envía "GO"
    (botón), o en su defecto al hacer skip ('x') o al recibir [SEQ] END.
  - Al pasar a Obstacle: TURN_OFF + servo al centro.

Fase 2 (Obstacle):
  - Evasión de pilares R/G/M con prioridad MAG sobre RED (red_clean = red - magenta)
  - TURN_CENTER con histéresis + lockout post-salida + timeout anti-pegado
  - RUN: centrado lateral usando pared = negro + magenta (gating por proximidad)
  - Líneas naranja/azul solo cuentan (no deciden sentido)
  - ROIs y overlays SIEMPRE visibles
  - Teclas: q (salir), s (guardar CFG), r (reset CFG), m (visor MAG/RED), x (saltar salida)
"""

import cv2, numpy as np, time, json, os, serial, serial.tools.list_ports, signal

# ===================== Cámara / UI =====================
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
FONT = cv2.FONT_HERSHEY_SIMPLEX
MIRROR_X = False            # pon True si tu cámara está en espejo
TURN_DIR_SIGN = +1          # usa -1 si tu TURN sale invertido

# ===================== Serial =====================
def open_serial(baud=115200):
    for p in ("/dev/ttyUSB0","/dev/ttyACM0"):
        try: return serial.Serial(p, baudrate=baud, timeout=0.02)
        except: pass
    for p in serial.tools.list_ports.comports():
        try: return serial.Serial(p.device, baudrate=baud, timeout=0.02)
        except: pass
    print("[WARN] No serial; vision-only.")
    return None

def send(ser, s):
    try:
        if ser: ser.write((s+"\n").encode())
    except Exception as e:
        print("[WARN] Serial write:", e)

# ===================== Paths / Defaults =====================
LAB_MODEL_PATH  = os.path.expanduser("~/rulobot_lab_color_model.json")
CFG_PATH        = os.path.expanduser("~/.rulobot_vision_tuning.json")  # solo NO-color

DEFAULTS = {
    # ------- Global image -------
    "GainS_global": 120, "GainV_global": 90, "Gamma_x100": 100,
    # ------- Negro fallback -------
    "L_min": 0, "L_max": 95, "A_tol": 25, "B_tol": 25, "Blk_Vmax": 120, "Blk_Smax": 120,
    # ------- Morphology -------
    "Kernel": 5,
    # ------- Pilares -------
    "PillarMinArea": 600,
    # ------- Target ROIs (en %) -------
    "TRED_x0":  5, "TRED_y0": 46, "TRED_x1": 16, "TRED_y1": 78,
    "TGRN_x0": 84, "TGRN_y0": 46, "TGRN_x1": 97, "TGRN_y1": 78,
    "TMAG_x0": 45, "TMAG_y0": 46, "TMAG_x1": 55, "TMAG_y1": 78,
    # ------- PD Avoid (target tracking) -------
    "Kp_target_x100": 30, "Kd_target_x100": 40, "TargetMax_deg": 32,
    # ------- TURN CENTER -------
    "TurnBias_deg": 28, "TurnCenter_area_T": 400, "TurnCenter_hyst_x100": 60,
    "Turn_lockout_s": 1.0,
    "Turn_max_time_s": 1.2,
    # ------- Wall Guards -------
    "WALL_area_T": 700, "WALL_corr_deg": 8,
    # ------- RUN side-centering -------
    "Run_Kp_x100": 8, "Run_Kd_x100": 18, "Run_Max_deg": 9,
    # ------- MAG como pared (GATING por proximidad) -------
    "MAG_as_wall_gain_x100": 100,
    "Mag_avoid_y0_pct": 72,
    "Mag_avoid_hmin_px": 42,
    "Mag_avoid_area_min": 1200,
    "Mag_near_stable_fr": 3,
    "Mag_gain_far_x100": 15,
    "Mag_gain_near_x100": 100,
    "Mag_gain_ramp": 1,
    "Mag_ignore_after_exit_s": 1.2,
}

# ===================== CFG helpers =====================
def load_json(path):
    try:
        with open(path,"r") as f: return json.load(f)
    except: return None

LAB = load_json(LAB_MODEL_PATH) or {}
def get_lab_model(name):
    aliases = {
        "green":   ["green","verde","pillar_green","G"],
        "red":     ["red","rojo","pillar_red","R"],
        "magenta": ["magenta","morado","fucsia","M","pillar_magenta","parking","parqueo"],
        "blue":    ["blue","azul","line_blue","B"],
        "orange":  ["orange","naranja","line_orange","O"],
        "black":   ["black","negro","wall","floor_black"],
        "white":   ["white","blanco"],
    }
    for key, arr in aliases.items():
        if name.lower() in arr: return LAB.get(key, None)
    return LAB.get(name, None)

def load_cfg():
    data = load_json(CFG_PATH) or {}
    return {**DEFAULTS, **{k:data.get(k, DEFAULTS[k]) for k in DEFAULTS}}

def save_cfg(vals):
    try:
        with open(CFG_PATH,"w") as f: json.dump(vals, f, indent=2)
        print("[CFG] Guardado:", CFG_PATH)
    except Exception as e:
        print("[CFG] Error guardando:", e)

# ===================== TUNING UI =====================
def build_tuner():
    cv2.namedWindow("TUNING", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("TUNING", 600, 720)
    # Global
    cv2.createTrackbar("GainS x100","TUNING", DEFAULTS["GainS_global"], 300, lambda x: None)
    cv2.createTrackbar("GainV x100","TUNING", DEFAULTS["GainV_global"], 300, lambda x: None)
    cv2.createTrackbar("Gamma x100","TUNING", DEFAULTS["Gamma_x100"],  300, lambda x: None)
    # Kernel / Pilares
    cv2.createTrackbar("Kernel","TUNING", DEFAULTS["Kernel"], 15, lambda x: None)
    cv2.createTrackbar("PillarMinArea","TUNING", DEFAULTS["PillarMinArea"], 60000, lambda x: None)
    # PD evasión
    cv2.createTrackbar("Kp_tgt x100","TUNING", DEFAULTS["Kp_target_x100"], 400, lambda x: None)
    cv2.createTrackbar("Kd_tgt x100","TUNING", DEFAULTS["Kd_target_x100"], 400, lambda x: None)
    cv2.createTrackbar("TargetMax deg","TUNING", DEFAULTS["TargetMax_deg"], 60, lambda x: None)
    # Turn center
    cv2.createTrackbar("TurnBias deg","TUNING", DEFAULTS["TurnBias_deg"], 60, lambda x: None)
    cv2.createTrackbar("TurnC area","TUNING", DEFAULTS["TurnCenter_area_T"], 20000, lambda x: None)
    cv2.createTrackbar("TurnC hyst x100","TUNING", DEFAULTS["TurnCenter_hyst_x100"], 100, lambda x: None)
    cv2.createTrackbar("Turn lock s","TUNING", int(DEFAULTS["Turn_lockout_s"]*10), 30, lambda x: None)
    cv2.createTrackbar("Turn max s","TUNING", int(DEFAULTS["Turn_max_time_s"]*10), 50, lambda x: None)
    # Wall guards
    cv2.createTrackbar("WALL area T","TUNING", DEFAULTS["WALL_area_T"], 20000, lambda x: None)
    cv2.createTrackbar("WALL corr deg","TUNING", DEFAULTS["WALL_corr_deg"], 40, lambda x: None)
    # RUN
    cv2.createTrackbar("Run Kp x100","TUNING", DEFAULTS["Run_Kp_x100"], 400, lambda x: None)
    cv2.createTrackbar("Run Kd x100","TUNING", DEFAULTS["Run_Kd_x100"], 400, lambda x: None)
    cv2.createTrackbar("Run Max deg","TUNING", DEFAULTS["Run_Max_deg"], 60, lambda x: None)
    # Magenta gating
    cv2.createTrackbar("Mag y0 %","TUNING", DEFAULTS["Mag_avoid_y0_pct"], 100, lambda x: None)
    cv2.createTrackbar("Mag hmin px","TUNING", DEFAULTS["Mag_avoid_hmin_px"], 200, lambda x: None)
    cv2.createTrackbar("Mag area min","TUNING", DEFAULTS["Mag_avoid_area_min"], 50000, lambda x: None)
    cv2.createTrackbar("Mag near fr","TUNING", DEFAULTS["Mag_near_stable_fr"], 10, lambda x: None)
    cv2.createTrackbar("Mag gain FAR","TUNING", DEFAULTS["Mag_gain_far_x100"], 200, lambda x: None)
    cv2.createTrackbar("Mag gain NEAR","TUNING", DEFAULTS["Mag_gain_near_x100"], 200, lambda x: None)
    cv2.createTrackbar("Mag ignore s","TUNING", int(DEFAULTS["Mag_ignore_after_exit_s"]*10), 50, lambda x: None)

def read_tuner(P):
    g = lambda n: cv2.getTrackbarPos(n,"TUNING")
    P.update(dict(
        GainS_global=g("GainS x100"),
        GainV_global=g("GainV x100"),
        Gamma_x100=g("Gamma x100"),
        Kernel=max(1, g("Kernel")|1),
        PillarMinArea=g("PillarMinArea"),
        Kp_target_x100=g("Kp_tgt x100"),
        Kd_target_x100=g("Kd_tgt x100"),
        TargetMax_deg=g("TargetMax deg"),
        TurnBias_deg=g("TurnBias deg"),
        TurnCenter_area_T=g("TurnC area"),
        TurnCenter_hyst_x100=g("TurnC hyst x100"),
        Turn_lockout_s=g("Turn lock s")/10.0,
        Turn_max_time_s=g("Turn max s")/10.0,
        WALL_area_T=g("WALL area T"),
        WALL_corr_deg=g("WALL corr deg"),
        Run_Kp_x100=g("Run Kp x100"),
        Run_Kd_x100=g("Run Kd x100"),
        Run_Max_deg=g("Run Max deg"),
        Mag_avoid_y0_pct=g("Mag y0 %"),
        Mag_avoid_hmin_px=g("Mag hmin px"),
        Mag_avoid_area_min=g("Mag area min"),
        Mag_near_stable_fr=max(1,g("Mag near fr")),
        Mag_gain_far_x100=g("Mag gain FAR"),
        Mag_gain_near_x100=g("Mag gain NEAR"),
        Mag_gain_ramp=1,
        Mag_ignore_after_exit_s=g("Mag ignore s")/10.0,
    ))
    return P

# ===================== Procesado global imagen =====================
def apply_global_gains(bgr, gs_x100, gv_x100, gamma_x100):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    s = np.clip(s.astype(np.float32)*(gs_x100/100.0), 0, 255)
    v = np.clip(v.astype(np.float32)*(gv_x100/100.0), 0, 255)
    gamma = max(0.3, gamma_x100/100.0)
    v = (np.clip(v/255.0,0,1)**gamma)*255.0
    hsv_adj = cv2.merge([h, s.astype(np.uint8), v.astype(np.uint8)])
    return cv2.cvtColor(hsv_adj, cv2.COLOR_HSV2BGR)

# ===================== Color: HSV estricto (fallback) =====================
def mask_color(bgr, name):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if name == "magenta": return cv2.inRange(hsv, (135,120,120), (165,255,255))
    if name == "red":
        m1 = cv2.inRange(hsv, (0,140,140), (10,255,255))
        m2 = cv2.inRange(hsv, (170,140,140), (180,255,255))
        return cv2.bitwise_or(m1,m2)
    if name == "green":  return cv2.inRange(hsv, (40,80,80), (85,255,255))
    if name == "blue":   return cv2.inRange(hsv, (100,120,120), (130,255,255))
    if name == "orange": return cv2.inRange(hsv, (8,150,150), (22,255,255))
    if name == "white":
        s = hsv[:,:,1]; v=hsv[:,:,2]
        return cv2.inRange(s, 0, 40) & cv2.inRange(v, 200, 255)
    return np.zeros(bgr.shape[:2], np.uint8)

def mask_black(bgr, P):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L = lab[:,:,0]; A = lab[:,:,1]; B = lab[:,:,2]
    mL = cv2.inRange(L, P["L_min"], P["L_max"])
    mA = cv2.inRange(A, 128-P["A_tol"], 128+P["A_tol"])
    mB = cv2.inRange(B, P["B_tol"], 128+P["B_tol"])
    mLAB = cv2.bitwise_and(cv2.bitwise_and(mL,mA), mB)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mHSV = cv2.inRange(hsv[:,:,1], 0, P["Blk_Smax"]) & cv2.inRange(hsv[:,:,2], 0, P["Blk_Vmax"])
    m = cv2.bitwise_and(mLAB, mHSV)
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    m = cv2.GaussianBlur(m,(5,5),0); m = cv2.erode(m,k,1); m = cv2.dilate(m,k,1)
    return m

# ===================== ROIs =====================
WHITE_L_ROI = (0.00, 0.70, 0.30, 0.90)
WHITE_R_ROI = (0.70, 0.70, 1.00, 0.90)
ROIS = {
    "LINE_BOT":    (0.25, 0.87, 0.75, 0.95),
    "CENTER_BAND": (0.00, 0.42, 1.00, 1.00),
    "TURN_CENTER": (0.45, 0.48, 0.55, 0.85),
}
WALL_L_ROI  = (0.13, 0.85, 0.24, 1.00)
WALL_R_ROI  = (0.76, 0.85, 0.87, 1.00)

def frac_to_rect(img, f):
    h,w = img.shape[:2]
    x0,x1 = int(f[0]*w), int(f[2]*w)
    y0,y1 = int(f[1]*h), int(f[3]*h)
    x0,x1 = min(x0,x1), max(x0,x1)
    y0,y1 = min(y0,y1), max(y0,y1)
    if x1<=x0: x1 = min(w, x0+1)
    if y1<=y0: y1 = min(h, y0+1)
    return (x0,y0,x1,y1)

def draw_roi(frame, rect, label, color):
    x0,y0,x1,y1 = rect
    cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
    cv2.putText(frame,label,(x0+4,max(y0-6,18)),FONT,0.55,color,2,cv2.LINE_AA)

# ===================== Contornos helpers =====================
def grab_contours(res):
    if isinstance(res, tuple):
        return res[0] if len(res)==2 else (res[1] if len(res)==3 else [])
    return res

def largest_contour(mask):
    res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(res)
    if not cnts: return None, 0.0
    c = max(cnts, key=cv2.contourArea); return c, float(cv2.contourArea(c))

def top_contours(mask, k=2):
    res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(res)
    if not cnts: return []
    return sorted(cnts, key=cv2.contourArea, reverse=True)[:k]

# ===================== MAG priority & viewer =====================
def masks_magenta_red_with_priority(roi_bgr, kernel_size):
    mR = mask_color(roi_bgr, "red")
    mM = mask_color(roi_bgr, "magenta")
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    mR = cv2.morphologyEx(mR, cv2.MORPH_OPEN,  k, 1)
    mR = cv2.morphologyEx(mR, cv2.MORPH_CLOSE, k, 2)
    mM = cv2.morphologyEx(mM, cv2.MORPH_OPEN,  k, 1)
    mM = cv2.morphologyEx(mM, cv2.MORPH_CLOSE, k, 2)
    mR_clean = cv2.bitwise_and(mR, cv2.bitwise_not(mM))  # rojo limpio
    return mM, mR_clean

def detect_pillars_rgbm(proc_bgr, det_rect, kernel_size, min_area, show_view=False):
    """
    Detección de pilares RGB/M con prioridad MAG > RED.
    Devuelve para cada color: (cx_global, area_px).
    Además ahora DIBUJA el área en px al lado de cada bbox.  ### DEBUG AREA
    """
    x0,y0,x1,y1 = det_rect
    roi = proc_bgr[y0:y1, x0:x1].copy()

    mM, mR_clean = masks_magenta_red_with_priority(roi, kernel_size)
    mG = mask_color(roi, "green")
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    mG = cv2.morphologyEx(mG, cv2.MORPH_OPEN,  k, 1)
    mG = cv2.morphologyEx(mG, cv2.MORPH_CLOSE, k, 2)

    cGs = top_contours(mG, k=1)
    cRs = top_contours(mR_clean, k=1)
    cMs = top_contours(mM, k=2)

    out = {'R':(None,0.0), 'G':(None,0.0), 'M':(None,0.0)}

    # GREEN
    if cGs:
        cG = cGs[0]; aG = cv2.contourArea(cG)
        if aG >= min_area:
            x,y,w,h = cv2.boundingRect(cG); cx = x+w//2
            cv2.rectangle(roi,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.circle(roi,(cx,y+h//2),4,(255,255,255),-1)
            # --- texto con área ---
            cv2.putText(roi, f"G:{int(aG)}", (x, max(y-5, 12)),
                        FONT, 0.5, (0,255,0), 2, cv2.LINE_AA)   # ### DEBUG AREA
            out['G'] = (cx+x0, float(aG))

    # RED
    if cRs:
        cR = cRs[0]; aR = cv2.contourArea(cR)
        if aR >= min_area:
            x,y,w,h = cv2.boundingRect(cR); cx = x+w//2
            cv2.rectangle(roi,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.circle(roi,(cx,y+h//2),4,(255,255,255),-1)
            cv2.putText(roi, f"R:{int(aR)}", (x, max(y-5, 12)),
                        FONT, 0.5, (0,0,255), 2, cv2.LINE_AA)  # ### DEBUG AREA
            out['R'] = (cx+x0, float(aR))

    # MAGENTA (mejor de 2)
    best_M = (None, 0.0)
    best_box = None
    for cM in cMs:
        aM = cv2.contourArea(cM)
        if aM < min_area: continue
        x,y,w,h = cv2.boundingRect(cM); cx = x+w//2
        cv2.rectangle(roi,(x,y),(x+w,y+h),(255,0,255),2)
        cv2.circle(roi,(cx,y+h//2),4,(255,255,255),-1)
        if aM > best_M[1]:
            best_M = (cx+x0, float(aM))
            best_box = (x,y,w,h)
    if best_M[0] is not None:
        # área magenta dominante
        if best_box is not None:
            x,y,w,h = best_box
            cv2.putText(roi, f"M:{int(best_M[1])}", (x, max(y-5, 12)),
                        FONT, 0.5, (255,0,255), 2, cv2.LINE_AA)  # ### DEBUG AREA
        out['M'] = best_M

    proc_bgr[y0:y1, x0:x1] = roi
    cv2.putText(proc_bgr, f"M_cnt={len(cMs)}", (x0+6, y0+18), FONT, 0.55, (255,0,255), 2, cv2.LINE_AA)
    cv2.putText(proc_bgr, f"R_clean={'Y' if cRs else 'N'}", (x0+6, y0+38), FONT, 0.55, (0,0,255), 2, cv2.LINE_AA)

    if show_view:
        vis0 = roi
        vis1 = cv2.cvtColor(mM, cv2.COLOR_GRAY2BGR)
        vis2 = cv2.cvtColor(mR_clean, cv2.COLOR_GRAY2BGR)
        vis3 = cv2.bitwise_or(vis1, vis2)
        top = np.hstack([vis0, vis1]); bot = np.hstack([vis2, vis3])
        grid = np.vstack([top, bot])
        cv2.imshow("Viewer MAGENTA/RED", grid)

    return out

# ===================== Magenta GATING (proximidad) =====================
def magenta_nearness(roi_bgr, kernel, frame_h, y0_pct=72, hmin_px=42, area_min=1200):
    mM = mask_color(roi_bgr, "magenta")
    k  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel,kernel))
    mM = cv2.morphologyEx(mM, cv2.MORPH_OPEN,  k, 1)
    mM = cv2.morphologyEx(mM, cv2.MORPH_CLOSE, k, 2)

    res  = cv2.findContours(mM, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = res[0] if len(res)==2 else (res[1] if len(res)==3 else [])
    if not cnts: return (False, 0.0, None)

    c_best = max(cnts, key=cv2.contourArea)
    area   = cv2.contourArea(c_best)
    if area < area_min: return (False, 0.0, None)

    x,y,w,h = cv2.boundingRect(c_best)
    y_bottom = y + h

    near_h   = (h >= hmin_px)
    near_y   = (100.0 * y_bottom / float(frame_h)) >= float(y0_pct)
    is_near  = (near_h and near_y)

    y_lo = max(0.0, y0_pct-10.0) / 100.0 * frame_h
    y_hi = 1.00 * frame_h
    alpha = 0.0 if y_bottom <= y_lo else (1.0 if y_bottom >= y_hi else (y_bottom - y_lo) / (y_hi - y_lo))
    return (is_near, float(np.clip(alpha,0.0,1.0)), c_best)

# ===================== Main =====================
_stop=False
VIEW_RM=False
def _sig(_s,_f): 
    global _stop; _stop=True

def main():
    global VIEW_RM
    signal.signal(signal.SIGINT, _sig)
    ser = open_serial()

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened(): raise SystemExit("No camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_H)
    cap.set(cv2.CAP_PROP_FPS,30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    P = load_cfg()
    build_tuner()

    # ======== FASE 1: Decide EXT con BLANCO L/R y espera fin de salida ========
    win = "RuloBot — Exit by WHITE (L/R)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 960, 720)

    rx=b""; exit_done=False
    last_ext_dir='L'   # 'L' o 'R'
    last_sent=0.0
    locked_turn = None   # se fija cuando llega GO / skip / [SEQ] END

    print("[INFO] WHITE L/R activos. Pulsa botón en el Nano o presiona 'X' para saltar.")
    while (not exit_done) and (not _stop):
        ok, f = cap.read()
        if not ok: continue
        if MIRROR_X: f = cv2.flip(f, 1)

        P = read_tuner(P)

        fp = apply_global_gains(f, P["GainS_global"], P["GainV_global"], P["Gamma_x100"])
        Lr = frac_to_rect(fp, WHITE_L_ROI)
        Rr = frac_to_rect(fp, WHITE_R_ROI)

        wL = cv2.countNonZero(mask_color(fp[Lr[1]:Lr[3], Lr[0]:Lr[2]], "white"))
        wR = cv2.countNonZero(mask_color(fp[Rr[1]:Rr[3], Rr[0]:Rr[2]], "white"))
        last_ext_dir = 'R' if (wR>wL) else 'L'
        current_sense = 'RIGHT' if last_ext_dir=='R' else 'LEFT'

        now = time.time()
        if ser and (now-last_sent)>=0.25:
            send(ser, f"EXT:{current_sense}")
            last_sent = now

        draw_roi(fp, Lr, f"WHITE_L:{wL}", (0,255,255))
        draw_roi(fp, Rr, f"WHITE_R:{wR}", (0,255,255))

        lock_txt = "LOCK:? "
        if locked_turn == 'L': lock_txt = "LOCK:LEFT "
        elif locked_turn == 'R': lock_txt = "LOCK:RIGHT"
        cv2.putText(fp, f"EXT -> {current_sense}   (X = skip)   {lock_txt}",
                    (10,30), FONT, 0.7, (255,255,255), 2)
        cv2.imshow(win, fp)

        key = cv2.waitKey(1)&0xFF
        if key==ord('q'):
            save_cfg(P); return
        elif key==ord('s'):
            save_cfg(P)
        elif key==ord('r'):
            P = load_cfg()
        elif key==ord('x'):
            if locked_turn is None:
                locked_turn = last_ext_dir
                print(f"[DIR] LOCKED by skip (X): {locked_turn}")
            print("[INFO] Salto manual de salida → Obstacle.")
            exit_done = True
            break
        elif key==ord('m'):
            VIEW_RM = not VIEW_RM
            if not VIEW_RM:
                try: cv2.destroyWindow("Viewer MAGENTA/RED")
                except: pass

        if ser and ser.in_waiting:
            rx += ser.read(ser.in_waiting)
            while b"\n" in rx:
                line, rx = rx.split(b"\n",1)
                s=line.decode(errors='ignore').strip()
                sup = s.upper()
                if sup=="GO":
                    if locked_turn is None:
                        locked_turn = last_ext_dir
                        print(f"[DIR] LOCKED by GO: {locked_turn}")
                if s.startswith("[SEQ] END"):
                    print("[INFO] Exit por encoder finalizado.")
                    exit_done=True
                    break

    if not exit_done and not _stop:
        cap.release(); cv2.destroyAllWindows()
        if ser: 
            try: ser.close()
            except: pass
        return

    if locked_turn is None:
        locked_turn = last_ext_dir
        print(f"[DIR] LOCKED fallback at END: {locked_turn}")

    if ser:
        send(ser,"TURN_OFF")
        send(ser,"STEER:84")
        time.sleep(0.05)
        send(ser,"STEER:84")

    exit_ts = time.time()
    time.sleep(0.2)

    # ======== FASE 2: OBSTACLE COMPLETO ========
    win2 = "RuloBot — Obstacle FULL (RGB/M + TURN + RUN)"
    cv2.namedWindow(win2, cv2.WINDOW_NORMAL); cv2.resizeWindow(win2, 960, 720)

    SERVO_CENTER_CMD=84
    selected_turn = 'R' if locked_turn=='R' else 'L'
    print(f"[OBST] Direccion de bias TURN fija (locked): {selected_turn}")

    O_MIN_FRAC = 0.015; O_MIN_H = 7
    B_MIN_FRAC = 0.015; B_MIN_H = 7
    LINE_COOLDOWN_S = 3.0; NUM_LINES_TO_STOP = 13
    last_line_ts=0.0; line_count=0

    TX_PERIOD=0.015; last_tx=0.0
    prev_err_target=0.0; prev_err_run=0.0
    turning=False
    turn_lock_until = exit_ts + float(P["Turn_lockout_s"])
    turn_deadline = None

    mag_nearL_fr = 0
    mag_nearR_fr = 0

    t0=time.time(); n=0; fps=0.0

    while (not _stop):
        ok, f = cap.read()
        if not ok: continue
        if MIRROR_X: f = cv2.flip(f, 1)
        n+=1; t1=time.time()
        if (t1-t0)>=1.0: fps=n/(t1-t0); n=0; t0=t1

        P = read_tuner(P)
        frame = apply_global_gains(f, P["GainS_global"], P["GainV_global"], P["Gamma_x100"])

        lineR = frac_to_rect(frame, ROIS["LINE_BOT"])
        detR  = frac_to_rect(frame, ROIS["CENTER_BAND"])
        turnR = frac_to_rect(frame, ROIS["TURN_CENTER"])
        wallL = frac_to_rect(frame, WALL_L_ROI)
        wallR = frac_to_rect(frame, WALL_R_ROI)

        draw_roi(frame, lineR, "LINE_BOT", (200,200,0))
        draw_roi(frame, detR,  "CENTER_BAND (R/G/M)", (0,255,255))
        draw_roi(frame, turnR, "TURN_CENTER", (0,200,255))
        draw_roi(frame, wallL, "WALL_L", (0,255,0))
        draw_roi(frame, wallR, "WALL_R", (0,255,0))

        def detect_area(rect, color_name):
            x0,y0,x1,y1 = rect
            roi = frame[y0:y1, x0:x1].copy()
            mask = mask_color(roi, color_name)
            k = cv2.getStructuringElement(cv2.MORPH_RECT,(P["Kernel"],P["Kernel"]))
            mask = cv2.GaussianBlur(mask,(5,5),0); mask = cv2.erode(mask,k,1); mask = cv2.dilate(mask,k,1)
            c,_ = largest_contour(mask)
            if c is not None:
                cv2.drawContours(roi,[c],-1,(0,165,255) if color_name=="orange" else (255,120,0),2)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(roi,(x,y),(x+w,y+h),(0,140,255) if color_name=="orange" else (255,120,0),2)
            frame[y0:y1, x0:x1] = roi
            area = float(cv2.contourArea(c)) if c is not None else 0.0
            hbox = cv2.boundingRect(c)[3] if c is not None else 0
            return area, hbox

        aO,hO = detect_area(lineR, "orange")
        aB,hB = detect_area(lineR, "blue")
        roi_area = max(1,(lineR[2]-lineR[0])*(lineR[3]-lineR[1]))
        saw_orange = (aO/roi_area >= O_MIN_FRAC and hO>=O_MIN_H)
        saw_blue   = (aB/roi_area >= B_MIN_FRAC and hB>=B_MIN_H)

        now=time.time()
        if (saw_orange or saw_blue) and ((now-last_line_ts)>=LINE_COOLDOWN_S):
            line_count += 1; last_line_ts = now
            if line_count >= NUM_LINES_TO_STOP and ser:
                send(ser,"STOP")

        # ---- Pilares (RGB/M) ----
        pillars = detect_pillars_rgbm(frame, detR, P["Kernel"], P["PillarMinArea"], show_view=VIEW_RM)
        R_cx,R_area = pillars['R']
        G_cx,G_area = pillars['G']
        M_cx,M_area = pillars['M']

        # HUD extra con áreas de pilares  ### DEBUG AREA
        cv2.putText(frame,
                    f"Areas px  R={int(R_area)}  G={int(G_area)}  M={int(M_area)}",
                    (10, 64), FONT, 0.55, (0,255,255), 2, cv2.LINE_AA)

        tx0,ty0,tx1,ty1 = turnR
        roi_turn = frame[ty0:ty1, tx0:tx1].copy()
        m_turn = mask_black(roi_turn, P)
        cntT, area_turn = largest_contour(m_turn)
        if cntT is not None: cv2.drawContours(roi_turn, [cntT], -1, (0,200,255), 2)
        cv2.addWeighted(roi_turn,0.65,cv2.cvtColor(m_turn,cv2.COLOR_GRAY2BGR),0.35,0,roi_turn)
        frame[ty0:ty1, tx0:tx1] = roi_turn

        enter = (area_turn >= float(P["TurnCenter_area_T"]))
        exit_ = (area_turn <= float(P["TurnCenter_area_T"])*(max(10,P["TurnCenter_hyst_x100"])/100.0))

        if (not turning):
            if (now > turn_lock_until) and enter:
                turning=True; turn_deadline = now + float(P["Turn_max_time_s"])
                if ser: send(ser,"TURN_ON")
        else:
            if exit_ or (turn_deadline and now>turn_deadline):
                turning=False; turn_deadline=None
                if ser: send(ser,"TURN_OFF")

        pillar_active = ((R_area>=P["PillarMinArea"]) or (G_area>=P["PillarMinArea"]) or (M_area>=P["PillarMinArea"]))
        if pillar_active:
            mode = "AVOID"
        elif turning:
            mode = "TURN"
        else:
            mode = "RUN"

        steer_cmd = SERVO_CENTER_CMD

        def pct_rect_to_abs(px0,py0,px1,py1):
            x0 = int(np.clip(px0,0,100)/100.0 * FRAME_W)
            y0 = int(np.clip(py0,0,100)/100.0 * FRAME_H)
            x1 = int(np.clip(px1,0,100)/100.0 * FRAME_W)
            y1 = int(np.clip(py1,0,100)/100.0 * FRAME_H)
            return (min(x0,x1),min(y0,y1),max(x0,x1),max(y0,y1))

        if mode=="AVOID":
            ctrl = max([('R',R_area,R_cx), ('G',G_area,G_cx), ('M',M_area,M_cx)], key=lambda t:t[1])
            ctrl_color, ctrl_area, ctrl_cx = ctrl
            err = 0.0
            if ctrl_area >= P["PillarMinArea"] and ctrl_cx is not None:
                if ctrl_color=='R':
                    T = pct_rect_to_abs(P["TRED_x0"],P["TRED_y0"],P["TRED_x1"],P["TRED_y1"])
                elif ctrl_color=='G':
                    T = pct_rect_to_abs(P["TGRN_x0"],P["TGRN_y0"],P["TGRN_x1"],P["TGRN_y1"])
                else:
                    T = pct_rect_to_abs(P["TMAG_x0"],P["TMAG_y0"],P["TMAG_x1"],P["TMAG_y1"])
                tgt_cx = (T[0]+T[2])//2
                err = float(ctrl_cx - tgt_cx)

            d_err = err - prev_err_target; prev_err_target = err
            Kp = P["Kp_target_x100"]/100.0
            Kd = P["Kd_target_x100"]/100.0
            u = float(np.clip(Kp*err + Kd*d_err, -P["TargetMax_deg"], P["TargetMax_deg"]))

            def wall_area(rect):
                x0,y0,x1,y1 = rect
                roi = frame[y0:y1, x0:x1]
                m = mask_black(roi, P)
                return float(cv2.countNonZero(m))
            area_wall_l = wall_area(wallL)
            area_wall_r = wall_area(wallR)
            wallL_hit = (area_wall_l >= float(P["WALL_area_T"]))
            wallR_hit = (area_wall_r >= float(P["WALL_area_T"]))
            wall_corr = float(P["WALL_corr_deg"])
            if wallL_hit:
                if u < 0: u = 0.0
                u += wall_corr
                cv2.putText(frame, "WALL_L BLOCK → RIGHT", (10, 90), FONT, 0.55, (0,255,0),2)
            if wallR_hit:
                if u > 0: u = 0.0
                u -= wall_corr
                cv2.putText(frame, "WALL_R BLOCK → LEFT",  (10,110), FONT, 0.55, (0,255,0),2)

            steer_cmd = SERVO_CENTER_CMD + int(round(u))
            cv2.putText(frame, f"MODE:AVOID {ctrl_color} u={u:.1f}", (10, 44), FONT, 0.55, (0,255,255),2)

        elif mode=="TURN":
            bias = TURN_DIR_SIGN * float(P["TurnBias_deg"]) * (+1 if selected_turn=='R' else -1)
            steer_cmd = SERVO_CENTER_CMD + int(round(bias))
            cv2.putText(frame, f"MODE:TURN dir={selected_turn} area={int(area_turn)} bias={bias:.1f}",
                        (10,44), FONT, 0.55, (0,200,255),2)

        else:
            wl_x0,wl_y0,wl_x1,wl_y1 = wallL
            wr_x0,wr_y0,wr_x1,wr_y1 = wallR
            roiL = frame[wl_y0:wl_y1, wl_x0:wl_x1].copy()
            roiR = frame[wr_y0:wr_y1, wr_x0:wr_x1].copy()

            mBL = mask_black(roiL, P); areaBL = float(cv2.countNonZero(mBL))
            mBR = mask_black(roiR, P); areaBR = float(cv2.countNonZero(mBR))

            nearL, alphaL, cML = magenta_nearness(roiL, P["Kernel"], FRAME_H,
                                                  y0_pct=P["Mag_avoid_y0_pct"],
                                                  hmin_px=P["Mag_avoid_hmin_px"],
                                                  area_min=P["Mag_avoid_area_min"])
            nearR, alphaR, cMR = magenta_nearness(roiR, P["Kernel"], FRAME_H,
                                                  y0_pct=P["Mag_avoid_y0_pct"],
                                                  hmin_px=P["Mag_avoid_hmin_px"],
                                                  area_min=P["Mag_avoid_area_min"])

            if nearL: mag_nearL_fr += 1
            else:     mag_nearL_fr = 0
            if nearR: mag_nearR_fr += 1
            else:     mag_nearR_fr = 0

            if P.get("Mag_gain_ramp",1):
                gL = (P["Mag_gain_far_x100"]*(1.0-alphaL) + P["Mag_gain_near_x100"]*alphaL) / 100.0
                gR = (P["Mag_gain_far_x100"]*(1.0-alphaR) + P["Mag_gain_near_x100"]*alphaR) / 100.0
            else:
                gL = (P["Mag_gain_near_x100"]/100.0) if nearL else (P["Mag_gain_far_x100"]/100.0)
                gR = (P["Mag_gain_near_x100"]/100.0) if nearR else (P["Mag_gain_far_x100"]/100.0)

            if mag_nearL_fr < int(P["Mag_near_stable_fr"]): gL = min(gL, P["Mag_gain_far_x100"]/100.0)
            if mag_nearR_fr < int(P["Mag_near_stable_fr"]): gR = min(gR, P["Mag_gain_far_x100"]/100.0)

            if (time.time() - exit_ts) <= float(P["Mag_ignore_after_exit_s"]):
                gL = 0.0; gR = 0.0

            mML = mask_color(roiL, "magenta"); areaML = float(cv2.countNonZero(mML))
            mMR = mask_color(roiR, "magenta"); areaMR = float(cv2.countNonZero(mMR))

            areaL = areaBL + gL * areaML
            areaR = areaBR + gR * areaMR

            cv2.addWeighted(roiL,0.6, cv2.cvtColor(mBL,cv2.COLOR_GRAY2BGR),0.25, 0, roiL)
            cv2.addWeighted(roiR,0.6, cv2.cvtColor(mBR,cv2.COLOR_GRAY2BGR),0.25, 0, roiR)
            if cML is not None:
                x,y,w,h = cv2.boundingRect(cML); cv2.rectangle(roiL,(x,y),(x+w,y+h),(255,0,255),2)
            if cMR is not None:
                x,y,w,h = cv2.boundingRect(cMR); cv2.rectangle(roiR,(x,y),(x+w,y+h),(255,0,255),2)
            frame[wl_y0:wl_y1, wl_x0:wl_x1] = roiL
            frame[wr_y0:wr_y1, wr_x0:wr_x1] = roiR

            err = areaL - areaR
            d_err = err - prev_err_run; prev_err_run = err
            Kpr = P["Run_Kp_x100"]/100.0
            Kdr = P["Run_Kd_x100"]/100.0
            u = float(np.clip(Kpr*err + Kdr*d_err, -P["Run_Max_deg"], P["Run_Max_deg"]))
            steer_cmd = SERVO_CENTER_CMD + int(round(u))
            cv2.putText(frame, f"MODE:RUN u={u:.1f}", (10, 44), FONT, 0.55, (0,255,255),2)

        steer_cmd = int(np.clip(steer_cmd, 24, 144))
        if ser and (time.time()-last_tx)>=TX_PERIOD:
            send(ser, f"STEER:{steer_cmd}")
            last_tx = time.time()

        hud = f"{FRAME_W}x{FRAME_H} ~{fps:.1f}fps  Lines:{line_count}/{NUM_LINES_TO_STOP}  Bias:{selected_turn}"
        cv2.putText(frame, hud, (10,20), FONT, 0.58, (255,255,255),2)
        cv2.imshow(win2, frame)

        key = cv2.waitKey(1)&0xFF
        if key==ord('q'):
            save_cfg(P); break
        elif key==ord('s'):
            save_cfg(P)
        elif key==ord('r'):
            P = load_cfg()
        elif key==ord('m'):
            VIEW_RM = not VIEW_RM
            if not VIEW_RM:
                try: cv2.destroyWindow("Viewer MAGENTA/RED")
                except: pass

    cap.release(); cv2.destroyAllWindows()
    if ser:
        try: ser.close()
        except: pass

if __name__=="__main__":
    main()
