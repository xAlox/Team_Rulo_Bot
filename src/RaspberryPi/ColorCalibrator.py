#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Team Rulo Bot — LAB Calibrator + Viewer (Mahalanobis) — Same Preprocessing as Runtime
ES: Calibra clases LAB y muestra Viewer con los mismos GainS/GainV/Gamma/Kernel que usa Obstacle.
    Guarda: ~/rulobot_lab_color_model.json y ~/.rulobot_vision_tuning.json
EN: Calibrate LAB classes and preview masks using the same preprocessing as Obstacle runtime.
Keys:
  [1..6] select class (1=ORANGE,2=BLUE,3=RED,4=GREEN,5=BLACK,6=MAGENTA)
  [c] capture ROI samples   [p] process models   [v] toggle viewer   [s] save   [q] quit
"""

import cv2, numpy as np, json, os, time

# ============== Camera ==============
CAM_INDEX = 0; W,H,FPS = 640,480,30
FOURCC = 'MJPG'
FONT   = cv2.FONT_HERSHEY_SIMPLEX

# ============== Paths ==============
MODEL_PATH = os.path.expanduser("~/rulobot_lab_color_model.json")
CFG_PATH   = os.path.expanduser("~/.rulobot_vision_tuning.json")

# ============== Classes ==============
CLASSES = ["ORANGE","BLUE","RED","GREEN","BLACK","MAGENTA"]
name_map_save = {"ORANGE":"orange","BLUE":"blue","RED":"red","GREEN":"green","BLACK":"black","MAGENTA":"magenta"}

# ============== ROIs (viewer & capture) ==============
ROIS = {
    "LINE_BOT":    (0.25, 0.87, 0.75, 0.95),
    "CENTER_BAND": (0.00, 0.42, 1.00, 0.95),
    "TURN_CENTER": (0.45, 0.47, 0.55, 0.85),
    "WALL_L":      (0.13, 0.85, 0.24, 1.00),
    "WALL_R":      (0.76, 0.85, 0.87, 1.00),
}
CAPTURE_RECT_FRAC = (0.30, 0.35, 0.70, 0.70)
CAPTURE_KEEP_FRAC = 0.20

# ============== Tuning defaults (match Obstacle) ==============
DEFAULTS = dict(
    GainS_global=120, GainV_global=90, Gamma_x100=100,
    Kernel=5
)

# ============== State ==============
state = {c: {"a":[], "b":[]} for c in CLASSES}
TAU2  = 9.21       # chi^2, df=2 (~99%)
MORPH = 5

# ============== Helpers ==============
def frac_to_rect(img, f):
    h,w = img.shape[:2]
    return (int(f[0]*w), int(f[1]*h), int(f[2]*w), int(f[3]*h))
def shrink_rect_centered(rect, keep_frac):
    x0,y0,x1,y1=rect; cx=(x0+x1)/2; cy=(y0+y1)/2
    nw=(x1-x0)*keep_frac; nh=(y1-y0)*keep_frac
    nx0=int(round(cx-nw/2)); nx1=int(round(cx+nw/2))
    ny0=int(round(cy-nh/2)); ny1=int(round(cy+nh/2))
    return (nx0,ny0,nx1,ny1)
def draw_roi(frame, rect, label, color=(0,255,255)):
    x0,y0,x1,y1 = rect
    cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
    cv2.putText(frame,label,(x0+4,max(y0-6,18)),FONT,0.55,color,2,cv2.LINE_AA)
def largest_contour(mask):
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not cnts: return None, 0.0
    c = max(cnts, key=cv2.contourArea)
    return c, float(cv2.contourArea(c))
def apply_global_gains(bgr, gs_x100, gv_x100, gamma_x100):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    s = np.clip(s.astype(np.float32)*(gs_x100/100.0),0,255)
    v = np.clip(v.astype(np.float32)*(gv_x100/100.0),0,255)
    gamma = max(0.3, gamma_x100/100.0)
    v = (np.clip(v/255.0,0,1)**gamma)*255.0
    hsv_adj = cv2.merge([h, s.astype(np.uint8), v.astype(np.uint8)])
    return cv2.cvtColor(hsv_adj, cv2.COLOR_HSV2BGR)

def compute_maha_model(a_list, b_list, tau2=TAU2):
    A = np.asarray(a_list, dtype=np.float32).reshape(-1,1)
    B = np.asarray(b_list, dtype=np.float32).reshape(-1,1)
    X = np.hstack([A,B])
    if X.shape[0] < 5: return None
    mean = X.mean(axis=0)
    C = np.cov(X, rowvar=False)
    C += np.eye(2)*1e-3
    cov_inv = np.linalg.inv(C)
    return {"mean":[float(mean[0]), float(mean[1])],
            "cov_inv":[[float(cov_inv[0,0]), float(cov_inv[0,1])],
                       [float(cov_inv[1,0]), float(cov_inv[1,1])]],
            "tau2":float(tau2)}

def mahalanobis_mask_from_model(bgr_roi, model):
    lab = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2LAB)
    A = lab[:,:,1].astype(np.float32); B = lab[:,:,2].astype(np.float32)
    mean   = np.array(model["mean"], dtype=np.float32)
    covinv = np.array(model["cov_inv"], dtype=np.float32)
    tau2   = float(model["tau2"])
    X = np.stack([A-mean[0], B-mean[1]], axis=-1)
    dist2 = np.einsum('...i,ij,...j->...', X, covinv, X)
    m = (dist2 <= tau2).astype(np.uint8)*255
    k = cv2.getStructuringElement(cv2.MORPH_RECT,(MORPH,MORPH))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,k, iterations=1)
    return m

def load_cfg(path=CFG_PATH):
    try:
        with open(path,"r") as f: return json.load(f)
    except: return None
def save_cfg(vals, path=CFG_PATH):
    base = load_cfg() or {}
    base.update(vals)
    with open(path,"w") as f:
        json.dump(base, f, indent=2)
    print("[CFG] Guardado:", path)

# ============== Fallback HSV (solo para viewer auxiliar) ==============
FALLBACK = {
    "orange": (5,35,40,50),
    "blue":   (100,130,110,40),
    "green":  (40,85,70,70),
    "red1":   (0,10,120,70),
    "red2":   (170,180,120,70),
    "magenta":(135,165,110,70),
}
def mask_fallback(bgr, name):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if name=="orange":
        h1,h2,s,v = FALLBACK["orange"]; return cv2.inRange(hsv,(h1,s,v),(h2,255,255))
    if name=="blue":
        h1,h2,s,v = FALLBACK["blue"];   return cv2.inRange(hsv,(h1,s,v),(h2,255,255))
    if name=="green":
        h1,h2,s,v = FALLBACK["green"];  return cv2.inRange(hsv,(h1,s,v),(h2,255,255))
    if name=="magenta":
        h1,h2,s,v = FALLBACK["magenta"];return cv2.inRange(hsv,(h1,s,v),(h2,255,255))
    if name=="red":
        h1,h2,s,v = FALLBACK["red1"]; m1=cv2.inRange(hsv,(h1,s,v),(h2,255,255))
        h1,h2,s,v = FALLBACK["red2"]; m2=cv2.inRange(hsv,(h1,s,v),(h2,255,255))
        return cv2.bitwise_or(m1,m2)
    if name=="black":
        lab=cv2.cvtColor(bgr,cv2.COLOR_BGR2LAB); L=lab[:,:,0]
        mL=cv2.inRange(L,0,95); S=hsv[:,:,1]; V=hsv[:,:,2]
        return cv2.bitwise_and(mL, cv2.inRange(S,0,120) & cv2.inRange(V,0,120))
    return np.zeros(bgr.shape[:2], np.uint8)

# ============== Tuner UI (misma que Obstacle) ==============
def build_tuner(cfg):
    cv2.namedWindow("TUNING", cv2.WINDOW_NORMAL); cv2.resizeWindow("TUNING", 480, 320)
    cv2.createTrackbar("GainS_gl x100", "TUNING", cfg["GainS_global"], 300, lambda x: None)
    cv2.createTrackbar("GainV_gl x100", "TUNING", cfg["GainV_global"], 300, lambda x: None)
    cv2.createTrackbar("Gamma x100",    "TUNING", cfg["Gamma_x100"],  300, lambda x: None)
    cv2.createTrackbar("Kernel",         "TUNING", cfg["Kernel"],       15, lambda x: None)
def read_tuner():
    g=lambda n: cv2.getTrackbarPos(n,"TUNING")
    return dict(
        GainS_global=g("GainS_gl x100"),
        GainV_global=g("GainV_gl x100"),
        Gamma_x100=g("Gamma x100"),
        Kernel=max(1, g("Kernel")|1)
    )

# ============== Viewer overlay (usa LAB si hay modelo) ==============
def overlay_verification(frame_bgr, models, ker):
    def draw_mask(name, rect, keys, color, min_area=0):
        x0,y0,x1,y1 = rect; roi = frame_bgr[y0:y1,x0:x1]
        acc=None
        for k in keys:
            mdl = models.get(k)
            if mdl is not None:
                m = mahalanobis_mask_from_model(roi, mdl)
            else:
                # fallback HSV solo para visual
                fallback_name = {"ORANGE":"orange","BLUE":"blue","RED":"red","GREEN":"green",
                                 "BLACK":"black","MAGENTA":"magenta"}[k]
                m = mask_fallback(roi, fallback_name)
            kx = cv2.getStructuringElement(cv2.MORPH_RECT,(ker,ker))
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kx, iterations=1)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,kx, iterations=1)
            acc = m if acc is None else cv2.bitwise_or(acc,m)
        draw_roi(frame_bgr, (x0,y0,x1,y1), name, color)
        if acc is None: return
        cnt, area = largest_contour(acc)
        if cnt is not None and area>=min_area:
            cv2.drawContours(roi,[cnt],-1,color,2)
        cv2.addWeighted(roi,0.65,cv2.cvtColor(acc,cv2.COLOR_GRAY2BGR),0.35,0,roi)

    lineR  = frac_to_rect(frame_bgr, ROIS["LINE_BOT"])
    ctrR   = frac_to_rect(frame_bgr, ROIS["CENTER_BAND"])
    turnR  = frac_to_rect(frame_bgr, ROIS["TURN_CENTER"])
    wallL  = frac_to_rect(frame_bgr, ROIS["WALL_L"])
    wallR  = frac_to_rect(frame_bgr, ROIS["WALL_R"])

    draw_mask("LINE_BOT",    lineR, ["ORANGE","BLUE"], (0,200,255), 250)
    draw_mask("CENTER_BAND", ctrR,  ["RED","GREEN","MAGENTA"], (0,255,255), 400)
    draw_mask("TURN_CENTER", turnR, ["BLACK"], (0,255,200), 1000)
    draw_mask("WALL_L",      wallL, ["BLACK","MAGENTA"], (255,0,255), 800)
    draw_mask("WALL_R",      wallR, ["BLACK","MAGENTA"], (255,0,255), 800)

# ============== Main ==============
def main():
    # camera
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened(): raise SystemExit("[ERROR] No camera.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
    cap.set(cv2.CAP_PROP_FPS,FPS)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))

    # capture ROI
    dummy = np.zeros((H,W,3), np.uint8)
    base_rect = frac_to_rect(dummy, CAPTURE_RECT_FRAC)
    cap_rect  = shrink_rect_centered(base_rect, CAPTURE_KEEP_FRAC)

    # tuner
    cfg = load_cfg() or DEFAULTS.copy()
    cfg = {**DEFAULTS, **{k:cfg.get(k,DEFAULTS[k]) for k in DEFAULTS}}
    build_tuner(cfg)

    selected = CLASSES[0]
    models_now = {}  # dict NOMBRE->modelo
    view_masks = True

    help_lines = [
        "LAB Calibrator+Viewer — same preprocessing as runtime",
        "[1..6] class  (1=ORANGE,2=BLUE,3=RED,4=GREEN,5=BLACK,6=MAGENTA)",
        "[c] capture   [p] process   [v] view masks   [s] save JSON+CFG   [q] quit",
        f"MODEL: {MODEL_PATH}",
        f"CFG:   {CFG_PATH}",
    ]

    while True:
        ok, frame = cap.read()
        if not ok: continue

        # read tuner and apply same preprocessing used in runtime
        P = read_tuner()
        frame_proc = apply_global_gains(frame, P["GainS_global"], P["GainV_global"], P["Gamma_x100"])

        # HUD
        y = 22
        for t in help_lines: cv2.putText(frame_proc, t, (10,y), FONT, 0.55, (255,255,255),2,cv2.LINE_AA); y+=22
        cv2.putText(frame_proc, f"[SEL] {selected}", (10,y+5), FONT, 0.65, (0,255,255),2,cv2.LINE_AA)
        draw_roi(frame_proc, cap_rect, "CAPTURE ROI (80% smaller)", (0,255,255))

        # Viewer overlay (con la MISMA imagen preprocesada)
        if view_masks and len(models_now)>0:
            overlay_verification(frame_proc, models_now, P["Kernel"])

        cv2.imshow("LAB-Cal+Viewer (Mahalanobis)", frame_proc)
        k = cv2.waitKey(1) & 0xFF

        if k in [ord('1'),ord('2'),ord('3'),ord('4'),ord('5'),ord('6')]:
            selected = CLASSES[int(chr(k))-1]

        elif k == ord('c'):
            # toma muestras del frame ya preprocesado
            x0,y0,x1,y1 = cap_rect
            roi = frame_proc[y0:y1, x0:x1]
            lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            A = lab[:,:,1].ravel().astype(np.float32)
            B = lab[:,:,2].ravel().astype(np.float32)
            if A.size > 6000:
                idx = np.random.choice(A.size, 6000, replace=False)
                A = A[idx]; B = B[idx]
            state[selected]["a"].extend(A.tolist())
            state[selected]["b"].extend(B.tolist())
            print(f"[{selected}] muestras: {len(state[selected]['a'])}")

        elif k == ord('p'):
            models_now = {}
            for cls in CLASSES:
                m = compute_maha_model(state[cls]["a"], state[cls]["b"], TAU2)
                if m: models_now[cls] = m
            print("[INFO] Clases calibradas:", list(models_now.keys()))

        elif k == ord('v'):
            view_masks = not view_masks

        elif k == ord('s'):
            # Guardar modelos con nombres cortos + metadatos de preprocesado
            save_obj = dict(
                meta=dict(space="LAB_AB_mahalanobis",
                          tau2=float(TAU2),
                          created=time.strftime("%Y-%m-%d %H:%M:%S"),
                          preprocessing=dict(
                              GainS_global=P["GainS_global"],
                              GainV_global=P["GainV_global"],
                              Gamma_x100=P["Gamma_x100"],
                              Kernel=P["Kernel"]
                          )),
                classes={}
            )
            for k0, model in models_now.items():
                save_obj["classes"][name_map_save[k0]] = model
            try:
                with open(MODEL_PATH,"w") as f: json.dump(save_obj,f,indent=2)
                print(f"[OK] Guardado modelo: {MODEL_PATH}")
            except Exception as e:
                print(f"[ERR] Modelo: {e}")
            # Guardar CFG para que Obstacle vea igual
            save_cfg(dict(GainS_global=P["GainS_global"],
                          GainV_global=P["GainV_global"],
                          Gamma_x100=P["Gamma_x100"],
                          Kernel=P["Kernel"]))

        elif k == ord('q'):
            break

    cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
