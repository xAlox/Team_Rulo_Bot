#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ColorCalibrator — RuloBot

HSV + LAB auto-calibration script with forced saturation boost (S = 300%)
and robust margins. This program collects multiple samples of each color
under different lighting conditions to build a tuning configuration automatically.

Colors calibrated:
 - GREEN block (obstacle)
 - RED block (obstacle)
 - BLUE line
 - ORANGE line
 - BLACK wall

Each color is captured 5 times. Press 'c' to capture a sample with the object
inside the ROI, or 'q' to cancel at any point.

The resulting tuning values are saved to:
    ~/.rulobot_vision_tuning.json
and match the keys used by the main runtime.
"""

import cv2, numpy as np, json, os, time

# ===================== CAMERA SETTINGS =====================
CAM_INDEX = 0
W, H = 640, 480
FONT = cv2.FONT_HERSHEY_SIMPLEX

# ===================== FILE PATH =====================
CONFIG_PATH = os.path.expanduser("~/.rulobot_vision_tuning.json")

# ===================== FORCED IMAGE GAINS =====================
# Force saturation to make colors more vivid and consistent
GAIN_S = 300
GAIN_V = 90
GAMMA  = 1.00

# ===================== CALIBRATION ROIs =====================
# These are the ROIs used during calibration.
# You can edit these fractions (x0, y0, x1, y1) to fit your setup.
ROI_BLOCK = (0.30, 0.35, 0.70, 0.70)   # For color block calibration
ROI_LINE  = (0.25, 0.87, 0.75, 0.95)   # For line calibration
ROI_BLACK = (0.30, 0.50, 0.70, 0.70)   # For black wall/floor calibration

# ===================== HELPER FUNCTIONS =====================
def frac_to_rect(img, f):
    """Convert fractional ROI (0–1) to absolute pixel coordinates."""
    h,w = img.shape[:2]
    return (int(f[0]*w), int(f[1]*h), int(f[2]*w), int(f[3]*h))

def draw_roi(frame, rect, label, color=(0,255,255)):
    """Draw a labeled rectangle on the frame for calibration guidance."""
    x0,y0,x1,y1 = rect
    cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
    cv2.putText(frame,label,(x0+4,y0+18),FONT,0.6,color,2,cv2.LINE_AA)

def apply_gains(bgr, s_gain=GAIN_S, v_gain=GAIN_V, gamma=GAMMA):
    """Apply forced saturation and brightness to make colors stand out."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H,S,V = cv2.split(hsv)
    S = np.clip(S.astype(np.float32) * (s_gain/100.0), 0, 255)
    V = np.clip(V.astype(np.float32) * (v_gain/100.0), 0, 255)
    V = (np.clip(V/255.0,0,1)**max(0.3,float(gamma))) * 255.0
    hsv2 = cv2.merge([H, S.astype(np.uint8), V.astype(np.uint8)])
    return cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

def percentiles(a, p_low, p_high):
    """Return lower and upper percentiles of a NumPy array."""
    lo = np.percentile(a, p_low)
    hi = np.percentile(a, p_high)
    return float(lo), float(hi)

def tight_h_range(H, margin=5):
    """
    Compute a robust hue range with support for wrap-around (used for red).
    Returns:
        wrap (bool), hmin, hmax, R1, R2
    """
    H = H.astype(np.float32)
    h5, h95 = percentiles(H, 5, 95)
    hmin = max(0.0, h5 - margin)
    hmax = min(179.0, h95 + margin)

    # Detect wrap-around for red hue
    if (hmax - hmin) > 60:
        r1_hi = min(10.0+margin, hmax)
        r2_lo = max(170.0-margin, hmin)
        return True, hmin, hmax, (0.0, r1_hi), (r2_lo, 179.0)
    return False, hmin, hmax, None, None

def collect_pixels(frame, rect, ignore_white=True):
    """
    Extract H, S, V and LAB channels inside ROI.
    Filters out low-saturation / high-value pixels to ignore glare.
    """
    x0,y0,x1,y1 = rect
    roi = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    H,S,V = cv2.split(hsv)
    L,A,B = cv2.split(lab)

    mask = np.ones_like(S, dtype=bool)
    if ignore_white:
        mask &= (S > 40) & (V < 240)
    return H[mask], S[mask], V[mask], L[mask], A[mask], B[mask]

def show_text(img, lines, y0=28, color=(255,255,255)):
    """Display multiple lines of text on the frame."""
    y = y0
    for t in lines:
        cv2.putText(img, t, (10,y), FONT, 0.58, color, 2, cv2.LINE_AA)
        y += 22

def ensure_cfg(base=None):
    """
    Generate a full configuration dict with default values.
    This ensures no key is missing when saving.
    """
    d = dict(
        GainS_global=GAIN_S, GainV_global=GAIN_V, Gamma_x100=int(GAMMA*100),
        W_Smax=70, W_Vmin=190,
        O_Hmin=5, O_Hmax=30, O_Smin=80, O_Vmin=80,
        B_Hmin=100, B_Hmax=130, B_Smin=110, B_Vmin=40,
        G_Hmin=40, G_Hmax=85, G_Smin=70, G_Vmin=70,
        R1_Hmin=0, R1_Hmax=10, R2_Hmin=170, R2_Hmax=180, R_Smin=120, R_Vmin=70,
        L_min=0, L_max=95, A_tol=25, B_tol=25, Blk_Vmax=120, Blk_Smax=120,
    )
    if base: d.update(base)
    return d

def save_cfg(cfg, path=CONFIG_PATH):
    """Save configuration as JSON file."""
    with open(path,"w") as f:
        json.dump(cfg, f, indent=2)
    print(f"[OK] Saved to {path}")

# ===================== COLOR CALIBRATION =====================
def calibrate_color(cat_name, cap, rect, samples=5, is_red=False):
    """
    Capture multiple samples of a color and compute HSV thresholds.
    Special handling for red due to hue wrap-around.
    """
    allH, allS, allV = [], [], []
    for i in range(samples):
        while True:
            ok, fr = cap.read()
            if not ok: continue
            fr = apply_gains(fr)
            r = frac_to_rect(fr, rect)
            draw_roi(fr, r, f"{cat_name} — capture {i+1}/{samples}")
            show_text(fr, ["Press 'c' to capture", "Press 'q' to cancel"], y0=H-40)
            cv2.imshow("ColorCalibrator", fr)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('c'):
                Hc,Sc,Vc,_,_,_ = collect_pixels(fr, r, ignore_white=True)
                if Hc.size < 50:
                    print("[WARN] Not enough pixels, recapture.")
                    time.sleep(0.5)
                    continue
                allH.append(Hc); allS.append(Sc); allV.append(Vc)
                break
            elif k == ord('q'):
                raise SystemExit("[USER] Calibration cancelled.")

    Hc = np.concatenate(allH); Sc = np.concatenate(allS); Vc = np.concatenate(allV)
    Smin = max(0, int(percentiles(Sc, 20, 95)[0] - 20))
    Vmin = max(0, int(percentiles(Vc, 20, 95)[0] - 20))

    # Special handling for RED
    if is_red:
        wrap, hmin, hmax, r1, r2 = tight_h_range(Hc)
        if wrap and r1 and r2:
            return dict(
                R1_Hmin=int(round(r1[0])), R1_Hmax=int(round(r1[1])),
                R2_Hmin=int(round(r2[0])), R2_Hmax=int(round(r2[1])),
                R_Smin=int(Smin), R_Vmin=int(Vmin),
            )
        else:
            return dict(
                R1_Hmin=int(round(hmin)), R1_Hmax=int(round(hmax)),
                R2_Hmin=179, R2_Hmax=179,
                R_Smin=int(Smin), R_Vmin=int(Vmin),
            )
    else:
        h5,h95 = percentiles(Hc, 5, 95)
        Hmin = int(max(0, h5 - 5))
        Hmax = int(min(179, h95 + 5))
        if cat_name.upper().startswith("GREEN"):
            return dict(G_Hmin=Hmin, G_Hmax=Hmax, G_Smin=Smin, G_Vmin=Vmin)
        elif cat_name.upper().startswith("BLUE"):
            return dict(B_Hmin=Hmin, B_Hmax=Hmax, B_Smin=Smin, B_Vmin=Vmin)
        elif cat_name.upper().startswith("ORANGE"):
            return dict(O_Hmin=Hmin, O_Hmax=Hmax, O_Smin=Smin, O_Vmin=Vmin)
        else:
            return {}

def calibrate_black(cap, rect, samples=5):
    """
    Capture black wall/floor samples and calculate LAB + HSV thresholds.
    """
    allL, allA, allB, allS, allV = [], [], [], [], []
    for i in range(samples):
        while True:
            ok, fr = cap.read()
            if not ok: continue
            fr = apply_gains(fr)
            r = frac_to_rect(fr, rect)
            draw_roi(fr, r, f"BLACK — capture {i+1}/{samples}", (255,0,255))
            cv2.imshow("ColorCalibrator", fr)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('c'):
                _,_,_,L,A,B = collect_pixels(fr, r, ignore_white=False)
                hsv = cv2.cvtColor(fr[r[1]:r[3], r[0]:r[2]], cv2.COLOR_BGR2HSV)
                S = hsv[:,:,1].flatten(); V = hsv[:,:,2].flatten()
                allL.append(L); allA.append(A); allB.append(B); allS.append(S); allV.append(V)
                break
            elif k == ord('q'):
                raise SystemExit("[USER] Calibration cancelled.")

    L = np.concatenate(allL); A = np.concatenate(allA); B = np.concatenate(allB)
    S = np.concatenate(allS); V = np.concatenate(allV)
    Lmax = int(min(255, percentiles(L, 95, 99)[0] + 5))
    Amean = float(np.mean(A)); Bmean = float(np.mean(B))
    Atol = int(min(64, max(20, abs(Amean-128)+10)))
    Btol = int(min(64, max(20, abs(Bmean-128)+10)))
    Blk_Smax = int(min(255, percentiles(S, 90, 99)[0]))
    Blk_Vmax = int(min(255, percentiles(V, 90, 99)[0]))

    return dict(L_min=0, L_max=Lmax, A_tol=Atol, B_tol=Btol,
                Blk_Smax=Blk_Smax, Blk_Vmax=Blk_Vmax)

# ===================== MAIN =====================
def main():
    # --- Open camera ---
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened(): raise SystemExit("[ERROR] No camera.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # --- UI Window ---
    cv2.namedWindow("ColorCalibrator", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ColorCalibrator", 960, 720)

    # --- Load existing config if any ---
    try:
        with open(CONFIG_PATH,"r") as f:
            base = json.load(f)
    except Exception:
        base = None
    cfg = ensure_cfg(base)

    print("\n=== ColorCalibrator ===")
    print("Instructions:")
    print(" - Place the object inside the yellow ROI.")
    print(" - Press 'c' to capture each of the 5 samples.")
    print(" - Press 'q' to cancel.\n")

    # --- Calibration sequence ---
    cfg.update(calibrate_color("GREEN block", cap, ROI_BLOCK, samples=5, is_red=False))
    cfg.update(calibrate_color("RED block", cap, ROI_BLOCK, samples=5, is_red=True))
    cfg.update(calibrate_color("BLUE line", cap, ROI_LINE, samples=5, is_red=False))
    cfg.update(calibrate_color("ORANGE line", cap, ROI_LINE, samples=5, is_red=False))
    cfg.update(calibrate_black(cap, ROI_BLACK, samples=5))

    # --- Save configuration ---
    save_cfg(cfg, CONFIG_PATH)

    # --- Display summary window ---
    for t in range(60):
        ok, fr = cap.read()
        if not ok: break
        fr = apply_gains(fr)
        draw_roi(fr, frac_to_rect(fr, ROI_BLOCK), "ROI BLOCK", (0,255,255))
        draw_roi(fr, frac_to_rect(fr, ROI_LINE), "ROI LINE", (0,255,255))
        draw_roi(fr, frac_to_rect(fr, ROI_BLACK), "ROI BLACK", (255,0,255))
        show_text(fr, ["Calibration saved.", os.path.basename(CONFIG_PATH), "Press 'q' to close"], y0=28)
        cv2.imshow("ColorCalibrator", fr)
        k = cv2.waitKey(50) & 0xFF
        if k == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Done.")

if __name__ == "__main__":
    main()
