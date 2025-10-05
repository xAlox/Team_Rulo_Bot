#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Team Rulo Bot â€” Vision (GO-armed) â€” Color decision with strong WHITE suppression
- Decide SOLO tras GO.
- â€œLÃ­nea gruesa y estableâ€: Ã¡rea mÃ­nima + grosor + N frames.
- Blanco fuertemente suprimido (HSV y LAB).
- Azul medio-oscuro: H azul, S>=110, V<=200 y L<=190.
- Primer color vÃ¡lido decide TODO el recorrido:
    ORANGE -> RIGHT (TURN_R)
    BLUE   -> LEFT  (TURN_L)
"""

import cv2, numpy as np, time, signal, sys
import serial, serial.tools.list_ports

# -------- Camera --------
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

# -------- Serial --------
def open_serial(baud=115200):
    for p in ["/dev/ttyUSB0","/dev/ttyACM0"]:
        try:
            return serial.Serial(p, baudrate=baud, timeout=0.01)
        except: pass
    for p in serial.tools.list_ports.comports():
        try:
            return serial.Serial(p.device, baudrate=baud, timeout=0.01)
        except: pass
    print("[WARN] No serial; vision-only.")
    return None

# -------- ROIs (fracciones x0,y0,x1,y1) --------
ROIS = {
    "LEFT_WALL":      (0.00, 0.25, 0.35, 0.40),
    "RIGHT_WALL":     (0.65, 0.25, 1.00, 0.40),
    "ORANGE_BOT":     (0.25, 0.54, 0.75, 0.73),  # color decision ROI (azul/naranja)
    # FRONT por defecto (para RIGHT). Mantengo tu ROI actual:
    "FRONT_SQ_RIGHT": (0.50, 0.17, 0.57, 0.45),  # <<< NEW >>>
    # FRONT alternativo (para LEFT): mismas alturas, desplazado a la derecha:
    "FRONT_SQ_LEFT":  (0.43, 0.17, 0.50, 0.45),  # <<< NEW >>>
    "FRONT_SIDE_L":   (0.05, 0.42, 0.20, 0.52),
    "FRONT_SIDE_R":   (0.80, 0.42, 0.95, 0.52),
}

# -------- Rangos / Umbrales de color --------
LAB_BLACK     = (np.array([0,120,120],np.uint8),  np.array([70,136,136],np.uint8))
HSV_ORANGE_1  = (np.array([5,80,80],np.uint8),    np.array([20,255,255],np.uint8))
HSV_ORANGE_2  = (np.array([15,50,80],np.uint8),   np.array([30,255,255],np.uint8))
HSV_BLUE_DARK = (np.array([100,110,40],np.uint8), np.array([130,255,200],np.uint8))
HSV_WHITE     = (np.array([0,0,190],np.uint8),    np.array([179,70,255],np.uint8))
LAB_L_MAX_FOR_BLUE = 190
KERNEL        = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
PURPLE        = (180,0,180)
MIN_A_BLACK   = 80

# -------- Umbrales de decisiÃ³n (lÃ­nea gruesa + estabilidad) --------
MIN_COLOR_AREA_FRAC   = 0.02
MIN_COLOR_THICK_PX    = 14
COLOR_STABLE_FRAMES   = 3
COLOR_DECISION_MARGIN = 1.35

# -------- LÃ³gica frontal/laterales --------
FRONT_BLACK_AREA_THRESH = 2500.0
TURN_COOLDOWN_S         = 3
MAX_TURNS               = 12
SIDE_BLACK_AREA_THRESH  = 1400.0
NUDGE_DEG               = 13
NUDGE_MS                = 300
NUDGE_COOLDOWN_S        = 0.25

# -------- Helpers --------
def frac_to_rect(img, f):
    h,w = img.shape[:2]
    return (int(f[0]*w), int(f[1]*h), int(f[2]*w), int(f[3]*h))

def draw_roi(frame, rect, label, color=(255,200,0)):
    x0,y0,x1,y1 = rect
    cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
    cv2.putText(frame,label,(x0+4,y0+18),FONT,0.55,color,2,cv2.LINE_AA)

def detect_black_excluding_blue_mask(roi_bgr, roi_lab):
    m_lab  = cv2.inRange(roi_lab, LAB_BLACK[0], LAB_BLACK[1])
    m      = cv2.GaussianBlur(m_lab,(7,7),0)
    m      = cv2.erode(m,KERNEL,1)
    m      = cv2.dilate(m,KERNEL,1)
    return m

def detect_black_excluding_blue_and_draw(frame, bgr_img, lab_img, rect):
    x0,y0,x1,y1 = rect
    mask = detect_black_excluding_blue_mask(bgr_img[y0:y1,x0:x1], lab_img[y0:y1,x0:x1])
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if cnts:
        roi_draw = frame[y0:y1, x0:x1]
        for c in cnts:
            a = cv2.contourArea(c)
            if a < MIN_A_BLACK: continue
            cv2.drawContours(roi_draw,[c],-1,PURPLE,2)

def white_mask_hsv(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, HSV_WHITE[0], HSV_WHITE[1])
    m = cv2.medianBlur(m,5)
    return m

def too_bright_mask_lab(roi_bgr):
    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    L = lab[:,:,0]
    return (L > LAB_L_MAX_FOR_BLUE).astype(np.uint8) * 255

def detect_orange_line(frame_bgr, rect):
    x0,y0,x1,y1 = rect
    roi = frame_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, HSV_ORANGE_1[0], HSV_ORANGE_1[1])
    m2 = cv2.inRange(hsv, HSV_ORANGE_2[0], HSV_ORANGE_2[1])
    m  = cv2.bitwise_or(m1,m2)
    mw = white_mask_hsv(roi)
    m  = cv2.bitwise_and(m, cv2.bitwise_not(mw))
    m  = cv2.GaussianBlur(m,(7,7),0); m = cv2.erode(m,KERNEL,1); m = cv2.dilate(m,KERNEL,1)
    cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best=None
    if cnts:
        for c in cnts:
            a=cv2.contourArea(c)
            if a<150: continue
            x,y,w,h=cv2.boundingRect(c)
            if best is None or a>best[0]:
                best=(a,(x0+x,y0+y,w,h),m)
    return best

def detect_blue_line(frame_bgr, rect):
    x0,y0,x1,y1 = rect
    roi = frame_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    m_hsv = cv2.inRange(hsv, HSV_BLUE_DARK[0], HSV_BLUE_DARK[1])
    mw   = white_mask_hsv(roi)
    mL   = too_bright_mask_lab(roi)
    suppress = cv2.bitwise_or(mw, mL)
    m = cv2.bitwise_and(m_hsv, cv2.bitwise_not(suppress))
    m  = cv2.GaussianBlur(m,(7,7),0)
    m  = cv2.erode(m,KERNEL,1)
    m  = cv2.dilate(m,KERNEL,1)
    cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best=None
    if cnts:
        for c in cnts:
            a=cv2.contourArea(c)
            if a<150: continue
            x,y,w,h=cv2.boundingRect(c)
            if best is None or a>best[0]:
                best=(a,(x0+x,y0+y,w,h),m)
    return best

# Graceful exit
_stop = False
def _sigint(_s, _f):
    global _stop
    _stop = True

def main():
    signal.signal(signal.SIGINT, _sigint)
    cap = open_cam(CAM_INDEX)
    ser = open_serial(115200)

    win = "Team Rulo Bot â€” Vision (GO-armed)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 960, 720)

    armed=False; turn_count=0; last_turn_ts=0.0; last_nudge_ts=0.0
    selected_turn=None
    orange_hits=blue_hits=0
    o_area_frac=b_area_frac=0.0

    rx_buf=b""; t0=time.time(); n=0; fps=0.0

    while not _stop:
        ok, frame = cap.read()
        if not ok: continue
        n+=1; t1=time.time()
        if (t1-t0)>=1.0: fps=n/(t1-t0); n=0; t0=t1

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        # --- ROIs fijos ---
        leftR   = frac_to_rect(frame, ROIS["LEFT_WALL"])
        rightR  = frac_to_rect(frame, ROIS["RIGHT_WALL"])
        orangeR = frac_to_rect(frame, ROIS["ORANGE_BOT"])
        sideL   = frac_to_rect(frame, ROIS["FRONT_SIDE_L"])
        sideR   = frac_to_rect(frame, ROIS["FRONT_SIDE_R"])

        # --- ROI FRONT dependiente del sentido ---  # <<< NEW >>>
        if selected_turn == 'L':
            frontR = frac_to_rect(frame, ROIS["FRONT_SQ_LEFT"])
            front_label = "FRONT (LEFT ROI)"
        else:
            frontR = frac_to_rect(frame, ROIS["FRONT_SQ_RIGHT"])
            front_label = "FRONT (RIGHT ROI)"
        # -----------------------------------------  # <<< NEW >>>

        draw_roi(frame,leftR,"LEFT_WALL"); draw_roi(frame,rightR,"RIGHT_WALL")
        draw_roi(frame,orangeR,"COLOR_PICK (post-GO)")
        draw_roi(frame,frontR,front_label,(0,255,255))   # <<< NEW >>>
        draw_roi(frame,sideL,"SIDE_L",(0,255,0)); draw_roi(frame,sideR,"SIDE_R",(0,255,0))

        detect_black_excluding_blue_and_draw(frame, frame, lab, leftR)
        detect_black_excluding_blue_and_draw(frame, frame, lab, rightR)

        # RX serial
        if ser and ser.in_waiting:
            try:
                rx_buf += ser.read(ser.in_waiting)
                while b"\n" in rx_buf:
                    line, rx_buf = rx_buf.split(b"\n",1)
                    s=line.decode(errors='ignore').strip().upper()
                    if s=="GO":
                        armed=True; turn_count=0; last_turn_ts=0.0; last_nudge_ts=0.0
                        selected_turn=None; orange_hits=blue_hits=0
                        print("[INFO] ARMED by GO (color decision starts)")
                    elif s=="STOP":
                        armed=False; print("[INFO] DISARMED by STOP")
            except Exception as e:
                print(f"[WARN] Serial read error: {e}")

        now=time.time()

        # DecisiÃ³n SOLO con armed y si aÃºn no estÃ¡ elegido
        if armed and selected_turn is None:
            x0,y0,x1,y1 = orangeR
            roi_area = max(1,(x1-x0)*(y1-y0))

            O = detect_orange_line(frame, orangeR)
            if O:
                oa,(ox,oy,ow,oh),om = O
                o_area_frac = oa/roi_area
                cv2.rectangle(frame,(ox,oy),(ox+ow,oy+oh),(0,140,255),2)
                if o_area_frac>=MIN_COLOR_AREA_FRAC and oh>=MIN_COLOR_THICK_PX:
                    orange_hits += 1
                else:
                    orange_hits = 0
            else:
                o_area_frac=0.0; orange_hits=0

            B = detect_blue_line(frame, orangeR)
            if B:
                ba,(bx,by,bw,bh),bm = B
                b_area_frac = ba/roi_area
                cv2.rectangle(frame,(bx,by),(bx+bw,by+bh),(255,120,0),2)
                roi = frame[y0:y1, x0:x1]
                cv2.addWeighted(roi,0.7,cv2.cvtColor(bm,cv2.COLOR_GRAY2BGR),0.3,0,roi)
                if b_area_frac>=MIN_COLOR_AREA_FRAC and bh>=MIN_COLOR_THICK_PX:
                    blue_hits += 1
                else:
                    blue_hits = 0
            else:
                b_area_frac=0.0; blue_hits=0

            # Decide con margen
            if orange_hits>=COLOR_STABLE_FRAMES and (blue_hits==0 or o_area_frac>=b_area_frac*COLOR_DECISION_MARGIN):
                selected_turn='R'; print("[MODE] TURN RIGHT (orange)")
            elif blue_hits>=COLOR_STABLE_FRAMES and (orange_hits==0 or b_area_frac>=o_area_frac*COLOR_DECISION_MARGIN):
                selected_turn='L'; print("[MODE] TURN LEFT (blue)")

        # Acciones si armed
        if armed and turn_count<MAX_TURNS:
            fx0,fy0,fx1,fy1 = frontR
            fm = detect_black_excluding_blue_mask(frame[fy0:fy1,fx0:fx1], lab[fy0:fy1,fx0:fx1])
            fr_overlay = frame[fy0:fy1,fx0:fx1]
            cv2.addWeighted(fr_overlay,0.7,cv2.cvtColor(fm,cv2.COLOR_GRAY2BGR),0.3,0,fr_overlay)
            front_area = float(cv2.countNonZero(fm))

            if front_area>=FRONT_BLACK_AREA_THRESH and (now-last_turn_ts)>=TURN_COOLDOWN_S:
                if ser and selected_turn is not None:
                    try:
                        if selected_turn=='R':
                            ser.write(b"TURN_R\n"); print("[TX] TURN_R")
                        else:
                            ser.write(b"TURN_L\n"); print("[TX] TURN_L")
                    except Exception as e:
                        print(f"[WARN] Serial write error: {e}")
                turn_count+=1; last_turn_ts=now

            # SIDE nudges
            slx0,sly0,slx1,sly1 = sideL; srx0,sry0,srx1,sry1 = sideR
            ml = detect_black_excluding_blue_mask(frame[sly0:sly1,slx0:slx1], lab[sly0:sly1,slx0:slx1])
            mr = detect_black_excluding_blue_mask(frame[sry0:sry1,srx0:srx1], lab[sry0:sry1,srx0:srx1])
            cv2.addWeighted(frame[sly0:sly1,slx0:slx1],0.7,cv2.cvtColor(ml,cv2.COLOR_GRAY2BGR),0.3,0,frame[sly0:sly1,slx0:slx1])
            cv2.addWeighted(frame[sry0:sry1,srx0:srx1],0.7,cv2.cvtColor(mr,cv2.COLOR_GRAY2BGR),0.3,0,frame[sry0:sry1,srx0:srx1])
            al = float(cv2.countNonZero(ml)); ar = float(cv2.countNonZero(mr))

            if (now-last_nudge_ts)>=NUDGE_COOLDOWN_S:
                if al>=SIDE_BLACK_AREA_THRESH or ar>=SIDE_BLACK_AREA_THRESH:
                    if al>=ar and al>=SIDE_BLACK_AREA_THRESH:
                        cmd=f"NUDGE_R:{NUDGE_DEG}:{NUDGE_MS}\n".encode()
                    elif ar>al and ar>=SIDE_BLACK_AREA_THRESH:
                        cmd=f"NUDGE_L:{NUDGE_DEG}:{NUDGE_MS}\n".encode()
                    else:
                        cmd=None
                    if ser and cmd:
                        try: ser.write(cmd); print("[TX]",cmd.decode().strip())
                        except Exception as e: print(f"[WARN] Serial write error: {e}")
                        last_nudge_ts=now

            if turn_count>=MAX_TURNS and ser:
                try: ser.write(b"STOP\n"); print("[TX] STOP")
                except Exception as e: print(f"[WARN] Serial write error: {e}")
                armed=False

        dir_txt = {None:"WAIT_COLOR", 'R':"RIGHT", 'L':"LEFT"}[selected_turn]
        hud = f"{FRAME_W}x{FRAME_H} ~{fps:.1f}fps  armed:{int(armed)}  turns:{turn_count}/{MAX_TURNS}  DIR:{dir_txt}  O:{int(100*o_area_frac)}%  B:{int(100*b_area_frac)}%"
        cv2.putText(frame, hud, (10,20), FONT, 0.6, (255,255,255), 2, cv2.LINE_AA)

        cv2.imshow(win, frame)
        if (cv2.waitKey(1) & 0xFF)==ord('q'): break

    if ser:
        try: ser.close()
        except: pass
    cap.release(); cv2.destroyAllWindows()
    print("[INFO] Bye.")

if _name_ == "_main_":
    main()
