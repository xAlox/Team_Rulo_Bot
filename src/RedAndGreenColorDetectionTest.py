# -*- coding: utf-8 -*-
# Team Rulo Bot
# ES: Detección simple de ROJO y VERDE con webcam USB usando OpenCV.
# EN: Simple detection of RED and GREEN with a USB webcam using OpenCV.

import cv2
import numpy as np

# ------------------ Parámetros ajustables / Tunables ------------------
# ES: Índice de cámara (0 si tienes una sola webcam)
# EN: Camera index (0 if you have a single webcam)
CAM_INDEX = 0

# ES: Resolución objetivo (la cámara puede no soportar exactamente estos valores)
# EN: Target resolution (camera may not support these exact values)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ES: Área mínima de contorno para dibujar caja
# EN: Minimum contour area to draw a box
AREA_MIN = 1000

# ES: Kernel morfológico para limpiar la máscara (ruido)
# EN: Morphological kernel to clean the mask (noise)
K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# ES: Rangos HSV para VERDE (luz interior típica); ajusta S y V si ves falsos positivos.
# EN: HSV ranges for GREEN (typical indoor light); adjust S and V if you see false positives.
LOWER_GREEN = np.array([35, 70, 60], dtype=np.uint8)   # H,S,V
UPPER_GREEN = np.array([85, 255, 255], dtype=np.uint8)

# ES: ROJO en HSV cruza 0°, por eso usamos dos rangos.
# EN: RED in HSV wraps around 0°, so we use two ranges.
LOWER_RED1 = np.array([0, 120, 70], dtype=np.uint8)
UPPER_RED1 = np.array([10, 255, 255], dtype=np.uint8)
LOWER_RED2 = np.array([170, 120, 70], dtype=np.uint8)
UPPER_RED2 = np.array([180, 255, 255], dtype=np.uint8)

# ------------------ Funciones / Functions ------------------
def preprocess(bgr):
    # ES: Suaviza y convierte a HSV
    # EN: Smooth and convert to HSV
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    return hsv

def make_mask(hsv, lo, hi):
    # ES: Máscara + limpieza morfológica
    # EN: Mask + morphological cleanup
    m = cv2.inRange(hsv, lo, hi)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, K, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, K, iterations=2)
    return m

def draw_detections(frame, mask, color_bgr, label, area_min=AREA_MIN):
    # ES: Busca contornos y dibuja caja/etiqueta
    # EN: Find contours and draw box/label
    found = False
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < area_min:
            continue
        x, y, w, h = cv2.boundingRect(c)
        # ES: Rectángulo y centroide
        # EN: Rectangle and centroid
        cv2.rectangle(frame, (x, y), (x+w, y+h), color_bgr, 2)
        cx, cy = x + w//2, y + h//2
        cv2.circle(frame, (cx, cy), 4, color_bgr, -1)
        # ES: Etiqueta con área en píxeles
        # EN: Label with pixel area
        cv2.putText(frame, f"{label} ({int(area)})", (x, max(0, y-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2, cv2.LINE_AA)
        found = True
    return found

def main():
    # ES: Abrir webcam (forzamos backend V4L2 en Linux si está disponible)
    # EN: Open webcam (force V4L2 backend on Linux if available)
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        # ES: Intento genérico si V4L2 no está
        # EN: Fallback if V4L2 is not present
        cap = cv2.VideoCapture(CAM_INDEX)

    # ES: Intentamos fijar resolución/FPS
    # EN: Try to set resolution/FPS
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("❌ ES: No se pudo abrir la webcam | EN: Could not open webcam")
        return

    print("✅ ES: Detectando ROJO/VERDE. 'q' para salir | EN: Detecting RED/GREEN. Press 'q' to quit")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("⚠️ ES: Frame no válido | EN: Invalid frame")
            continue

        hsv = preprocess(frame)

        # ES: Máscaras por color
        # EN: Color masks
        mask_g = make_mask(hsv, LOWER_GREEN, UPPER_GREEN)
        mask_r1 = make_mask(hsv, LOWER_RED1, UPPER_RED1)
        mask_r2 = make_mask(hsv, LOWER_RED2, UPPER_RED2)
        mask_r = cv2.bitwise_or(mask_r1, mask_r2)

        # ES: Conteo de píxeles (útil para lógicas de decisión)
        # EN: Pixel count (useful for decision logic)
        green_pixels = int(cv2.countNonZero(mask_g))
        red_pixels = int(cv2.countNonZero(mask_r))

        # ES: Dibujo de detecciones (BGR correcto: rojo=(0,0,255), verde=(0,255,0))
        # EN: Draw detections (correct BGR: red=(0,0,255), green=(0,255,0))
        found_g = draw_detections(frame, mask_g, (0, 255, 0), "VERDE / GREEN")
        found_r = draw_detections(frame, mask_r, (0, 0, 255), "ROJO / RED")

        # ES: Resumen arriba a la izquierda
        # EN: Summary top-left
        status = []
        if found_g: status.append("G")
        if found_r: status.append("R")
        status = " & ".join(status) if status else "NONE"
        cv2.putText(frame,
                    f"ES: G={green_pixels} R={red_pixels} | EN: G={green_pixels} R={red_pixels} | {status}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

        # ES: Mostrar
        # EN: Display
        cv2.imshow("ES/EN: Deteccion Rojo/Verde (Webcam)", frame)
        cv2.imshow("ES/EN: Mask VERDE / GREEN", mask_g)
        cv2.imshow("ES/EN: Mask ROJO / RED", mask_r)

        # ES: Salir con 'q'
        # EN: Quit with 'q'
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
