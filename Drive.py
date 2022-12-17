import cv2
import numpy as np
import time

# Kamera öffnen
camera = cv2.VideoCapture(0)

# Bildaufnahme-Einstellungen anpassen (optional)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Funktion zum Einstellen des Servo-Winkels
def set_angle(angle):
    if angel > 0:
        duty_cycle = angle + 90
    elif angel < 0:
        duty_cycle = angle - 90
        pwm.ChangeDutyCycle(duty_cycle)
    else:
        #geradeaus

while True:
    # Bild aufnehmen
    ret, frame = camera.read()

    # Bild in Graustufen umwandeln
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Kantenerkennung durchführen
    edges = cv2.Canny(gray, 50, 150, apertureSize = 3)

    # Linien mit Hough-Transformation erkennen
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    # Mitte des Bildes bestimmen
    image_center = frame.shape[1] // 2

    # Linienpositionen bestimmen
    line_positions = []
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        line_positions.append((x1 + x2) // 2)

    # Mittelpunkt zwischen den Linien bestimmen
    center_pos = (line_positions[0] + line_positions[1])
    
    # Winkel zwischen Mittelpunkt und Mitte des Bildes berechnen
    angle = np.arctan2(center_pos - image_center, frame.shape[0])

    # Winkel in Grad umrechnen
    angle = angle * 180 / np.pi

    # Servo entsprechend einstellen
    set_angle(angle)


