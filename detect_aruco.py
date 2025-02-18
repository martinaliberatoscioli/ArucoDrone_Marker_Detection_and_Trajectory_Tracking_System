import os
import cv2
import numpy as np


os.environ["QT_QPA_PLATFORM"] = "xcb"

# Percorso del marker ArUco
ARUCO_IMAGE_PATH = "/home/martina/PX4-Autopilot/Tools/simulation/gz/models/arucotag/arucotag.png"

def detect_aruco_and_show_image(image_path):
    """Carica l'immagine, pre-processa e rileva l'ArUco"""
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"Errore: Impossibile aprire {image_path}. Controlla il percorso.")
        return

    print(f"Dimensioni dell'immagine: {frame.shape}")

    cv2.imshow("Immagine Originale", frame)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

    
    print("Valori min/max dei pixel:", frame.min(), frame.max())

    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow("Scala di Grigi", gray)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

    # aggiunta una soglia binaria per rendere più chiaro il marker
    _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    
    cv2.imshow("Immagine Binarizzata", binary)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

    # Detection ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("Debug: Tentativo di rilevare un marker ArUco DICT_6X6_250")
    corners, ids, _ = detector.detectMarkers(binary)

    if ids is None:
        print("Nessun marker rilevato, provo a ruotare l'immagine di 180°.")
        rotated = cv2.rotate(binary, cv2.ROTATE_180)
        corners, ids, _ = detector.detectMarkers(rotated)

    if ids is not None:
        print(f"Marker ArUco rilevato con ID: {ids.flatten()}")
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow('Detected Markers', frame)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()
    else:
        print("Nessun ArUco Marker rilevato.")

detect_aruco_and_show_image(ARUCO_IMAGE_PATH)
