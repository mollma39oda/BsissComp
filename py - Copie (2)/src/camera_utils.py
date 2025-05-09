# camera_utils.py - Fonctions pour la gestion de la caméra

import cv2
import numpy as np
import requests
import time
from config import USE_WEBCAM, CAMERA_ID, ESP32_CAM_URL

def initialize_camera():
    """
    Initialise la caméra en fonction de la configuration
    
    Returns:
        Un objet VideoCapture si webcam ou None si ESP32-CAM, ainsi que les dimensions
    """
    if USE_WEBCAM:
        cap = cv2.VideoCapture(CAMERA_ID)
        if not cap.isOpened():
            raise IOError("Erreur : Impossible d'accéder à la caméra.")
        
        # Récupérer les dimensions de la frame
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return cap, frame_width, frame_height
    else:
        # Dimensions par défaut pour l'ESP32-CAM (à ajuster selon la configuration)
        return None, 320, 240

def get_frame(cap):
    """
    Récupère une frame depuis la webcam ou l'ESP32-CAM
    
    Args:
        cap: L'objet VideoCapture ou None si ESP32-CAM
        
    Returns:
        La frame capturée ou None en cas d'erreur
    """
    if USE_WEBCAM:
        ret, frame = cap.read()
        if not ret:
            return None
        
        # Retourner l'image horizontalement pour une expérience miroir
        frame = cv2.flip(frame, 1)
        return frame
    else:
        # Récupérer l'image de l'ESP32-CAM
        try:
            response = requests.get(ESP32_CAM_URL, timeout=3)
            img_array = np.array(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, -1)
            return frame
        except requests.exceptions.RequestException as e:
            print(f"Erreur de connexion à l'ESP32-CAM: {e}")
            return None

def send_ball_position(x, y):
    """
    Envoie la position de la balle à l'ESP32-CAM
    
    Args:,
        x: Position x de la balle
        y: Position y de la balle
        
    Returns:
        True si l'envoi a réussi, False sinon
    """
    if not USE_WEBCAM:
        from config import BALL_POSITION_URL
        try:
            requests.post(BALL_POSITION_URL, data={"x": x, "y": y}, timeout=1)
            return True
        except requests.exceptions.RequestException as e:
            print(f"Erreur lors de l'envoi des données à l'ESP32-CAM: {e}")
            return False
    return False

def release_resources(cap):
    """
    Libère les ressources de la caméra
    
    Args:
        cap: L'objet VideoCapture ou None si ESP32-CAM
    """
    if USE_WEBCAM and cap is not None:
        cap.release()
    cv2.destroyAllWindows()