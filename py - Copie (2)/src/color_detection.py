# color_detection.py - Fonctions pour la détection de couleurs

import cv2
import numpy as np
from config import (
    LOWER_RED1, UPPER_RED1, LOWER_RED2, UPPER_RED2,
    LOWER_GREEN_YELLOW1, UPPER_GREEN_YELLOW1,
    LOWER_GREEN_YELLOW2, UPPER_GREEN_YELLOW2,
    KERNEL
)

def create_color_mask(frame, color_type):
    """
    Crée un masque pour isoler les objets de la couleur spécifiée

    Args:
        frame: Image à traiter (format BGR)
        color_type: Type de couleur ("red" ou "green_yellow")

    Returns:
        Le masque binaire et le masque flouté
    """
    # Vérifier si l'image est valide
    if frame is None:
        return None, None

    # Appliquer un flou pour lisser les petites variations
    frame_blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # Conversion en HSV
    hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)

    # Égalisation de l'histogramme sur le canal V (luminosité)
    h, s, v = cv2.split(hsv)
    v_eq = cv2.equalizeHist(v)
    hsv = cv2.merge((h, s, v_eq))

    # Création du masque selon la couleur demandée
    if color_type == "red":
        mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)
    elif color_type == "green_yellow":
        mask1 = cv2.inRange(hsv, LOWER_GREEN_YELLOW1, UPPER_GREEN_YELLOW1)
        mask2 = cv2.inRange(hsv, LOWER_GREEN_YELLOW2, UPPER_GREEN_YELLOW2)
        mask = cv2.bitwise_and(mask1, mask2)
    else:
        raise ValueError("Type de couleur non reconnu. Utilisez 'red' ou 'green_yellow'.")

    # Nettoyage du masque avec des opérations morphologiques
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    # Floutage du masque pour lisser les contours
    blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2)

    return mask, blurred_mask
