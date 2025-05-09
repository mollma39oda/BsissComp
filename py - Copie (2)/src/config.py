# config.py - Configuration et constantes pour le projet de détection de balles

import numpy as np

# Configuration de la caméra
USE_WEBCAM = False  # Mettre à False pour utiliser ESP32-CAM
CAMERA_ID = 1 # ID de la caméra à utiliser si USE_WEBCAM est True
ESP32_IP = "192.168.8.16"  # Adresse IP de l'ESP32-CAM

# URLs pour l'ESP32-CAM
ESP32_CAM_URL = f"http://{ESP32_IP}/cam-lo.jpg"
BALL_POSITION_URL = f"http://{ESP32_IP}/ball_position"

# Couleur à détecter
COLOR_TO_DETECT = "green_yellow"  # "red" ou "green_yellow"

# Paramètres d'affichage
SHOW_MASKS = True  # Si True, affiche aussi les masques

SHOW_MASKS_BITWISE = True  # Si True, affiche aussi les masques bitwise

SHOW_FRAME = True  # Si True, affiche aussi les frames

DISPLAY_WITH_IMSHOW =  True
DISPLAY_WITH_APPUI = False  
# Plages de couleur pour la détection Rouge
# [hue, saturation, valeur] (HSV)
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([179, 255, 255])


# Vert-Jaune — Plage large pour cas général
LOWER_GREEN_YELLOW1 = np.array([25, 50, 50])
UPPER_GREEN_YELLOW1 = np.array([45, 255, 255])

# Vert-Jaune — Plage plus précise pour conditions lumineuses spécifiques
#HSV Min: 29,184,104 Max: 41,255,255
LOWER_GREEN_YELLOW2 = np.array([29, 184, 104])
UPPER_GREEN_YELLOW2 = np.array([41, 255, 255])

LOWER_GREEN_YELLOW4= np.array([27, 201, 147])
UPPER_GREEN_YELLOW4= np.array([34, 232, 255])

LOWER_GREEN_YELLOW3 = np.array([0, 63, 206])
UPPER_GREEN_YELLOW3 = np.array([79, 255, 255])


"""
# Définir la plage de couleur vert-jaune en HSV
lower_green_yellow = np.array([25, 50, 50])
upper_green_yellow = np.array([45, 255, 255])
"""

# Paramètres pour les opérations morphologiques
KERNEL = np.ones((5, 5), np.uint8)

# Paramètres pour la détection de cercles
HOUGH_PARAM1 = 100
HOUGH_PARAM2 = 30
MIN_RADIUS = 10
MAX_RADIUS = 100

# Paramètre pour filtrer les contours
MIN_CONTOUR_AREA = 300

# Zone morte (en pixels depuis le centre)
DEAD_ZONE_WIDTH = 160  # Largeur totale de la zone morte

#KNOWN_DISTANCE,KNOWN8WIDTH
KNOWN_DISTANCE = 20  # Distance connue (en cm)
KNOWN_WIDTH = 5 # Largeur connue de l'objet (en cm)