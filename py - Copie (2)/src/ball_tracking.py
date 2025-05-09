import cv2
import numpy as np
from config import HOUGH_PARAM1, HOUGH_PARAM2, MIN_RADIUS, MAX_RADIUS, MIN_CONTOUR_AREA, DEAD_ZONE_WIDTH

def detect_ball(blurred_mask, frame, frame_width, frame_height):
    """
    Détecte les balles en utilisant la détection de cercles et de contours
    
    Args:
        blurred_mask: Masque flouté
        frame: Image originale
        frame_width: Largeur de l'image
        frame_height: Hauteur de l'image
    
    Returns:
        Position x, position y, rayon et commande
    """
    # Initialisation des variables
    ball_pos_x = None
    ball_pos_y = None
    ball_radius = None
    command = None
    ball_detected = False
    
    if blurred_mask is None or frame is None:
        return ball_pos_x, ball_pos_y, ball_radius, command, ball_detected
    
    CannyImg = cv2.Canny(blurred_mask, 100, 200)
    left_threshold = int(frame_width / 2 - DEAD_ZONE_WIDTH / 2)
    right_threshold = int(frame_width / 2 + DEAD_ZONE_WIDTH / 2)

    # Méthode 1 : Hough Circles
    circles = cv2.HoughCircles(CannyImg, cv2.HOUGH_GRADIENT, 1,
                                50, param1=HOUGH_PARAM1, param2=HOUGH_PARAM2,
                                minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        if circles[0].size > 0:
            x, y, r = int(circles[0][0][0]), int(circles[0][0][1]), int(circles[0][0][2])
            
            if x < left_threshold:
                command = "LEFT"
            elif x > right_threshold:
                command = "RIGHT"
            else:
                command = "FORWARD"
            
            ball_pos_x = x
            ball_pos_y = y
            ball_radius = r
            ball_detected = True

    # METHODE 2 : Détection de formes géométriques si aucun cercle détecté
    if not ball_detected:
        contours, _ = cv2.findContours(CannyImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_CONTOUR_AREA:
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                obj_corners = len(approx)
                x, y, w, h = cv2.boundingRect(approx)

                center_x = x + w // 2
                center_y = y + h // 2
                approx_radius = (w + h) // 4

                if obj_corners > 4:  # Forme arrondie détectée (cercle, ellipse…)
                    if center_x < left_threshold:
                        command = "LEFT"
                    elif center_x > right_threshold:
                        command = "RIGHT"
                    else:
                        command = "FORWARD"
                    
                    ball_pos_x = center_x
                    ball_pos_y = center_y
                    ball_radius = approx_radius
                    ball_detected = True
                    break  # On arrête après la première forme circulaire détectée

    return ball_pos_x, ball_pos_y, ball_radius, command, ball_detected
