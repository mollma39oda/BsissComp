# ui_utils.py - Fonctions pour l'affichage à l'écran

import cv2
from config import DEAD_ZONE_WIDTH

def draw_detection_ui(frame, ball_x, ball_y, ball_radius, command, ball_detected, 
                      frame_width, frame_height, distance,using_circle_detection=True, using_corners=False):

    result_frame = frame.copy()
    
    # Dessiner la zone centrale (zone morte) en bleu
    cv2.rectangle(result_frame,
                 (int(frame_width / 2 - DEAD_ZONE_WIDTH / 2), 0),
                 (int(frame_width / 2 + DEAD_ZONE_WIDTH / 2), frame_height),
                 (255, 0, 0), 2)
    
    # Si une balle est détectée
    if ball_detected and ball_x is not None and ball_y is not None and ball_radius is not None:
        if using_circle_detection:
            # Dessiner le cercle et son centre
            cv2.circle(result_frame, (ball_x, ball_y), ball_radius, (0, 255, 0), 2)
            cv2.circle(result_frame, (ball_x, ball_y), 2, (0, 0, 255), 3)
            
            # Afficher les coordonnées
            cv2.putText(result_frame, f"Ball ({ball_x}, {ball_y})", 
                        (ball_x - 40, ball_y - ball_radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        elif using_corners:
            color = [255, 0, 0]
            thickness = 2
            
            # Calcul des coins
            TopLeft = [
                (ball_x - ball_radius - 2, ball_y - ball_radius - 2),
                (ball_x - ball_radius - 2 + ball_radius // 4, ball_y - ball_radius - 2),
                (ball_x - ball_radius - 2, ball_y - ball_radius - 2 + ball_radius // 4)
            ]
            TopRight = [
                (ball_x + ball_radius + 2, ball_y - ball_radius - 2),
                (ball_x + ball_radius + 2 - ball_radius // 4, ball_y - ball_radius - 2),
                (ball_x + ball_radius + 2, ball_y - ball_radius - 2 + ball_radius // 4)
            ]
            BottomLeft = [
                (ball_x - ball_radius - 2, ball_y + ball_radius + 2),
                (ball_x - ball_radius - 2 + ball_radius // 4, ball_y + ball_radius + 2),
                (ball_x - ball_radius - 2, ball_y + ball_radius + 2 - ball_radius // 4)
            ]
            BottomRight = [
                (ball_x + ball_radius + 2, ball_y + ball_radius + 2),
                (ball_x + ball_radius + 2 - ball_radius // 4, ball_y + ball_radius + 2),
                (ball_x + ball_radius + 2, ball_y + ball_radius + 2 - ball_radius // 4)
            ]
            
            # Dessiner les coins
            cv2.line(result_frame, TopLeft[0], TopLeft[1], color, thickness)
            cv2.line(result_frame, TopLeft[0], TopLeft[2], color, thickness)
            cv2.line(result_frame, TopRight[0], TopRight[1], color, thickness)
            cv2.line(result_frame, TopRight[0], TopRight[2], color, thickness)
            cv2.line(result_frame, BottomLeft[0], BottomLeft[1], color, thickness)
            cv2.line(result_frame, BottomLeft[0], BottomLeft[2], color, thickness)
            cv2.line(result_frame, BottomRight[0], BottomRight[1], color, thickness)
            cv2.line(result_frame, BottomRight[0], BottomRight[2], color, thickness)
            cv2.putText(result_frame, f"(x,y,r) = {(ball_x,ball_y,ball_radius)}", (ball_x-ball_radius-10,ball_y-ball_radius-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        else:
            # Dessiner un rectangle autour de la balle
            rect_size = ball_radius * 2
            rect_x = ball_x - ball_radius
            rect_y = ball_y - ball_radius
            cv2.rectangle(result_frame, 
                         (rect_x, rect_y), 
                         (rect_x + rect_size, rect_y + rect_size), 
                         (0, 165, 255), 2)
            
            # Dessiner le centre
            cv2.circle(result_frame, (ball_x, ball_y), 2, (0, 0, 255), 3)
    
        # Afficher la commande
        if command:
            cv2.putText(result_frame, f"CMD: {command} (x,y,r,d) = {(ball_x,ball_y,ball_radius,distance)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    else:
        # Afficher un message si aucune balle n'est détectée
        cv2.putText(result_frame, "Aucune balle detectee", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    return result_frame
