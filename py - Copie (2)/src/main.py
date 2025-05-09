# main.py - Point d'entrée principal du programme de détection de balles
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
import time
from config import USE_WEBCAM, COLOR_TO_DETECT,DISPLAY_WITH_APPUI,DISPLAY_WITH_IMSHOW
from camera_utils import initialize_camera, get_frame, send_ball_position, release_resources
from color_detection import create_color_mask
from ball_tracking import detect_ball
from ui_utils import draw_detection_ui
from ui_desplay import CV_Imshow_Display,App_Display
from algos_vision import distanceMain,Distance_Init


def main():
    print(f"Démarrage de la détection de balles...")
    print(f"Mode: {'Webcam' if USE_WEBCAM else 'ESP32-CAM'}")
    print(f"Couleur à détecter: {COLOR_TO_DETECT}")
    
    try:
        # Initialiser la caméra
        cap, frame_width, frame_height = initialize_camera()
        ref = cv2.imread("Ref_image.jpg")
        _,blurred_ref = create_color_mask(ref, COLOR_TO_DETECT)
        Focal_length_found =Distance_Init(blurred_ref)
        
        print(f"Dimensions de l'image: {frame_width}x{frame_height}")
        print("Appuyez sur 'q' pour quitter.")
        
        while True:
            # Récupérer une frame
            frame = get_frame(cap)
            
            if frame is None:
                print("Erreur lors de la capture d'image. Nouvelle tentative...")
                time.sleep(1)
                continue
            
            # Créer le masque pour la détection de couleur
            mask, blurred_mask = create_color_mask(frame, COLOR_TO_DETECT)
            
            if mask is None or blurred_mask is None:
                continue
            
            # Détecter les balles
            ball_x, ball_y, ball_radius, command, ball_detected = detect_ball(
                blurred_mask, frame, frame_width, frame_height)
            
            # Calculer la distance
            distance =  distanceMain(blurred_mask, frame, frame_width, frame_height,Focal_length_found)
            # Créer l'interface utilisateur
            using_circle_detection = ball_detected and ball_radius is not None
            result_frame = draw_detection_ui(
                frame, ball_x, ball_y, ball_radius, command, ball_detected,
                frame_width, frame_height,distance, using_circle_detection ,False)
            
            
           
            # Afficher les résultats de détection
            if command:
                print(f"Commande envoyée: {command}")
            
            
            # Si on utilise l'ESP32-CAM et qu'une balle est détectée,
            # envoyer sa position à l'ESP32
            if not USE_WEBCAM and ball_detected and ball_x is not None and ball_y is not None:
                send_ball_position(ball_x, ball_y)
                
            masked_bitwise = cv2.bitwise_and(frame, frame, mask=mask)
            if DISPLAY_WITH_IMSHOW:
                CV_Imshow_Display(result_frame,mask,masked_bitwise)
            if DISPLAY_WITH_APPUI:
                App_Display(result_frame,blurred_mask,masked_bitwise)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("\nProgramme arrêté par l'utilisateur")
    
    except Exception as e:
        print(f"Erreur inattendue: {e}")
    
    finally:
        # Nettoyer les ressources
        release_resources(cap)
        print("Programme terminé.")

# Point d'entrée du programme
if __name__ == "__main__":
    main()