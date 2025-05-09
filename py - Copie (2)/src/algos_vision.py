import cv2 as cv
import numpy as np
from config import (KNOWN_DISTANCE, KNOWN_WIDTH)
from ball_tracking import detect_ball

# Focal Length calculation function
def FocalLength(measured_distance, real_width, width_in_rf_image):
    """
    This function calculates the Focal Length (distance between lens and CMOS sensor).
    It is a simple constant we can find by using MEASURED_DISTANCE, REAL_WIDTH (actual width of the object),
    and WIDTH_OF_OBJECT_IN_IMAGE.
    
    :param measured_distance (int or float): Distance from the object to the camera while capturing the reference image.
    :param real_width (int or float): Actual width of the object in the real world (e.g., My face width = 5.7 inches).
    :param width_in_rf_image (int or float): The object's width in the reference image (measured by the face detector).
    
    :return: Focal length (float).
    """
    # Vérification des types
    assert isinstance(measured_distance, (int, float)), "Measured Distance doit être un int ou float"
    assert isinstance(real_width, (int, float)), "Real Width doit être un int ou float"
    assert isinstance(width_in_rf_image, (int, float)), "Width in Reference Image doit être un int ou float"

    # Calcul de la longueur focale
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length

# Distance estimation function
def Distance_finder(Focal_Length, real_face_width, ball_width_in_frame):
    """
    This function estimates the distance between the object and the camera using the given parameters:
    Focal Length, Real Object Width, and Object Width in the image.
    
    :param Focal_Length (float): The focal length calculated by the FocalLength function.
    :param real_face_width (int or float): Actual width of the face in the real world.
    :param ball_width_in_frame (int or float): The width of the face in the image frame (detected face).
    
    :return: Estimated distance (float).
    """
 
    
    # Calcul de la distance
    distance = (real_face_width * Focal_Length) / ball_width_in_frame
    return distance

# Main function to process the frame and calculate distance
def distanceMain(blurred_mask, frame, frame_width, frame_height,Focal_length_found):
    """
    Main function to calculate the distance of an object (ball) in the frame using the reference image.

    :param blurred_mask (ndarray): Mask image to help in detecting objects.
    :param frame (ndarray): The video frame from the camera feed.
    :param frame_width (int): The width of the frame.
    :param frame_height (int): The height of the frame.

    :return: Estimated distance (float) of the object (ball) from the camera.
    """
   
    
   
 
    
    # Détection du ballon dans l'image actuelle
    _, _, ball_radius, _, ball_detected = detect_ball(blurred_mask, frame, frame_width, frame_height)
    if not ball_detected:
        print("Ballon non détecté dans l'image.")
        return None
    
    # Calcul de la largeur du ballon dans le cadre actuel
    ball_width_in_frame = 2 * ball_radius
    
    # Estimation de la distance en utilisant la longueur focale et la largeur du ballon dans le cadre
    Distance = Distance_finder(Focal_length_found, KNOWN_WIDTH, ball_width_in_frame)
    print(f"Distance estimée: {Distance} pouces")
    
    # Retour de la distance estimée
    return Distance

def Distance_Init(blurred_mask,):
     # Lecture de l'image de référence
    ref_image = cv.imread("Ref_image.jpg")
    if ref_image is None:
        print("Erreur: L'image de référence n'a pas pu être chargée.")
        return None
    
    # Détection du ballon en utilisant le module ball_tracking
    _, _, ball_radius, _, ball_detected = detect_ball(blurred_mask, ref_image, 100, 100)

    if not ball_detected:
        print("Ballon non détecté dans l'image.")
        return None
    
    # Calcul de la longueur focale en utilisant l'image de référence
    ref_image_ball_width = 2 * ball_radius  # Largeur du ballon dans l'image de référence
    Focal_length_found = FocalLength(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_ball_width)
    return Focal_length_found