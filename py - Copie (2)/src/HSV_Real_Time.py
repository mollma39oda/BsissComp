import cv2
import numpy as np

# Valeurs HSV initiales (ex: vert-jaune)
hmin, smin, vmin = 25, 50, 50
hmax, smax, vmax = 45, 255, 255

# Options d'affichage
SHOW_MASK = True
SHOW_CONTOURS = True
SHOW_EXTRACTED_REGION = True
MIN_CONTOUR_AREA = 200

def setup_trackbars():
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("Hue Min", "Trackbars", hmin, 179, lambda x: None)
    cv2.createTrackbar("Hue Max", "Trackbars", hmax, 179, lambda x: None)
    cv2.createTrackbar("Sat Min", "Trackbars", smin, 255, lambda x: None)
    cv2.createTrackbar("Sat Max", "Trackbars", smax, 255, lambda x: None)
    cv2.createTrackbar("Val Min", "Trackbars", vmin, 255, lambda x: None)
    cv2.createTrackbar("Val Max", "Trackbars", vmax, 255, lambda x: None)

def get_trackbar_values():
    hmin = cv2.getTrackbarPos("Hue Min", "Trackbars")
    hmax = cv2.getTrackbarPos("Hue Max", "Trackbars")
    smin = cv2.getTrackbarPos("Sat Min", "Trackbars")
    smax = cv2.getTrackbarPos("Sat Max", "Trackbars")
    vmin = cv2.getTrackbarPos("Val Min", "Trackbars")
    vmax = cv2.getTrackbarPos("Val Max", "Trackbars")
    return hmin, smin, vmin, hmax, smax, vmax

def apply_hsv_threshold(img, hsv, mask):
    hmin, smin, vmin, hmax, smax, vmax = get_trackbar_values()
    lower = np.array([hmin, smin, vmin])
    upper = np.array([hmax, smax, vmax])
    mask[:] = cv2.inRange(hsv, lower, upper)
    return mask

def find_and_draw_contours(img, mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > MIN_CONTOUR_AREA:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

def main():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Erreur : Impossible d'ouvrir la caméra.")
        return

    ret, frame = cap.read()
    if not ret:
        print("Erreur : Impossible de lire une image.")
        return

    hsv = np.zeros_like(frame)
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

    setup_trackbars()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = apply_hsv_threshold(frame, hsv, mask)

        if SHOW_CONTOURS:
            find_and_draw_contours(frame, mask)

        if SHOW_MASK:
            cv2.imshow("Masque HSV", mask)

        if SHOW_EXTRACTED_REGION:
            extracted = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.imshow("Région extraite", extracted)

        cv2.imshow("Image originale", frame)

        hmin, smin, vmin, hmax, smax, vmax = get_trackbar_values()
        print(f"HSV Min: {hmin},{smin},{vmin} Max: {hmax},{smax},{vmax}")

        if cv2.waitKey(30) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
