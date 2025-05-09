import cv2
import numpy as np

# Capture vidéo
cap = cv2.VideoCapture(1)

# Structuring element
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# Soustracteur de fond
fgbg = cv2.createBackgroundSubtractorMOG2(history=0, varThreshold=100, detectShadows=False)

# Boucle principale
while True:
    ret, frame1 = cap.read()
    if not ret:
        break

    # Redimensionnement
    frame = cv2.resize(frame1, (1364, 700), interpolation=cv2.INTER_CUBIC)

    # Perspective
    width, height = 682, 350
    pts1 = np.float32([[426, 90], [940, 90], [11, 652], [1353, 652]])
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    warpOutput = cv2.warpPerspective(frame, matrix, (width, height))

    # Masquage ROI
    mask = np.ones_like(warpOutput, dtype=np.uint8) * 255
    roi_corners = np.array([[(0, 340), (682, 340), (682, 20), (0, 20)]], dtype=np.int32)
    cv2.fillPoly(mask, roi_corners, 0)
    masking = cv2.bitwise_or(warpOutput, mask)

    # Soustraction de fond et traitement
    fgmask = fgbg.apply(masking)
    fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Parcourir les contours
    for contour in contours:
        if cv2.contourArea(contour) > 5:
            x, y, w, h = cv2.boundingRect(contour)
            player_img = warpOutput[y:y + h, x:x + w]
            player_hsv = cv2.cvtColor(player_img, cv2.COLOR_BGR2HSV)

            # Définition des plages HSV
            color_ranges = {
                'Purple': ([150, 150, 100], [184, 255, 255], (0, 255, 255)),
                'Yellow': ([20, 150, 100], [40, 255, 255], (0, 255, 0)),
                'Green':  ([50, 150, 100], [90, 255, 255], (255, 255, 0)),
                'Orange': ([5, 150, 100], [20, 255, 255], (255, 0, 0))
            }

            detected = False
            for name, (lower, upper, color) in color_ranges.items():
                mask = cv2.inRange(player_hsv, np.array(lower), np.array(upper))
                count = cv2.countNonZero(mask)
                if count > 5:
                    cv2.rectangle(warpOutput, (x, y), (x + w, y + h), color, 2)
                    detected = True
                    break

            if not detected:
                cv2.rectangle(warpOutput, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # Affichage
    cv2.imshow("Original", frame)
    cv2.imshow("Perspective", warpOutput)

    # Sortie avec 'q'
    if cv2.waitKey(10) & 0xFF == ord('q'):
        print("Video Stopped")
        break

# Nettoyage
cap.release()
cv2.destroyAllWindows()
