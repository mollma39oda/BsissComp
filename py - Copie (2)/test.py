import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_circles_high_precision(image_path, display_result=True):
    """
    Detects circles in an image with high precision using OpenCV.
    
    Args:
        image_path (str): Path to the input image
        display_result (bool): Whether to display the result
        
    Returns:
        circles: Detected circles as numpy array with [x, y, radius] format
        result_image: Image with detected circles drawn on it
    """
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    
    # Create a copy for drawing results
    result_image = image.copy()
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply adaptive histogram equalization for better contrast
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(blurred)
    
    # Use binary adaptive threshold to further enhance edges
    _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Parameters for Hough Circle Transform
    # 1. dp: Inverse ratio of accumulator resolution to image resolution
    # 2. minDist: Minimum distance between circle centers
    # 3. param1: Higher threshold for Canny edge detector
    # 4. param2: Accumulator threshold (lower = more circles detected)
    # 5. minRadius: Minimum circle radius
    # 6. maxRadius: Maximum circle radius
    
    # Use HoughCircles with carefully tuned parameters
    circles = cv2.HoughCircles(
        enhanced,
        cv2.HOUGH_GRADIENT_ALT,  # Use HOUGH_GRADIENT_ALT for better precision
        dp=1.5,
        minDist=20,
        param1=300,
        param2=0.85,  # Lower threshold for higher sensitivity
        minRadius=10,
        maxRadius=100
    )
    
    # If circles are detected
    if circles is not None:
        # Convert to int values
        circles = np.uint16(np.around(circles))
        
        # Draw detected circles
        for i in circles[0, :]:
            # Draw the outer circle
            cv2.circle(result_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(result_image, (i[0], i[1]), 2, (0, 0, 255), 3)
            
        print(f"Detected {len(circles[0])} circles")
    else:
        print("No circles detected")
    
    if display_result:
        # Display images using matplotlib
        plt.figure(figsize=(12, 6))
        
        plt.subplot(131)
        plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        plt.title('Original Image')
        plt.axis('off')
        
        plt.subplot(132)
        plt.imshow(enhanced, cmap='gray')
        plt.title('Enhanced Image')
        plt.axis('off')
        
        plt.subplot(133)
        plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
        plt.title('Detected Circles')
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
    
    return circles, result_image

def fine_tune_parameters(image_path):
    """
    Provides a way to fine-tune the Hough Circle Transform parameters
    for the specific image to achieve higher precision.
    
    Args:
        image_path (str): Path to the input image
    """
    # Load image
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply adaptive histogram equalization
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(blurred)
    
    # Parameters to test
    dp_values = [1.2, 1.5, 1.8]
    param1_values = [200, 300, 400]
    param2_values = [0.75, 0.85, 0.95]
    
    best_result = None
    best_params = None
    max_circles = 0
    
    # Find the best parameters
    for dp in dp_values:
        for param1 in param1_values:
            for param2 in param2_values:
                circles = cv2.HoughCircles(
                    enhanced,
                    cv2.HOUGH_GRADIENT_ALT,
                    dp=dp,
                    minDist=20,
                    param1=param1,
                    param2=param2,
                    minRadius=10,
                    maxRadius=100
                )
                
                if circles is not None and len(circles[0]) > max_circles:
                    max_circles = len(circles[0])
                    best_result = circles
                    best_params = (dp, param1, param2)
    
    if best_params:
        print(f"Best parameters: dp={best_params[0]}, param1={best_params[1]}, param2={best_params[2]}")
        print(f"Detected {max_circles} circles")
        
        # Draw circles with best parameters
        result_image = image.copy()
        if best_result is not None:
            circles = np.uint16(np.around(best_result))
            for i in circles[0, :]:
                cv2.circle(result_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(result_image, (i[0], i[1]), 2, (0, 0, 255), 3)
            
            plt.figure(figsize=(12, 6))
            plt.subplot(121)
            plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            plt.title('Original Image')
            plt.axis('off')
            
            plt.subplot(122)
            plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
            plt.title('Detected Circles (Best Parameters)')
            plt.axis('off')
            
            plt.tight_layout()
            plt.show()
    else:
        print("No circles detected with any parameter combination")

# Example usage
if __name__ == "__main__":
    # Replace with your image path
    image_path = "circles.jpg"
    
    # Detect circles with high precision
    circles, result = detect_circles_high_precision(image_path)
    
    # To fine-tune parameters for your specific image:
    # fine_tune_parameters(image_path)