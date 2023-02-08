import cv2
import numpy as np

# Load the Limelight camera parameters
# Will have to adjust these two values for lemlight
focal_length = 998.2  # focal length of the camera in pixels
principal_point = (640/2, 480/2)  # principal point of the camera in pixels

text = "CONE&CUBE"
font = cv2.FONT_HERSHEY_SIMPLEX
org = (50, 50)
fontScale = 1
color = (0, 255, 0)
thickness = 2

def runPipeline():
    # Open the camera
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range of yellow color in HSV
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        # Define the range of purple color in HSV
        purple_lower = np.array([140, 100, 100])
        purple_upper = np.array([160, 255, 255])

        # Threshold the HSV image to get only the yellow or purple colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        purple_mask = cv2.inRange(hsv, purple_lower, purple_upper)
        mask = cv2.bitwise_or(yellow_mask, purple_mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour
        largest_area = 0
        largest_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                largest_contour = contour

        if largest_contour is not None:
            # Draw the largest contour
            cv2.drawContours(frame, [largest_contour], 0, (0, 0, 255), 2)

            # Calculate the distance and position of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                position = np.array([cx, cy])

                # Print the position of the largest contour
                print("Position:", position)

        cv2.putText(frame, text, org, font, color, thickness)
        # Display the frame with the largest contour
        cv2.imshow("Largest Contour Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

# Call the pipeline function - Don't need this line when we add it to the lemlight
runPipeline()