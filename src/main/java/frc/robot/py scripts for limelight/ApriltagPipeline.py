"""
This is 5411's custom pipeline for limelight v3

This script is to detect apriltags, find its distance, and position relative to the camera

runPipeline() function is to let limelight know that this is a pipeline

In the function:
    Set global variables to be used outside the function
    Convert video to grayscale with cv2.cvtColor
    Use Detector from the pupil_apriltags library to to define the apriltag detection method
    We use .detect to actually detect the apriltag in the 
    
    In the if loop:
        Check for the biggest apriltag within the frame,
        We do this by checking which apriltag has the biggest area
        Then calculate distance of the apriltag from the camera
        We then calculate the position the apriltag relative to the camera frame and set position in a numpy array
        We then return our variables to be used outside the function
        However, if the variables are empty, return None (null)
        
cv2.VideoCapture(0) is exactly what it sounds like, capture video input

While loop to check every single frame

ret, frame = cap.read() -> This is an interesting line, frame is each frame of the video, .read() reads each frame and treats it as an image
ret is a dummy variable, it does essentially nothing, its only used to make sure that line is actually working, but it won't work without it

cv2.putText writes text directly over the output video, in this case, it displays three items: tag id, distance, and position

cv2.imshow() returns the video output, this line is probably not needed for the limelight pipeline, it displays the output
in a separate window, the first string is the name of that window, the second part is the video itself

if loop waits for the key press of q to quit the process, again this loop is also not needed for the limelight pipeline

cap.release() stops using the camera and allows the camera to be used by other applications
this is why you can only use the camera in only one application at a time
Also not needed for the limelight pipeline

cv2.destroyAllWindows() destroys the output windows we made earlier to display the output, this line is also not needed for the limelight pipeline

"""


import cv2
import numpy as np
from pupil_apriltags import Detector

def runPipeline(frame):
    global x
    global y
    global tag_id
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Define the AprilTag detector with the 16h5 family
    detector = Detector(families='16h5', nthreads=1, quad_decimate=1.0,
                        quad_sigma=0.0, refine_edges=True, decode=True,
                        Debug=False)

    # Detect AprilTags in the frame
    tags = detector.detect(frame, gray, return_image=False, camera_params=None)

    if len(tags) > 0:
        # Find the tag with the largest area
        tag = max(tags, key=lambda x: x.area)
        center = tag.center
        tag_id = tags.tag_id
        x, y = center[0], center[1]

        # Calculate the distance using the size of the tag in the frame
        focal_length = 100
        tag_size = 0.1  # meters
        pixel_width = tag.corners[2][0] - tag.corners[0][0]
        distance = (tag_size * focal_length) / pixel_width

        # Calculate the position using the Limelight pipeline
        x_image_center = frame.shape[1] / 2
        y_image_center = frame.shape[0] / 2
        x_offset = x - x_image_center
        y_offset = y - y_image_center
        position = np.array([x_offset, y_offset, distance])


        return distance, position, tag_id, x, y
    else:
        return None, None


# Capture video from a camera
cap = cv2.VideoCapture(0)

# Loop over the frames
while True:
    # Capture a frame
    ret, frame = cap.read()

    # Run the Limelight pipeline on the frame
    distance, position = runPipeline(frame)

    # Display the information of the AprilTag
    # I don't understand why x,y shows as undefined - Have to find solution later
    # Possibly fixed?
    if distance is not None and position is not None:
        cv2.putText(frame, f"ID: {tag_id}", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Distance: {distance:.2f} m", (x,y + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})", (x, y + 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Display the frame
    cv2.imshow("Frame", frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture
cap.release()

# Destroy all windows
cv2.destroyAllWindows()
