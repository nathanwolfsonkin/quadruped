import cv2
import numpy as np

def video_to_csv(video_local_path, csv_local_path):
    # Update video path
    video_path = 'src/energy_model/kinematics_data/' + video_local_path
    csv_path = 'src/energy_model/kinematics_data/' + csv_local_path

    # Load video
    video = cv2.VideoCapture(video_path)

    # Read the first frame of the video
    ret, frame = video.read()
    first_frame = True

    if frame is None:
        print("Error: Frame is None. Check the video source.")
        exit()

    # Define an initial bounding box around the leg
    bbox = cv2.selectROI("Tracking", frame, False)
    cv2.destroyAllWindows()

    # Initialize the tracker
    tracker = cv2.TrackerCSRT_create()
    tracker.init(frame, bbox)

    # To store the kinematic data (position over time)
    positions = []

    # CODE SHOULD EXECUTE IN THE FOLLOWING ORDER
    # 1) READ NEXT FRAME
    # 2) UPDATE TRACKER
    # 3) SHOW CURRENT TRACKER BOUNDING BOX
    # 4) WAIT FOR USER INPUT (CONFIRM SELCTION, REDRAW THE BOUNDING BOX, OR QUIT)
    # 5) RECORD THE RESULTS

    # Loop through video frames provided that a frame has been found
    while ret == True:

        # Read new frame only if not first itteration
        if first_frame == True:
            first_frame = False
        else:
            ret, frame = video.read()
            ret, frame = video.read()

        if not ret:
            break

        # Update the tracker
        success, bbox = tracker.update(frame)

        # Update bounding box
        if success:
            # Draw bounding box around the tracked object
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        else: 
            cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        
        # Display the frame
        cv2.imshow("Tracking", frame)

        key = cv2.waitKey(0) & 0xFF  # Wait indefinitely for a key press
        if key == ord(' '):  # Spacebar pressed
            pass
        elif key == ord('r'):
            bbox = cv2.selectROI("Tracking", frame, False)  # Re-select bounding box
            tracker = cv2.TrackerCSRT_create()  # Reinitialize the tracker
            tracker.init(frame, bbox)  # Initialize the tracker with the new bounding box
            success, bbox = tracker.update(frame)
            cv2.imshow("Tracking", frame)
        elif key == ord('q'):  # Exit if 'q' is pressed
            # Set ret to exit the while loop
            ret = False

        # Record 
        if success:            
            # Save the center point of the bounding box
            center_x = int(bbox[0] + bbox[2] / 2)
            center_y = int(bbox[1] + bbox[3] / 2)
            positions.append((center_x, center_y))

    # Release video and close windows
    video.release()
    cv2.destroyAllWindows()

    # Convert the tracked positions into a NumPy array for further analysis
    positions = np.array(positions)

    # Save the position data to a CSV file for later analysis
    np.savetxt(csv_path, positions, delimiter=",", header="x,y", comments='')

def main():
    video = 'unitree_a1/unitree_a1_gait.mp4'
    csv = 'unitree_a1/rear_foot.csv'
    video_to_csv(video, csv)
 
if __name__ == "__main__":
    main()
