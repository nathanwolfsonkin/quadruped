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

    # Get the width and height of the video frames
    frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print('width: ', frame_width)
    print('height: ', frame_height)

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
    
    flag = True

    # Loop through the video frames
    while True:
        ret, frame = video.read()
        
        if not ret:
            break
        
        # Update the tracker
        success, bbox = tracker.update(frame)

        # Handle key events
        if flag == False:
            while True:
                key = cv2.waitKey(0) & 0xFF  # Wait indefinitely for a key press
                breaker = False
                if key == ord(' '):  # Spacebar pressed
                    break
                elif key == ord('r'):
                        bbox = cv2.selectROI("Tracking", frame, False)  # Re-select bounding box
                        tracker = cv2.TrackerCSRT_create()  # Reinitialize the tracker
                        tracker.init(frame, bbox)  # Initialize the tracker with the new bounding box
                        continue  
                if key == ord('q'):  # Exit if 'q' is pressed
                    breaker = True
                    break
            if breaker:
                break

        flag = False

        if success:
            # Draw bounding box around the tracked object
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
            
            # Save the center point of the bounding box
            center_x = int(bbox[0] + bbox[2] / 2)
            center_y = int(bbox[1] + bbox[3] / 2)
            positions.append((center_x, center_y))
        else: 
            cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        # Display the frame
        cv2.imshow("Tracking", frame)

    # Release video and close windows
    video.release()
    cv2.destroyAllWindows()

    # Convert the tracked positions into a NumPy array for further analysis
    positions = np.array(positions)

    # Save the position data to a CSV file for later analysis
    np.savetxt(csv_path, positions, delimiter=",", header="x,y", comments='')

def main():
    video = 'unitree_a1/unitree_a1_gait.mp4'
    csv = 'unitree_a1/test.csv'
    video_to_csv(video, csv)

if __name__ == "__main__":
    main()