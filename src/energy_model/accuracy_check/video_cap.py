import cv2
import os

def resize_image(img, scale_factor):

    # Resize the image to the specified scale factor
    width = int(img.shape[1] * scale_factor)
    height = int(img.shape[0] * scale_factor)
    resized_img = cv2.resize(img, (width, height))

    # Convert from BGR (OpenCV format) to RGB (Matplotlib format)
    resized_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB)
    return resized_img

# Path to save frames for Matplotlib script
frame_folder = "/workspace/src/energy_model/accuracy_check/video_frames"

video_path = "/workspace/src/energy_model/kinematics_data/unitree_a1/unitree_a1_gait.mp4"  # Path to your input video
cap = cv2.VideoCapture(video_path)

frame_count = 0

while True:
    if frame_count == 0:
        ret, frame = cap.read()
    else:
        ret, frame = cap.read()
        ret, frame = cap.read()

    if not ret:
        break

    frame = resize_image(frame, .75)

    # Save each frame as an image for Matplotlib script
    cv2.imwrite(os.path.join(frame_folder, f"frame_{frame_count:04d}.png"), frame)
    frame_count += 1

cap.release()
print("Video processing complete. Frames saved.")
