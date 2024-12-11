import cv2
import face_recognition

# Load the reference image and compute its face encoding
reference_image_path = "images\\ben.jpg"  # Replace with the path to your reference image
reference_image = face_recognition.load_image_file(reference_image_path)
reference_encoding = face_recognition.face_encodings(reference_image)[0]

# Initialize video capture from the video file
video_path = "images\\ben_vid1.mp4"  # Replace with your video file path
video_capture = cv2.VideoCapture(video_path)

# Skip frames to process only every nth frame
frame_skip = 2  # Adjust based on performance needs
frame_count = 0

while True:
    # Capture a single frame from the video
    ret, frame = video_capture.read()
    if not ret:
        print("End of video or failed to grab frame. Exiting...")
        break

    frame_count += 1
    if frame_count % frame_skip != 0:
        continue  # Skip processing for this frame

    # Resize frame to speed up processing (adjust scale for balance)
    small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    # Detect all faces in the resized frame and their encodings
    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    for face_encoding, face_location in zip(face_encodings, face_locations):
        # Compare each detected face with the reference face
        matches = face_recognition.compare_faces([reference_encoding], face_encoding, tolerance=0.6)

        if matches[0]:  # If a match is found
            # Scale up face location to match the original frame size
            top, right, bottom, left = face_location
            top, right, bottom, left = top * 2, right * 2, bottom * 2, left * 2

            # Draw a rectangle around the matched face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

    # Display the video frame
    cv2.imshow("Video Face Tracking", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
video_capture.release()
cv2.destroyAllWindows()
