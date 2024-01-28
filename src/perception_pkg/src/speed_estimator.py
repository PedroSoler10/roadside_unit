from ultralytics import YOLO
from ultralytics.solutions import speed_estimation
import cv2
import imutils

model = YOLO("yolov8n.pt")
names = model.model.names
print(names)

video_path = "/home/pedro/Videos/person_walking.mp4"
cap = cv2.VideoCapture("/home/pedro/Videos/person_walking.mp4")
# print(video_path)
assert cap.isOpened(), "Error reading video file"
w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

# Video writer
# video_writer = cv2.VideoWriter("speed_estimation.avi",
#                                cv2.VideoWriter_fourcc(*'mp4v'),
#                                fps,
#                                (w, h))
# print(w)
# print(h)
# line_pts = [(0 + 10, 0 + 10), (h - 10, w - 10)]
# line_pts = [(20, 400), (1260, 400)]
line_pts = [(400, 20), (400, 1260)]

# Init speed-estimation obj
speed_obj = speed_estimation.SpeedEstimator()
speed_obj.set_args(reg_pts=line_pts,
                   names=names,
                   view_img=True)

while cap.isOpened():

    success, im0 = cap.read()
    if not success:
        print("Video frame is empty or video processing has been successfully completed.")
        break

    im0 = imutils.rotate_bound(im0, 90)  # Rotate 90 degrees
    tracks = model.track(im0, persist=True, show=False)

    im0 = speed_obj.estimate_speed(im0, tracks)
    # video_writer.write(im0)

cap.release()
# video_writer.release()
cv2.destroyAllWindows()