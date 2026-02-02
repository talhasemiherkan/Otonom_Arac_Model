from ultralytics import YOLO
import shutil
import os

# Download standard yolov8n model (it will download to current dir if not found)
model = YOLO("yolov8n.pt") 

# Move to correct location
src = "yolov8n.pt"
dst = "/home/pi/Desktop/pi2/models/yolov8n.pt"

if os.path.exists(src):
    print(f"Downloaded {src}, moving to {dst}")
    shutil.move(src, dst)
else:
    print("Download failed or file not found in CWD")
