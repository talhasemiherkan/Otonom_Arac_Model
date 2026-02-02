from ultralytics import YOLO

def check_model(path):
    print(f"\n--- Checking {path} ---")
    try:
        model = YOLO(path)
        print(f"Classes: {model.names}")
    except Exception as e:
        print(f"Error loading {path}: {e}")

if __name__ == "__main__":
    base_path = "/home/pi/Desktop/pi2/models/"
    models_to_check = ["model.pt", "stop.pt", "yolov8n.pt"]
    
    for m in models_to_check:
        check_model(base_path + m)

