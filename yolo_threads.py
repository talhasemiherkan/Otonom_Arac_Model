# yolo_threads.py
import time
import threading
from queue import Empty
from ultralytics import YOLO

# ================== BASE THREAD ==================
class BaseYoloThread(threading.Thread):
    def __init__(self, frame_q, shared_state, lock,
                 model_path, fps=6, imgsz=416, conf=0.5, classes=None, persistence=0.5, min_box_area=0):
        super().__init__(daemon=True)
        self.frame_q = frame_q
        self.shared_state = shared_state
        self.lock = lock
        self.model = YOLO(model_path)
        self.sleep = 1.0 / fps
        self.imgsz = imgsz
        self.conf = conf
        self.classes = classes
        self.min_box_area = min_box_area
        
        # Persistence (Kalicilik)
        self.persistence = persistence
        self.last_seen_time = 0.0
        self.last_valid_boxes = []

    def get_frame(self):
        try:
            return self.frame_q.get(timeout=0.5)
        except Empty:
            return None


# ================== PERSON ==================
class PersonThread(BaseYoloThread):
    def run(self):
        while True:
            frame = self.get_frame()
            if frame is None:
                time.sleep(self.sleep)
                continue

            # ROI ISLEMI KALDIRILDI: Tum goruntu islenecek
            h, w = frame.shape[:2]
            # Eskiden ustten %30 kesiliyordu, simdi 0 (Kesme yok)
            crop_h = 0 
            roi_frame = frame

            r = self.model(
                roi_frame,
                imgsz=self.imgsz,
                conf=self.conf,
                classes=self.classes,
                verbose=False
            )[0]

            boxes_out = []
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                
                # Koordinatlari orjinal resme gore duzelt (offset ekle)
                y1 += crop_h
                y2 += crop_h
                
                conf = float(b.conf[0])
                
                # Confidence kontrolunu parametreye bagla
                if conf < self.conf:
                    continue
                
                # Boyut filtresi (Kucuk nesneleri at)
                area = (x2 - x1) * (y2 - y1)
                if area < self.min_box_area:
                    continue

                boxes_out.append((x1, y1, x2, y2, conf))

            with self.lock:
                now = time.time()
                if boxes_out:
                    # Nesne goruldu, zamani ve kutuyu guncelle
                    self.last_seen_time = now
                    self.last_valid_boxes = boxes_out[:1]
                    self.shared_state["person"] = True
                    self.shared_state["person_boxes"] = boxes_out[:1]
                else:
                    # Nesne gorulmedi, persistence suresini kontrol et
                    if (now - self.last_seen_time) < self.persistence:
                        # Sure dolmadi, hala VAR kabul et (Eski kutularla)
                        self.shared_state["person"] = True
                        self.shared_state["person_boxes"] = self.last_valid_boxes
                    else:
                        # Sure doldu, gercekten YOK
                        self.shared_state["person"] = False
                        self.shared_state["person_boxes"] = []

            time.sleep(self.sleep)


# ================== STOP ==================
class StopThread(BaseYoloThread):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.consecutive_frames = 0
        self.required_frames = 1

    def run(self):
        while True:
            frame = self.get_frame()
            if frame is None:
                time.sleep(self.sleep)
                continue

            r = self.model(
                frame,
                imgsz=self.imgsz,
                conf=self.conf,
                classes=self.classes,
                verbose=False
            )[0]

            boxes_out = []
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                conf = float(b.conf[0])
                cls_id = int(b.cls[0])
                cls_name = r.names[cls_id]
                
                # DEBUG: Ne gorudugunu yazdir
                # print(f"DEBUG: StopThread saw {cls_name} ({conf:.2f})")
                
                if conf < self.conf:
                    continue
                
                # Stop levhasi icin de kucuk parazitleri atalim
                area = (x2 - x1) * (y2 - y1)
                if area < self.min_box_area:
                    continue
                
                # Eger model COCO ise ve 'stop sign' (ID 11) degilse atla
                # Eger custom tek sinif ise bu kontrolu yapma veya ID 0 kontrolu yap
                print(f"DEBUG: StopThread DETECTED -> {cls_name} ID:{cls_id} Conf:{conf:.2f}")

                # FILTRE: Isminde "stop" gecmeyen seyleri gormezden gel
                # Ornek: "green_sign" gelirse buraya takilir
                if "stop" not in cls_name.lower():
                     # print(f"DEBUG: IGNORED {cls_name} (Not a stop sign)")
                     continue

                boxes_out.append((x1, y1, x2, y2, conf))

            with self.lock:
                now = time.time()
                if boxes_out:
                    self.consecutive_frames += 1
                    
                    # Yalnizca N kare boyunca gorulduyse kabul et
                    if self.consecutive_frames >= self.required_frames:
                        self.last_seen_time = now
                        self.last_valid_boxes = boxes_out[:1]
                        self.shared_state["stop"] = True
                        self.shared_state["stop_boxes"] = boxes_out[:1]
                        # Sayaci sinirla (overflow olmasin)
                        if self.consecutive_frames > 100:
                            self.consecutive_frames = self.required_frames
                else:
                    # Algilama yoksa sayaci sifirla
                    self.consecutive_frames = 0
                    
                    if (now - self.last_seen_time) < self.persistence:
                        self.shared_state["stop"] = True
                        self.shared_state["stop_boxes"] = self.last_valid_boxes
                    else:
                        self.shared_state["stop"] = False
                        self.shared_state["stop_boxes"] = []

            time.sleep(self.sleep)


# ================== TRAFFIC LIGHT ==================
class LightThread(BaseYoloThread):
    def __init__(self, *args, red_id, green_id, **kwargs):
        super().__init__(*args, **kwargs)
        self.red_id = red_id
        self.green_id = green_id
        # Isiklar icin ayri persistence
        self.last_seen_red = 0.0
        self.last_seen_green = 0.0
        self.last_red_boxes = []
        self.last_green_boxes = []

    def run(self):
        while True:
            frame = self.get_frame()
            if frame is None:
                time.sleep(self.sleep)
                continue

            r = self.model(
                frame,
                imgsz=self.imgsz,
                conf=self.conf,
                verbose=False
            )[0]

            red_boxes = []
            green_boxes = []

            for b in r.boxes:
                cid = int(b.cls[0])
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                conf = float(b.conf[0])
                
                if conf < self.conf:
                    continue
                
                # Isiklarda da boyut kontrolu (cok uzaktaki isiklari gorme)
                area = (x2 - x1) * (y2 - y1)
                if area < self.min_box_area:
                    continue

                if cid == self.red_id:
                    red_boxes.append((x1, y1, x2, y2, conf))
                elif cid == self.green_id:
                    green_boxes.append((x1, y1, x2, y2, conf))

            with self.lock:
                now = time.time()
                
                # RED LOGIC
                if red_boxes:
                    self.last_seen_red = now
                    self.last_red_boxes = red_boxes[:1]
                    self.shared_state["red"] = True
                    self.shared_state["red_boxes"] = red_boxes[:1]
                elif (now - self.last_seen_red) < 0.3: # Isik icin 0.3s persistence
                    self.shared_state["red"] = True
                    self.shared_state["red_boxes"] = self.last_red_boxes
                else:
                    self.shared_state["red"] = False
                    self.shared_state["red_boxes"] = []

                # GREEN LOGIC
                if green_boxes:
                    self.last_seen_green = now
                    self.last_green_boxes = green_boxes[:1]
                    self.shared_state["green"] = True
                    self.shared_state["green_boxes"] = green_boxes[:1]
                elif (now - self.last_seen_green) < 0.3:
                    self.shared_state["green"] = True
                    self.shared_state["green_boxes"] = self.last_green_boxes
                else:
                    self.shared_state["green"] = False
                    self.shared_state["green_boxes"] = []

            time.sleep(self.sleep)
