#!/usr/bin/env python3

# -*- coding: utf-8 -*-



import time

import cv2

import threading

import numpy as np

from queue import Queue, Empty

from flask import Flask, Response, render_template

import RPi.GPIO as GPIO

from picamera2 import Picamera2



from lane_detector import LaneDetector

from vehicle import Vehicle

from servo_motor import Servo_Motor

from yolo_threads import LightThread, PersonThread, StopThread



# ================== AYARLAR ==================

HTTP_HOST = "0.0.0.0"

HTTP_PORT = 5000

STREAM_W, STREAM_H = 480, 360

JPEG_QUALITY = 70



# ================== DÄ°REKSÄ°YON AYARLARI ==================

MAX_DELTA_PX   = 120

MAX_STEER_DEG  = 25.0     # 30 -> 25 (SG90 iyice zorlanmasin)

STEER_CENTER   = 90.0

STEER_OFFSET   = -8.0     # Hep saga gittigi icin, sola (-8) offset verelim

DEADBAND_PX    = 5        # 10 -> 5 (Daha hassas)

LPF_ALPHA      = 0.7      # 0.3 -> 0.7 (Daha hizli tepki)

MAX_SERVO_STEP = 15.0     # 50 -> 15 (More smoothing)



app = Flask(__name__)

stop_event = threading.Event()



frame_q = Queue(maxsize=1)

stream_q = Queue(maxsize=1)



servo = None

vehicle = None

detector = None

picam2 = None



# ================== PAYLASIMLI DURUM (TRAFFIC LIGHT, PERSON & STOP) ==================

shared_state = {

    "red": False,

    "green": False,

    "person": False,

    "stop": False,

    "red_boxes": [],

    "green_boxes": [],

    "person_boxes": [],

    "stop_boxes": []

}

state_lock = threading.Lock()



# ================== HUD ==================

def draw_hud(frame, fps, speed, delta):

    lines = [

        f"FPS: {fps:.1f}",

        "MODE: LANE_ONLY",

        f"SPEED: {speed}",

        f"DELTA: {delta}"

    ]

    y = 40

    for t in lines:

        cv2.putText(frame, t, (10, y),

                    cv2.FONT_HERSHEY_SIMPLEX,

                    0.6, (0, 255, 255), 2)

        y += 25



# ================== DONANIM ==================

def init_hardware():

    global servo, vehicle

    servo = Servo_Motor(servo_pin=17)

    servo.center()

    vehicle = Vehicle()



# ================== KAMERA THREAD ==================

class CameraThread(threading.Thread):

    def __init__(self):

        super().__init__(daemon=True)



    def run(self):

        global picam2

        picam2 = Picamera2()

        cfg = picam2.create_video_configuration(

            main={"size": (640, 480), "format": "XRGB8888"}

        )

        picam2.configure(cfg)

        picam2.start()

        time.sleep(0.3)



        while not stop_event.is_set():

            frame = picam2.capture_array("main")

            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)



            if frame_q.full():

                frame_q.get_nowait()

            frame_q.put_nowait(frame)



# ================== PID KONTROL ==================

class PID:

    def __init__(self, Kp, Ki, Kd):

        self.Kp = Kp

        self.Ki = Ki-+

        self.Kd = Kd

        self.last_error = 0

        self.integral = 0

        self.last_time = time.time()



    def update(self, error):

        current_time = time.time()

        dt = current_time - self.last_time

        if dt <= 0: dt = 0.02

        self.last_time = current_time



        self.integral += error * dt

        derivative = (error - self.last_error) / dt

        self.last_error = error



        return (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)



# ================== DRIVE THREAD (PID) ==================

class DriveThread(threading.Thread):

    def __init__(self):

        super().__init__(daemon=True)

        self.last_time = time.time()

        self.fps = 0.0

        # PID AyarlarÄ±: Kp=0.12 (Daha agresif), Kd=0.01 (SÃ¶nÃ¼mleme)

        self.pid = PID(Kp=0.12, Ki=0.00, Kd=0.01)

        

        self.current_steer = STEER_CENTER

        self.stopped_by_light = False

        self.parked_forever = False # Stop.pt ile tamamen durma

        

        # Person Detection State

        self.person_wait_active = False

        self.person_wait_start = 0.0

        self.person_cooldown = 0.0



    def run(self):

        global detector

        detector = LaneDetector()



        while not stop_event.is_set():

            try:

                frame = frame_q.get(timeout=0.5)

            except Empty:

                continue



            # ===== TRAFIK ISIGI & INSAN & STOP KONTROLU =====

            with state_lock:

                is_red = shared_state.get("red", False)

                is_green = shared_state.get("green", False)

                is_person = shared_state.get("person", False)

                is_stop_sign = shared_state.get("stop", False)



            now = time.time()

            speed_status = "NORMAL" # VarsayÄ±lan



            # 0. YOL SONU / STOP CONTROL (En yuksek oncelik)

            if self.parked_forever:

                vehicle.stop()

                speed_status = "TASK COMPLETED"

                # Goruntuyu islemeye devam et ama motoru calistirma

                # HUD guncellemek icin asagi devam ediyoruz ama motor logic'de duracak

                pass 

            # Kirmizi isikta beklerken Stop levhasini dikkate ALMA (Cakismanin onlenmesi)

            # Ayrica: Eger o an Kirmizi isik algilaniyorsa (is_red), Stop levhasi logic'ine yine girme

            elif is_stop_sign and not self.stopped_by_light and not is_red:

                print("!!! STOP LEVHASI ALGILANDI - GOREV TAMAMLANDI - DURUYORUZ !!!")

                self.parked_forever = True

                vehicle.stop()

                speed_status = "STOP DETECTED"

            else:

                # 1. Isik Kontrolu

                if is_red:

                    if not self.stopped_by_light:

                        print("!!! KIRMIZI ISIK - DURUYOR !!!")

                        self.stopped_by_light = True

                

                if self.stopped_by_light:

                    if is_green:

                        print(">>> YESIL ISIK - DEVAM EDIYOR >>>")

                        self.stopped_by_light = False

                    else:

                        speed_status = "DUR (KIRMIZI)"

                

                # 2. Insan Kontrolu

                # Kirmizi isikta beklerken insan kontrolune GEREK YOK (Zaten duruyoruz)

                if not self.stopped_by_light:

                    if self.person_wait_active:

                        # Zaten insan gorduk bekliyoruz

                        elapsed = now - self.person_wait_start

                        remaining = 5.0 - elapsed

                        

                        if remaining <= 0:

                            # Sure doldu, kontrol et: Hala insan var mi?

                            if is_person:

                                print(f"!!! SURE DOLDU AMA YAYA HALA VAR - TEKRAR BEKLE !!!")

                                self.person_wait_start = now # Sayaci basa sar

                                speed_status = "YAYA (BEKLE)"

                            else:

                                print(">>> YAYA GITTI - DEVAM >>>")

                                self.person_wait_active = False

                                # Cooldown kaldirildi

                        else:

                            speed_status = f"YAYA ({remaining:.1f}s)"

                    

                    elif is_person:

                        # Yeni insan gorduk (Cooldown kontrolu yok)

                        print("!!! YAYA ALGILANDI - DURUYOR !!!")

                        self.person_wait_active = True

                        self.person_wait_start = now

                        speed_status = "YAYA (DUR)"





            # FPS Hesaplama

            dt = now - self.last_time

            self.last_time = now

            if dt > 0:

                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)



            # Åerit Tespiti

            annotated, decision, result_data = detector.process_frame(frame)

            

            if isinstance(result_data, tuple):

                delta_px, curve = result_data

            else:

                delta_px, curve = result_data, 0.0



            # ===== PID HESAPLAMA =====

            if abs(delta_px) < 10: 

                pid_output = 0

            else:

                pid_output = self.pid.update(delta_px)

            

            K_curve = 0.0

            pid_out = pid_output

            ang_out = curve * K_curve

            

            raw_target = STEER_CENTER + STEER_OFFSET + pid_out + ang_out

            

            # Step Limiting

            diff = raw_target - self.current_steer

            diff = np.clip(diff, -MAX_SERVO_STEP, MAX_SERVO_STEP)

            target_steer = self.current_steer + diff

            

            # Low Pass Filter

            target_steer = (LPF_ALPHA * target_steer) + ((1.0 - LPF_ALPHA) * self.current_steer)

            target_steer = np.clip(target_steer, 45.0, 135.0)

            

            self.current_steer = target_steer

            servo.set_angle(target_steer)



            # ===== MOTOR KONTROL =====

            if self.parked_forever:

                vehicle.stop()

                speed_status = "PARKED"

            elif self.stopped_by_light:

                vehicle.stop()

            elif self.person_wait_active:

                vehicle.stop()

            else:

                # Normal seyir

                abs_delta = abs(delta_px)

                if abs_delta < 40:

                    vehicle.forward_fast()

                    speed_status = "HIZLI"

                elif abs_delta < 80:

                    vehicle.forward_normal()

                    speed_status = "NORMAL"

                else:

                    vehicle.forward_slow()

                    speed_status = "YAVAS"



            # print telemetry every 10 frames

            self.frame_count = getattr(self, 'frame_count', 0) + 1

            if self.frame_count % 10 == 0:

                print(f"DEBUG: D={int(delta_px)}, St={int(target_steer)}, Spd={speed_status}")



            draw_hud(annotated, self.fps, speed_status, f"{int(delta_px)} C:{int(curve)}")



            if stream_q.full():

                stream_q.get_nowait()

            stream_q.put_nowait(annotated)



            time.sleep(0.05)



# ================== STREAM ==================

def mjpeg_generator():

    while not stop_event.is_set():

        try:

            frame = stream_q.get(timeout=0.5)

        except Empty:

            continue



        frame = cv2.resize(frame, (STREAM_W, STREAM_H))

        _, jpg = cv2.imencode(

            ".jpg", frame,

            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

        )

        yield (

            b"--frame\r\n"

            b"Content-Type: image/jpeg\r\n\r\n" +

            jpg.tobytes() + b"\r\n"

        )



@app.route("/")

def index():

    return render_template("index.html")



@app.route("/video_feed")

def video_feed():

    return Response(

        mjpeg_generator(),

        mimetype="multipart/x-mixed-replace; boundary=frame"

    )



# ================== MAIN ==================

def main():

    try:

        init_hardware()

        

        # Traffic Light Thread

        t_light = LightThread(

            frame_q=frame_q,

            shared_state=shared_state,

            lock=state_lock,

            model_path="/home/pi/Desktop/pi2/models/model.pt",

            fps=5,

            red_id=1,

            green_id=0

        )

        t_light.start()



        # Person Detection Thread

        t_person = PersonThread(

            frame_q=frame_q,

            shared_state=shared_state,

            lock=state_lock,

            model_path="/home/pi/Desktop/pi2/models/yolov8n.pt",

            fps=5,

            conf=0.45,         # Guven esigi dusuruldu (0.60 -> 0.45)

            classes=[0],       # Sadece 'person' (Class 0)

            min_box_area=1500  # Kucuk nesneler (fotograflar) icin alan dusuruldu (3000 -> 1500)

        )

        t_person.start()



        # Stop Sign Thread

        t_stop = StopThread(

            frame_q=frame_q,

            shared_state=shared_state,

            lock=state_lock,

            model_path="/home/pi/Desktop/pi2/models/stop.pt",

            fps=5,

            conf=0.65,        # Guven esigi artirildi (0.5 -> 0.65)

            min_box_area=1500 # Kucuk parazitleri yoksay

        )

        t_stop.start()



        CameraThread().start()

        DriveThread().start()

        app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)

    except KeyboardInterrupt:

        pass

    finally:

        print("Kapatiliyor...")

        stop_event.set()

        if servo: servo.cleanup()

        if vehicle: vehicle.stop()

        if picam2:

            try:

                picam2.stop()

                picam2.close()

            except:

                pass

        GPIO.cleanup()



if __name__ == "__main__":

    main()

