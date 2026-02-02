# servo_motor.py

import time

import RPi.GPIO as GPIO



class Servo_Motor:

    """

    Direksiyon servosu:

      - BCM 17 pin

      - 50 Hz PWM

      - 0â€“180 derece aÃ§Ä± ile kontrol

      - Merkez: 90 derece

    """



    def __init__(self, servo_pin=17):

        self.pin = servo_pin

        GPIO.setmode(GPIO.BCM)

        GPIO.setwarnings(False)

        try:

            GPIO.setup(self.pin, GPIO.OUT)

        except RuntimeWarning:

            pass # Zaten ayarlÄ±ysa sorun yok



        self.pwm = GPIO.PWM(self.pin, 50)  # 50 Hz

        self.pwm.start(0)

        time.sleep(0.3)



        self.center()



    def calculate_duty_cycle(self, angle: float) -> float:

        """

        0 derece  -> ~2.5 duty

        180 derece -> ~12.5 duty

        """

        angle = max(0.0, min(180.0, float(angle)))

        duty = 2.5 + (angle / 180.0) * 10.0

        return duty



    def set_angle(self, angle: float):

        duty = self.calculate_duty_cycle(angle)

        self.pwm.ChangeDutyCycle(duty)

        # time.sleep(0.02)  <-- KALDIRILDI: Loop'u yavaÅŸlatÄ±yor

        # titremeyi azaltmak iÃ§in gerekirse baÅŸka yÃ¶ntem kullanÄ±lÄ±r



    def center(self):

        """Direksiyonu 90 derece (dÃ¼z) konuma getir."""

        self.set_angle(90)



    def cleanup(self):

        """Sadece PWM'i kapat. GPIO.cleanup() ana programda Ã§aÄŸrÄ±lacak."""

        self.pwm.stop()

