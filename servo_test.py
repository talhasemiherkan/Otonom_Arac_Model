#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from servo_motor import Servo_Motor
import RPi.GPIO as GPIO

SERVO_PIN = 17   # Senin kullandÄ±ÄŸÄ±n pin
CENTER = 90.0
RIGHT  = 120.0
LEFT   = 60.0
WAIT   = 2.0     # saniye

def main():
    print("ğŸ”§ Servo test basliyor...")

    servo = Servo_Motor(servo_pin=SERVO_PIN)

    try:
        print("â¡ï¸ Merkez")
        servo.set_angle(CENTER)
        time.sleep(WAIT)

        print("â¡ï¸ Sag")
        servo.set_angle(RIGHT)
        time.sleep(WAIT)

        print("â¡ï¸ Sol")
        servo.set_angle(LEFT)
        time.sleep(WAIT)

        print("â¡ï¸ Merkez")
        servo.set_angle(CENTER)
        time.sleep(WAIT)

        print("âœ… Test tamamlandi")

    except KeyboardInterrupt:
        print("â›” Test iptal edildi")

    finally:
        servo.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
