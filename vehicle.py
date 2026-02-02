# vehicle.py

from gpiozero import Motor, PWMOutputDevice



class Vehicle:

    def __init__(self):

        # Sol motor (INA, INB)

        self.motor_left = Motor(forward=27, backward=22)

        # SaÄŸ motor (INC, IND)

        self.motor_right = Motor(forward=23, backward=24)



        # ENA ve ENB PWM Ã§Ä±kÄ±ÅŸlarÄ±

        self.pwm_left = PWMOutputDevice(18)   # ENA

        self.pwm_right = PWMOutputDevice(19)  # ENB



        # 0.0 = durdu, 1.0 = tam gÃ¼Ã§

        self.SPEED_FAST   = 0.30

        self.SPEED_NORMAL = 0.25

        self.SPEED_SLOW   = 0.22



        self.stop()  # BaÅŸlangÄ±Ã§ta motorlar durur



    def set_speed(self, left_speed, right_speed):

        # GÃ¼venli aralÄ±k

        self.pwm_left.value  = max(0.0, min(1.0, left_speed))

        self.pwm_right.value = max(0.0, min(1.0, right_speed))



    def forward_fast(self):

        self.motor_left.forward()

        self.motor_right.forward()

        self.set_speed(self.SPEED_FAST, self.SPEED_FAST)



    def forward_normal(self):

        self.motor_left.forward()

        self.motor_right.forward()

        self.set_speed(self.SPEED_NORMAL, self.SPEED_NORMAL)



    def forward_slow(self):

        self.motor_left.forward()

        self.motor_right.forward()

        self.set_speed(self.SPEED_SLOW, self.SPEED_SLOW)



    def backward(self):

        self.motor_left.backward()

        self.motor_right.backward()

        self.set_speed(self.SPEED_NORMAL, self.SPEED_NORMAL)



    def stop(self):

        self.set_speed(0.0, 0.0)

