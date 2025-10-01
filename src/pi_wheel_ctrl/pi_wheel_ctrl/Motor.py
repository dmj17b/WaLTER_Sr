import RPi.GPIO as GPIO

class BrushedMotor:
    def __init__(self, P_pin: int, A_pin: int, B_pin: int):
        self.p_pin = int(P_pin)
        self.a_pin = int(A_pin)
        self.b_pin = int(B_pin)

        self.duty_tol = 5
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.a_pin, GPIO.OUT)
        GPIO.setup(self.b_pin, GPIO.OUT)
        GPIO.setup(self.p_pin, GPIO.OUT)
        self.p = GPIO.PWM(self.p_pin, 100)
        self.p.start(0)

    def drive(self, signed_duty_cycle):
        if(signed_duty_cycle > self.duty_tol):
            self._fwd_drive(abs(signed_duty_cycle))
        elif(signed_duty_cycle < -self.duty_tol):
            self._rev_drive(abs(signed_duty_cycle))
        else:
            self._brake()

    def _fwd_drive(self, duty_cycle):
        GPIO.output(self.a_pin, GPIO.HIGH)
        GPIO.output(self.b_pin, GPIO.LOW)
        self.p.ChangeDutyCycle(duty_cycle)

    def _rev_drive(self, duty_cycle):
        GPIO.output(self.a_pin, GPIO.LOW)
        GPIO.output(self.b_pin, GPIO.HIGH)
        self.p.ChangeDutyCycle(duty_cycle)

    def _brake(self):
        GPIO.output(self.a_pin, GPIO.LOW)
        GPIO.output(self.b_pin, GPIO.LOW)

    def _coast(self):
        GPIO.output(self.a_pin, GPIO.HIGH)
        GPIO.output(self.b_pin, GPIO.HIGH)
