import board
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO
import time

class Wheel:
    def __init__(self, pwm_channel, channel1, channel2):
        self.pwm = pwm_channel
        self.channel1 = channel1
        self.channel2 =  channel2

    def move(self, throttle):
        duty_cycle = int(0xFFFF * abs(throttle))
        self.pwm.duty_cycle = duty_cycle
        if throttle>=0:
            self.channel1.duty_cycle=0
            self.channel2.duty_cycle=0xFFFF
        else:
            self.channel1.duty_cycle=0xFFFF
            self.channel2.duty_cycle=0


class WheelGPIO(Wheel):
    def move(self, throttle):
        duty_cycle = int(0xFFFF * abs(throttle))
        self.pwm.duty_cycle = duty_cycle
        if throttle>=0:
            GPIO.output(self.channel1,0)
            GPIO.output(self.channel2,1)
        else:
            GPIO.output(self.channel1,1)
            GPIO.output(self.channel2,0)


class Taobot:
    t_time = 0.01
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 1600
        GPIO.setwarnings(True)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(25,GPIO.OUT)
        GPIO.setup(24,GPIO.OUT)
        
        self.LF = Wheel(self.pca.channels[0],
                        self.pca.channels[2],
                        self.pca.channels[1])

        self.RR = Wheel(self.pca.channels[5],
                        self.pca.channels[4],
                        self.pca.channels[3])
        
        self.LR = Wheel(self.pca.channels[6],
                        self.pca.channels[7],
                        self.pca.channels[8])
        
        self.RF = WheelGPIO(self.pca.channels[11],
                        25,
                        24)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.LF.move(0)
        self.RR.move(0)
        self.LR.move(0)
        self.RF.move(0)
        GPIO.cleanup()

    def forward(self, throttle):
        self.LF.move(throttle)
        self.RR.move(throttle)
        self.LR.move(throttle)
        self.RF.move(throttle)
        time.sleep(Taobot.t_time)
    
    def backward(self, throttle):
        self.LF.move(-throttle)
        self.RR.move(-throttle)
        self.LR.move(-throttle)
        self.RF.move(-throttle)
        time.sleep(Taobot.t_time)

    def left(self, throttle):
        self.LF.move(-throttle)
        self.RR.move(throttle)
        self.LR.move(-throttle)
        self.RF.move(throttle)
        time.sleep(Taobot.t_time)
    
    def right(self, throttle):
        self.LF.move(throttle)
        self.RR.move(-throttle)
        self.LR.move(throttle)
        self.RF.move(-throttle)
        time.sleep(Taobot.t_time)
    
    def stop(self):
        self.LF.move(0)
        self.RR.move(0)
        self.LR.move(0)
        self.RF.move(0)
        time.sleep(Taobot.t_time)
