#!/usr/bin/env python
"""GpioController module for controlling rover motors using Raspberry Pi GPIO pins."""
try:
    import RPi.GPIO as GPIO
except ImportError:
    # For non-Raspberry Pi environments, create a stub for GPIO
    class GPIOStub:
        BCM = 'BCM'
        OUT = 'OUT'
        def setmode(self, mode):
            print(f"[GPIOStub] setmode({mode})")
        def setup(self, pin, mode):
            print(f"[GPIOStub] setup(pin={pin}, mode={mode})")
        def PWM(self, pin, frequency):
            print(f"[GPIOStub] PWM(pin={pin}, frequency={frequency})")
            return self.PWMStub(pin, frequency)
        def cleanup(self):
            print("[GPIOStub] cleanup()")
        class PWMStub:
            def __init__(self, pin, frequency):
                self.pin = pin
                self.frequency = frequency
            def start(self, duty_cycle):
                print(f"[GPIOStub] PWM on pin {self.pin} start with duty cycle {duty_cycle}")
            def ChangeDutyCycle(self, duty_cycle):
                print(f"[GPIOStub] PWM on pin {self.pin} change duty cycle to {duty_cycle}")
            def stop(self):
                print(f"[GPIOStub] PWM on pin {self.pin} stop")
    GPIO = GPIOStub()


class GpioController:
    def __init__(self, left_pin, right_pin):
        """Initialize GPIO pins and setup PWM for motor speed control."""
        self.left_pin = left_pin
        self.right_pin = right_pin

        GPIO.setmode(GPIO.BCM)  # using BCM numbering
        GPIO.setup(self.left_pin, GPIO.OUT)
        GPIO.setup(self.right_pin, GPIO.OUT)

        # Setup PWM on the pins with a frequency of 1000Hz
        self.left_pwm = GPIO.PWM(self.left_pin, 1000)
        self.right_pwm = GPIO.PWM(self.right_pin, 1000)
        
        # Start PWM with 0% duty cycle
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds using PWM duty cycle (0-100%)."""
        self.left_pwm.ChangeDutyCycle(left_speed)
        self.right_pwm.ChangeDutyCycle(right_speed)

    def cleanup(self):
        """Stop PWM and clean up GPIO configuration."""
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()
