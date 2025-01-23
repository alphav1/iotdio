import RPi.GPIO as GPIO

CLK = 17  # GPIO pin for rotary encoder CLK
DT = 18   # GPIO pin for rotary encoder DT

GPIO.setmode(GPIO.BCM)
GPIO.setup(CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class Encoder:
    def __init__(self, clk_pin, dt_pin, callback=None):
        self.clk_pin = clk_pin
        self.dt_pin = dt_pin
        self.callback = callback
        self.value = 0
        self.last_state = self.read_state()

        GPIO.add_event_detect(self.clk_pin, GPIO.BOTH,
                              callback=self.transition_occurred)
        GPIO.add_event_detect(self.dt_pin, GPIO.BOTH,
                              callback=self.transition_occurred)

    def read_state(self):
        return GPIO.input(self.clk_pin), GPIO.input(self.dt_pin)

    def transition_occurred(self, channel):
        current_state = self.read_state()
        if self.last_state == (0, 0):
            if current_state == (0, 1):
                self.value += 1
            elif current_state == (1, 0):
                self.value -= 1
        elif self.last_state == (0, 1):
            if current_state == (1, 1):
                self.value += 1
            elif current_state == (0, 0):
                self.value -= 1
        elif self.last_state == (1, 0):
            if current_state == (1, 1):
                self.value -= 1
            elif current_state == (0, 0):
                self.value += 1
        elif self.last_state == (1, 1):
            if current_state == (0, 1):
                self.value -= 1
            elif current_state == (1, 0):
                self.value += 1

        self.last_state = current_state

        if self.callback:
            self.callback(self.value)
