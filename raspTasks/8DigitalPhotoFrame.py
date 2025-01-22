import os
import time
from PIL import Image, ImageDraw, ImageFont
import lib.oled.SSD1331 as SSD1331
from datetime import datetime
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
        return (GPIO.input(self.clk_pin), GPIO.input(self.dt_pin))

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


class DigitalPhotoFrame:
    def __init__(self, photo_dir, display_interval=5):
        self.photo_dir = photo_dir
        self.display_interval = display_interval
        self.photos = self.load_photos()
        self.current_index = 0
        self.start_time = time.time()

        self.encoder = Encoder(CLK, DT, callback=self.encoder_callback)

        self.disp = SSD1331.SSD1331()
        self.disp.Init()
        self.disp.clear()

        self.font = ImageFont.load_default()

    def load_photos(self):
        photos = [os.path.join(self.photo_dir, f) for f in os.listdir(self.photo_dir)
                  if f.lower().endswith(('png', 'jpg', 'jpeg'))]
        return photos

    def get_photo_date(self, photo_path):
        try:
            timestamp = os.path.getmtime(photo_path)
            return [datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d'), datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')]
        except Exception as e:
            print(f"Error reading date: {e}")
            return "Unknown"

    def draw_frame(self, image):
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, image.width - 1, image.height - 1),
                       outline=255, width=2)

    def display_photo(self, photo_path):
        try:
            image = Image.open(photo_path)
            image = image.resize((96, 64))

            self.draw_frame(image)

            date_text = self.get_photo_date(photo_path)
            draw = ImageDraw.Draw(image)
            draw.text((2, 2), date_text[0], font=self.font, fill=255)
            draw.text((2, 10), date_text[1], font=self.font, fill=255)

            self.disp.ShowImage(image, 0, 0)
        except Exception as e:
            print(f"Error displaying photo: {e}")

    def encoder_callback(self, value):
        if value > 0:
            self.current_index = (self.current_index + 1) % len(self.photos)
        elif value < 0:
            self.current_index = (self.current_index - 1) % len(self.photos)
        self.start_time = time.time()

    def run(self):
        while True:
            if not self.photos:
                print("No photos found in the specified directory.")
                break
            self.display_photo(self.photos[self.current_index])
            if (time.time() - self.start_time) > self.display_interval:
                self.current_index = (
                    self.current_index + 1) % len(self.photos)
                self.start_time = time.time()


if __name__ == "__main__":
    photo_dir = "/home/pi/tests/photos"
    display_interval = 5

    frame = DigitalPhotoFrame(photo_dir, display_interval)
    try:
        frame.run()
    except KeyboardInterrupt:
        print("Exiting...")
        GPIO.cleanup()
