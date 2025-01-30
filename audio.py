"""
Audio Effects Processor for Raspberry Pi

This module implements an audio processing system with real-time effects:
- Gain control
- Threshold/distortion
- Delay
- Phaser

Hardware components:
- Raspberry Pi
- OLED Display
- Rotary Encoder
- Push Buttons

Authors: Mikołaj Domalewski, Michał Biegański, Michał Kaliszewski
"""

# Standard library imports
import time
from typing import Final

# Third-party imports
import numpy as np
import sounddevice as sd
from PIL import Image, ImageDraw, ImageFont, ImageOps
from scipy.io.wavfile import read
import RPi.GPIO as GPIO

# Local imports
import lib.oled.SSD1331 as SSD1331

# GPIO Pin configuration
CLK = 17  # Rotary encoder CLK pin
DT = 18   # Rotary encoder DT pin
BUTTON_NEXT = 5  # Button for switching effects
BUTTON_PLAY = 6  # Button for starting playback

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_NEXT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_PLAY, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Flags for button states and global variables initialization
button_next_pressed = False
button_play_pressed = False
global display_manager
current_effect = 0
delta = 0

# Path to test .wav file
file_path = "test.wav"

# Shared variables for effects parameters and current selection
effects_params = {
    "gain": 1.0,
    "threshold": 0.3,
    "delay_time": 0.3,
    "delay_feedback": 0.5,
    "phaser_depth": 1.0,
    "phaser_rate": 1.0
}

# Button and encoder callback functions


def button_next_callback(channel):
    global button_next_pressed
    button_next_pressed = True


def button_play_callback(channel):
    global button_play_pressed
    button_play_pressed = True


def encoder_callback(self, value):
    if value > 0:
        self.current_index = (self.current_index + 1) % len(self.photos)
    elif value < 0:
        self.current_index = (self.current_index - 1) % len(self.photos)
    self.start_time = time.time()


# Register button callbacks
GPIO.add_event_detect(BUTTON_NEXT, GPIO.FALLING,
                      callback=button_next_callback, bouncetime=200)
GPIO.add_event_detect(BUTTON_PLAY, GPIO.FALLING,
                      callback=button_play_callback, bouncetime=200)


def disable_interrupts():
    """
    Disable GPIO interrupts for buttons and encoder.
    """
    GPIO.remove_event_detect(BUTTON_NEXT)
    GPIO.remove_event_detect(BUTTON_PLAY)
    GPIO.remove_event_detect(encoder.clk_pin)
    GPIO.remove_event_detect(encoder.dt_pin)


def enable_interrupts():
    """
    Re-enable GPIO interrupts for buttons and encoder.
    """
    GPIO.add_event_detect(BUTTON_NEXT, GPIO.FALLING,
                          callback=button_next_callback, bouncetime=200)
    GPIO.add_event_detect(BUTTON_PLAY, GPIO.FALLING,
                          callback=button_play_callback, bouncetime=200)
    GPIO.add_event_detect(encoder.clk_pin, GPIO.BOTH,
                          callback=encoder.transition_occurred)
    GPIO.add_event_detect(encoder.dt_pin, GPIO.BOTH,
                          callback=encoder.transition_occurred)

# Initialize rotary encoder


class Encoder:
    def __init__(self, clk_pin, dt_pin):
        self.clk_pin = clk_pin
        self.dt_pin = dt_pin
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
        self.callback()

    def callback(self):
        global delta
        if self.value > 0:
            delta = 0.1
        elif self.value < 0:
            delta = -0.1


encoder = Encoder(CLK, DT)


class DisplayManager:
    def __init__(self, disp, font_path, icon_paths, icon_size=(14, 14)):
        self.disp = disp
        self.font_large = ImageFont.truetype(font_path, 14)
        self.font_small = ImageFont.truetype(font_path, 10)
        self.icons = [self.prepare_icon(path, icon_size)
                      for path in icon_paths]
        self.texts = ["0"] * 4

    def prepare_icon(self, filename, size, background="WHITE"):
        icon = Image.open(filename).convert("RGBA")
        background_image = Image.new("RGB", icon.size, background)
        background_image.paste(icon, (0, 0), icon)
        background_image = ImageOps.invert(background_image)
        return background_image.resize(size)

    def update_text(self, index, new_text):
        if 0 <= index < len(self.texts):
            self.texts[index] = new_text

    def display(self):
        image = Image.new("RGB", (96, 64), "BLACK")
        draw = ImageDraw.Draw(image)
        row_y = [0, 15, 30, 45]

        for i, (icon, text) in enumerate(zip(self.icons, self.texts)):
            image.paste(icon, (0, row_y[i] + 3))
            draw.text((14, row_y[i]), text, font=self.font_large, fill="WHITE")

        self.disp.ShowImage(image, 0, 0)

# Update display based on current effect


def update_display():
    effect_names = ["Gain", "Thresh", "Delay", "Phaser"]
    values = [effects_params["gain"], effects_params["threshold"],
              effects_params["delay_feedback"], effects_params["phaser_depth"]]
    for i, (name, value) in enumerate(zip(effect_names, values)):
        display_manager.update_text(i, f"{name}: {value:.2f}")
    display_manager.display()

# Distortion function


def apply_distortion(audio_data, gain, threshold):
    """
    Apply distortion effect to the audio data.
    Gain boosts the signal, and threshold clips it.
    """
    audio_data = audio_data * gain  # Apply gain
    audio_data = np.clip(audio_data, -threshold,
                         threshold)  # Apply hard clipping
    audio_data /= threshold  # Normalize back to the original range
    return audio_data

# Delay effect


def apply_delay(audio_data, delay_buffer, delay_time, feedback, samplerate):
    """
    Apply delay effect to the audio data.
    delay_time is in seconds, feedback is the echo strength.
    """
    delay_samples = int(delay_time * samplerate)
    delay_output = np.zeros_like(audio_data)
    for i in range(len(audio_data)):
        delay_output[i] = audio_data[i] + delay_buffer[i % delay_samples]
        delay_buffer[i % delay_samples] = audio_data[i] + \
            feedback * delay_buffer[i % delay_samples]
    return delay_output, delay_buffer

# Phaser effect


def apply_phaser(audio_data, phase, depth, rate, samplerate):
    """
    Apply phaser effect to the audio data.
    depth controls how deep the phase modulation is, rate controls speed.
    """
    modulation = np.sin(2 * np.pi * rate * phase / samplerate)
    return audio_data * (1 - depth * modulation)

# Stream callback function


def callback(outdata, frames, time, status):
    global data, position, effects_params, delay_buffer
    if status:
        print(f"Status: {status}")

    chunk = data[position:position + frames]

    gain = effects_params["gain"]
    threshold = effects_params["threshold"]
    delay_time = effects_params["delay_time"]
    delay_feedback = effects_params["delay_feedback"]
    phaser_depth = effects_params["phaser_depth"]
    phaser_rate = effects_params["phaser_rate"]

    # Apply effects
    chunk = apply_distortion(chunk, gain, threshold)
    chunk, delay_buffer = apply_delay(
        chunk, delay_buffer, delay_time, delay_feedback, samplerate)
    chunk = apply_phaser(chunk, position, phaser_depth,
                         phaser_rate, samplerate)

    outdata[:len(chunk)] = chunk.reshape(-1, data.shape[1])
    if len(chunk) < frames:
        outdata[len(chunk):] = 0

    position += frames
    if position >= len(data):
        raise sd.CallbackStop()


def menu_logic():
    global display_manager, delta
    # Initialize OLED display
    disp = SSD1331.SSD1331()
    disp.Init()
    disp.clear()
    display_manager = DisplayManager(disp, './lib/oled/Font.ttf', [
        "./icons/gain.jpg", "./icons/threshold.jpg", "./icons/delay.jpg", "./icons/phaser.jpg"
    ])
    global current_effect, effects_params, button_next_pressed, button_play_pressed

    def modify_effect(effect_idx, delta):
        param_names = ["gain", "threshold", "delay_feedback", "phaser_depth"]
        param_name = param_names[effect_idx]
        effects_params[param_name] += delta
        if param_name == "gain":
            effects_params[param_name] = min(
                max(0.0, effects_params[param_name]), 10.0)
        elif param_name == "threshold":
            effects_params[param_name] = min(
                max(0.1, effects_params[param_name]), 1.0)
        else:
            effects_params[param_name] = min(
                max(0.0, effects_params[param_name]), 1.0)

    update_display()

    while True:
        if button_next_pressed:
            button_next_pressed = False
            current_effect = (current_effect + 1) % 4
            update_display()
            time.sleep(0.2)

        if button_play_pressed:
            button_play_pressed = False
            disable_interrupts()
            play_audio()
            enable_interrupts()
            update_display()

        if delta != 0:
            modify_effect(current_effect, delta)
            update_display()
            delta = 0

# Play audio


def play_audio():
    global position, delay_buffer, samplerate, data
    samplerate, data = read(file_path)
    data = data / (32768.0 if data.dtype == np.int16 else 2147483648.0)
    position = 0
    delay_buffer = np.zeros_like(data)
    with sd.OutputStream(
        samplerate=samplerate,
        blocksize=4096,
        latency='high',
        channels=data.shape[1] if len(data.shape) > 1 else 1,
        callback=callback
    ):
        sd.sleep(int(len(data) / samplerate * 1000))


# Main program
try:
    menu_logic()
except Exception as e:
    print(f"Error: {e}")
