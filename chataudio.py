import sounddevice as sd
from scipy.io.wavfile import read
import numpy as np
import threading
import RPi.GPIO as GPIO
from encoder import Encoder
from display import DisplayManager
import time

import sounddevice as sd
from scipy.io.wavfile import read
import numpy as np
import threading
import RPi.GPIO as GPIO
from encoder import Encoder
from display import DisplayManager
import time

# Path to your .wav file
file_path = "Get Lucky - Daft Punk.wav"

# Shared variables for distortion, delay, and phaser parameters
effects_params = {
    "gain": 1.0,  # Initial gain for distortion
    "threshold": 0.3,  # Initial threshold for distortion
    "delay_time": 0.3,  # Initial delay time in seconds
    "delay_feedback": 0.5,  # Delay feedback (strength of the echo)
    "phaser_depth": 0.5,  # Depth of the phaser effect
    "phaser_rate": 1.0  # Rate of the phaser sweep (speed of the modulation)
}
lock = threading.Lock()

# Set the buffer size and latency
sd.default.blocksize = 4096  # Increase buffer size
sd.default.latency = 'high'  # Set low latency

# GPIO setup for buttons
buttonRed = 23
buttonGreen = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(buttonRed, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buttonGreen, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize the encoder
encoder = Encoder(17, 18)

# Initialize the display
disp = SSD1331.SSD1331()
disp.Init()
display_manager = DisplayManager(disp, './lib/oled/Font.ttf', [
                                 "./icons/gain.jpg", "./icons/threshold.jpg", "./icons/delay.jpg", "./icons/phaser.jpg"])

# Distortion function


def apply_distortion(audio_data, gain, threshold):
    audio_data = audio_data * gain  # Apply gain
    audio_data = np.clip(audio_data, -threshold,
                         threshold)  # Apply hard clipping
    audio_data /= threshold  # Normalize back to the original range
    return audio_data

# Delay effect


def apply_delay(audio_data, delay_buffer, delay_time, feedback, samplerate):
    delay_samples = int(delay_time * samplerate)
    delay_output = np.zeros_like(audio_data)
    for i in range(len(audio_data)):
        delay_output[i] = audio_data[i] + delay_buffer[i]
        delay_buffer[i] = audio_data[i] + feedback * delay_buffer[i]
    return delay_output, delay_buffer

# Phaser effect


def apply_phaser(audio_data, phase, depth, rate, samplerate):
    modulation = np.sin(2 * np.pi * rate * phase / samplerate)
    return audio_data * (1 - depth * modulation)  # Phaser effect

# Stream callback function


def callback(outdata, frames, time, status):
    global data, position, effects_params, delay_buffer
    if status:
        print(f"Status: {status}")

    # Get the current chunk of audio
    chunk = data[position:position + frames]

    # Read current effects parameters safely
    with lock:
        gain = effects_params["gain"]
        threshold = effects_params["threshold"]
        delay_time = effects_params["delay_time"]
        delay_feedback = effects_params["delay_feedback"]
        phaser_depth = effects_params["phaser_depth"]
        phaser_rate = effects_params["phaser_rate"]

    # Apply distortion
    distorted_chunk = apply_distortion(chunk, gain, threshold)

    # Apply delay
    delayed_chunk, delay_buffer = apply_delay(
        distorted_chunk, delay_buffer, delay_time, delay_feedback, samplerate)

    # Apply phaser
    phaser_chunk = apply_phaser(
        delayed_chunk, position, phaser_depth, phaser_rate, samplerate)

    # Output the processed audio
    outdata[:len(phaser_chunk)] = phaser_chunk.reshape(-1, data.shape[1])

    # Pad with silence if needed (e.g., at the end of the file)
    if len(phaser_chunk) < frames:
        outdata[len(phaser_chunk):] = 0

    # Update position
    position += frames
    if position >= len(data):
        raise sd.CallbackStop()  # Stop the stream at the end of the audio

# Function to update effects parameters in real time


def update_effects():
    global effects_params
    effect_names = ["gain", "threshold", "delay_time",
                    "delay_feedback", "phaser_depth", "phaser_rate"]
    current_effect_index = 0

    print("You can adjust effects in real time using the encoder and buttons.")
    print("Press the red button to change the effect parameter.")
    print("Use the encoder to adjust the current effect parameter.")

    while True:
        # Check if the red button is pressed to change the effect parameter
        if GPIO.input(buttonRed) == GPIO.LOW:
            current_effect_index = (
                current_effect_index + 1) % len(effect_names)
            print(f"Selected effect: {effect_names[current_effect_index]}")
            time.sleep(0.5)  # Debounce delay

        # Adjust the current effect parameter using the encoder
        with lock:
            effects_params[effect_names[current_effect_index]
                           ] += encoder.value * 0.01
            encoder.value = 0  # Reset encoder value

        # Update the display with the current effect parameters
        for i, effect in enumerate(effect_names):
            display_manager.update_text(
                i, f"{effect}: {effects_params[effect]:.2f}")
        display_manager.display()

        # Check if the green button is pressed to exit
        if GPIO.input(buttonGreen) == GPIO.LOW:
            print("Exiting effects adjustment.")
            break


# Read the audio file
try:
    samplerate, data = read(file_path)

    # Normalize data to range [-1, 1] for floating-point processing
    if data.dtype == np.int16:
        data = data / 32768.0  # Convert 16-bit PCM to float
    elif data.dtype == np.int32:
        data = data / 2147483648.0  # Convert 32-bit PCM to float

    position = 0  # Track current playback position
    delay_buffer = np.zeros_like(data)  # Initialize the delay buffer

    # Start a thread for real-time parameter adjustment
    threading.Thread(target=update_effects, daemon=True).start()

    print(f"Playing with effects '{file_path}'...")
    with sd.OutputStream(
        samplerate=samplerate,
        channels=data.shape[1] if len(data.shape) > 1 else 1,
        callback=callback
    ):
        # Sleep until playback is complete
        sd.sleep(int(len(data) / samplerate * 1000))
    print("Playback finished.")
except FileNotFoundError:
    print(f"File not found: {file_path}")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    GPIO.cleanup()
