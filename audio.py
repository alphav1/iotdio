import sounddevice as sd
from scipy.io.wavfile import read
import numpy as np
import threading

# Path to your .wav file
file_path = "test.wav"

# Shared variables for distortion, delay, and phaser parameters
effects_params = {
    "gain": 10.0,  # Initial gain for distortion
    "threshold": 0.3,  # Initial threshold for distortion
    "delay_time": 0.3,  # Initial delay time in seconds
    "delay_feedback": 0.5,  # Delay feedback (strength of the echo)
    "phaser_depth": 0.5,  # Depth of the phaser effect
    "phaser_rate": 1.0  # Rate of the phaser sweep (speed of the modulation)
}
lock = threading.Lock()

# Distortion function
def apply_distortion(audio_data, gain, threshold):
    """
    Apply distortion effect to the audio data.
    Gain boosts the signal, and threshold clips it.
    """
    audio_data = audio_data * gain  # Apply gain
    audio_data = np.clip(audio_data, -threshold, threshold)  # Apply hard clipping
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
        delay_output[i] = audio_data[i] + delay_buffer[i]
        delay_buffer[i] = audio_data[i] + feedback * delay_buffer[i]
    return delay_output, delay_buffer

# Phaser effect
def apply_phaser(audio_data, phase, depth, rate, samplerate):
    """
    Apply phaser effect to the audio data.
    depth controls how deep the phase modulation is, rate controls speed.
    """
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
    delayed_chunk, delay_buffer = apply_delay(distorted_chunk, delay_buffer, delay_time, delay_feedback, samplerate)
    
    # Apply phaser
    phaser_chunk = apply_phaser(delayed_chunk, position, phaser_depth, phaser_rate, samplerate)
    
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
    print("You can adjust effects in real time:")
    print("Commands: 'g' to adjust gain, 't' to adjust threshold, 'd' to adjust delay, 'p' to adjust phaser, 'q' to quit adjustments.")
    
    while True:
        command = input("Enter command (g/t/d/p/q): ").strip().lower()
        if command == "g":
            try:
                new_gain = float(input("Enter new gain value: "))
                with lock:
                    effects_params["gain"] = new_gain
                print(f"Gain updated to {new_gain}")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")
        elif command == "t":
            try:
                new_threshold = float(input("Enter new threshold value (0.0 to 1.0): "))
                if 0.0 < new_threshold <= 1.0:
                    with lock:
                        effects_params["threshold"] = new_threshold
                    print(f"Threshold updated to {new_threshold}")
                else:
                    print("Threshold must be between 0.0 and 1.0.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")
        elif command == "d":
            try:
                new_delay_time = float(input("Enter new delay time in seconds: "))
                new_feedback = float(input("Enter new delay feedback (0.0 to 1.0): "))
                with lock:
                    effects_params["delay_time"] = new_delay_time
                    effects_params["delay_feedback"] = new_feedback
                print(f"Delay updated to {new_delay_time} seconds with {new_feedback} feedback.")
            except ValueError:
                print("Invalid input. Please enter numeric values.")
        elif command == "p":
            try:
                new_phaser_depth = float(input("Enter new phaser depth (0.0 to 1.0): "))
                new_phaser_rate = float(input("Enter new phaser rate: "))
                with lock:
                    effects_params["phaser_depth"] = new_phaser_depth
                    effects_params["phaser_rate"] = new_phaser_rate
                print(f"Phaser updated to {new_phaser_depth} depth with {new_phaser_rate} rate.")
            except ValueError:
                print("Invalid input. Please enter numeric values.")
        elif command == "q":
            print("Exiting effects adjustment.")
            break
        else:
            print("Invalid command. Use 'g', 't', 'd', 'p', or 'q'.")

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
        sd.sleep(int(len(data) / samplerate * 1000))  # Sleep until playback is complete
    print("Playback finished.")
except FileNotFoundError:
    print(f"File not found: {file_path}")
except Exception as e:
    print(f"An error occurred: {e}")
