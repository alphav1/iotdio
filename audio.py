import wave
import pyaudio
import numpy as np


def apply_distortion(data, distortion_level):
    """
    Applies distortion to the audio signal.
    :param data: Raw audio data (bytes).
    :param distortion_level: Level of distortion (float, 0.0 to 1.0).
    :return: Processed audio data (bytes).
    """
    # Convert raw audio bytes to NumPy array
    audio_signal = np.frombuffer(data, dtype=np.int16)

    # Apply distortion effect
    max_val = np.iinfo(np.int16).max
    audio_signal = audio_signal * (1 + distortion_level * 10)
    audio_signal = np.clip(audio_signal, -max_val, max_val)

    # Convert back to raw bytes
    return audio_signal.astype(np.int16).tobytes()


def play_audio_with_distortion(file_path, distortion_level=0.5):
    # Open the WAV file
    wf = wave.open(file_path, 'rb')

    # Create a PyAudio instance
    audio = pyaudio.PyAudio()

    # Define chunk size
    chunk_size = 1024

    # Open a stream for playback
    stream = audio.open(format=audio.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True,
                        frames_per_buffer=chunk_size)

    # Read and process audio data in chunks
    data = wf.readframes(chunk_size)

    while data:
        # Apply distortion to the audio data
        modified_data = apply_distortion(data, distortion_level)

        # Write modified audio data to the stream
        stream.write(modified_data)
        data = wf.readframes(chunk_size)

    # Stop and close the stream
    stream.stop_stream()
    stream.close()

    # Close PyAudio and the WAV file
    audio.terminate()
    wf.close()


if __name__ == "__main__":
    # Specify the path to your .wav file
    file_path = "test.wav"  # Replace with the path to your WAV file

    # Set the distortion level (0.0 = no distortion, 1.0 = max distortion)
    distortion_level = 0.5

    play_audio_with_distortion(file_path, distortion_level)
