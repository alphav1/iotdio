import time
import board
import busio
from datetime import datetime
import adafruit_bme280.advanced as adafruit_bme280
import paho.mqtt.client as mqtt_client
import neopixel
import traceback

# Adafruit.IO configuration
'ADAFRUIT_IO_USERNAME ==>REDACTED'
'ADAFRUIT_IO_KEY==>REDACTED'
BROKER = "io.adafruit.com"
PORT = 1883

# Feeds
FEED_TEMPERATURE = f"{ADAFRUIT_IO_USERNAME}/feeds/temperature"
FEED_PRESSURE = f"{ADAFRUIT_IO_USERNAME}/feeds/pressure"
FEED_HUMIDITY = f"{ADAFRUIT_IO_USERNAME}/feeds/humidity"
FEED_RGB_LED = f"{ADAFRUIT_IO_USERNAME}/feeds/rgb-led"

# NeoPixel configuration
LED_PIN = board.D18  # GPIO pin connected to the NeoPixel
NUM_PIXELS = 8       # Number of WS2812 LEDs
pixels = neopixel.NeoPixel(LED_PIN, NUM_PIXELS)

# Initialize I2C for BME280
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize BME280 sensor
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)

# MQTT callback functions


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.connected_flag = True          # set flag
        print("Connected OK")
        client.subscribe(FEED_RGB_LED, qos=0)
    else:
        print(f"Failed to connect, return code {rc}")


def on_message(client, userdata, msg):
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
    if msg.topic == FEED_RGB_LED:
        try:
            # Parse color from Adafruit.IO (e.g., "255,0,0" for red)
            color = msg.payload.decode("utf-8")
            rgb_tuple = hex_to_rgb(color)
            set_led_color(rgb_tuple)
        except Exception as e:
            print("Error parsing color:", e)


def set_led_color(color):
    """Set all LEDs to the specified RGB color."""
    pixels.fill(color)
    pixels.show()


def hex_to_rgb(hex_color):
    """Convert a hexadecimal color string to an RGB tuple."""
    # Remove '#' if present
    hex_color = hex_color.lstrip('#')
    # Convert to a tuple of integers
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

# Connect to Adafruit.IO via MQTT


def connect_mqtt():
    client = mqtt_client.Client(
        client_id="MeteoStation", callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)
    client.username_pw_set(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT)
    return client

# Publish sensor data


def publish_data(client):
    temperature = bme280.temperature
    pressure = bme280.pressure
    humidity = bme280.humidity

    try:
        client.publish(FEED_TEMPERATURE, temperature)
        client.publish(FEED_PRESSURE, pressure)
        client.publish(FEED_HUMIDITY, humidity)
        print(f"Published: {temperature}°C, {pressure} hPa, {humidity}%")
    except Exception as e:
        print("Error publishing data:", e)


# Main loop
try:
    client = connect_mqtt()
    client.loop_start()

    while True:
        publish_data(client)
        time.sleep(10)  # Publish data every 10 seconds
except KeyboardInterrupt:
    print("\nExiting...")
    pixels.fill((0, 0, 0))  # Turn off LEDs
    pixels.show()
    exit()
except Exception as e:
    print("Error:", e)
    print(traceback.format_exc())
    pixels.fill((0, 0, 0))
    pixels.show()
    exit()
