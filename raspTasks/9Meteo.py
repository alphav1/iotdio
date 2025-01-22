import time
import board
import busio
from datetime import datetime
import adafruit_bme280.advanced as adafruit_bme280
from PIL import Image, ImageDraw, ImageFont, ImageOps
import lib.oled.SSD1331 as SSD1331
import config
import traceback

# Initialize I2C for BME280
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize BME280 sensor
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)

# Set BME280 sensor parameters
bme280.mode = adafruit_bme280.MODE_NORMAL
bme280.standby_period = adafruit_bme280.STANDBY_TC_500
bme280.iir_filter = adafruit_bme280.IIR_FILTER_X16
bme280.overscan_pressure = adafruit_bme280.OVERSCAN_X16
bme280.overscan_humidity = adafruit_bme280.OVERSCAN_X1
bme280.overscan_temperature = adafruit_bme280.OVERSCAN_X2

# Initialize SSD1331 OLED display
try:
    disp = SSD1331.SSD1331()
    disp.Init()
    disp.clear()
except Exception as e:
    print("Error initializing SSD1331:", e)
    config.module_exit()
    exit()

# Create font
# Smaller font for compact display
font_large = ImageFont.truetype('./lib/oled/Font.ttf', 14)
font_small = ImageFont.truetype('./lib/oled/Font.ttf', 10)  # Even smaller


def prepare_icon(filename, size, background="WHITE"):
    """Load an icon, handle transparency, and resize to the specified size."""
    icon = Image.open(filename).convert(
        "RGBA")  # Ensure the icon has an alpha channel
    # Create a new image with the same size as the icon, filled with the background color
    background_image = Image.new("RGB", icon.size, background)
    # Paste the icon onto the background, handling transparency
    background_image.paste(icon, (0, 0), icon)
    background_image = ImageOps.invert(background_image)
    # Resize to the desired size
    return background_image.resize(size, Image.ANTIALIAS)


# Load icons and resize them to 12x12 pixels
ICON_SIZE = (14, 14)  # Resizing for smaller display space
icon_temp = prepare_icon("icons/icon_temperature.png", ICON_SIZE)
icon_humidity = prepare_icon("icons/icon_humidity.png", ICON_SIZE)
icon_pressure = prepare_icon("icons/icon_pressure.png", ICON_SIZE)
icon_altitude = prepare_icon("icons/icon_altitude.png", ICON_SIZE)

# Function to calculate altitude
SEA_LEVEL_PRESSURE = 992.0  # Default value in hPa


def calculate_altitude(pressure, sea_level_pressure=SEA_LEVEL_PRESSURE):
    """Calculate altitude based on pressure and sea-level pressure."""
    return 44330 * (1.0 - (pressure / sea_level_pressure) ** (1 / 5.255))


# Main loop
try:
    disp.clear()
    while True:
        # Read data from the BME280 sensor
        temperature = bme280.temperature
        humidity = bme280.humidity
        pressure = bme280.pressure
        altitude = calculate_altitude(pressure)
        current_time = datetime.now().strftime("%H:%M")

        # Create a new blank image
        image = Image.new("RGB", (disp.width, disp.height), "BLACK")
        draw = ImageDraw.Draw(image)

        # Layout coordinates
        row1_y = 0
        row2_y = 15
        row3_y = 30
        row4_y = 45

        # Draw data with icons
        image.paste(icon_temp, (0, row1_y+3))  # Place icon
        image.paste(icon_humidity, (0, row2_y+3))  # Place icon
        image.paste(icon_pressure, (0, row3_y+3))  # Place icon
        image.paste(icon_altitude, (0, row4_y+3))  # Place icon

        # icons_image = image.copy()
        # Row 1: Temperature
        draw.text((14, row1_y), f" {temperature:.1f}℃",
                  font=font_large, fill="WHITE")
        # Row 2: Humidity
        draw.text((14, row2_y), f" {humidity:.1f}%",
                  font=font_large, fill="WHITE")
        # Row 3: Pressure
        draw.text((14, row3_y), f" {pressure:.1f} hPa",
                  font=font_large, fill="WHITE")
        # Row 4: Altitude
        draw.text((14, row4_y), f" {altitude:.1f} masl",
                  font=font_large, fill="WHITE")
        # Draw time at the top right
        draw.text((65, 0), current_time, font=font_small, fill="YELLOW")

        # Show only icon image
        # disp.ShowImage(icons_image, 0, 0)
        # Display the image
        disp.ShowImage(image, 0, 0)

        # Wait for a second before refreshing
        time.sleep(5)


except KeyboardInterrupt:
    print("\nExiting...")
    disp.clear()
    config.module_exit()
    exit()

except Exception as e:
    print("Error:", e)
    print(traceback.format_exc())
    disp.clear()
    config.module_exit()
    exit()
