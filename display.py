import time
from PIL import Image, ImageDraw, ImageFont, ImageOps
import lib.oled.SSD1331 as SSD1331


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
        return background_image.resize(size, Image.ANTIALIAS)

    def update_text(self, index, new_text):
        if 0 <= index < len(self.texts):
            self.texts[index] = new_text

    def display(self):
        image = Image.new("RGB", (self.disp.width, self.disp.height), "BLACK")
        draw = ImageDraw.Draw(image)
        row_y = [0, 15, 30, 45]

        for i, (icon, text) in enumerate(zip(self.icons, self.texts)):
            image.paste(icon, (0, row_y[i] + 3))
            draw.text((14, row_y[i]), text, font=self.font_large, fill="WHITE")

        self.disp.ShowImage(image, 0, 0)


# display_manager = DisplayManager(disp, './lib/oled/Font.ttf', [
#                                  "./icons/gain.jpg", "./icons/threshold.jpg", "./icons/delay.jpg", "./icons/phaser.jpg"])
