import time
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import Adafruit_SSD1306   # This is the driver chip for the Adafruit PiOLED


# 128x32 display with hardware I2C:
# setting gpio to 1 is hack to avoid platform detection
disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
y = padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

try:
    while True:

        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, width, height), outline=0, fill=0)

        # Line 1
        draw.text((x, y),       "eth0: ",  font=font, fill=255)

        # line 2
        draw.text((x, y+8),     "GPU:  ", font=font, fill=255)

        # line 3
        draw.text((x, y+16), "Line 3", font=font, fill=255)

        # line 4
        draw.text((x, y+25), "Line 4", font=font, fill=255)

        # Display image.
        # Set the SSD1306 image to the PIL image we have made, then dispaly
        disp.image(image)
        disp.display()
        # 1.0 = 1 second; The divisor is the desired updates (frames) per second
        time.sleep(1.0/4)

except KeyboardInterrupt:
    print("\nBye!")
    # Clear display.
    disp.clear()
    disp.display()
