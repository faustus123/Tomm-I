import board
import digitalio
import busio

print("Hello Blinka!")

# Try creating a digital input/output
pin = digitalio.DigitalInOut(board.D4)
print("Digital IO ok!")

# Try creating an I2C device
i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok!")

# Try creating an SPI device
spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
print("SPI ok!")

print("done!")