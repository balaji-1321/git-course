import serial
import binascii
import time

PORT = "COM5"            # <<< Change this to your COM port
BAUD = 115200            # Must match STM32 UART baudrate
BIN_FILE = "app.bin"     # Your binary file name

ser = serial.Serial(PORT, BAUD, timeout=1)

# Read .bin file
with open(BIN_FILE, "rb") as f:
    data = f.read()

# Compute CRC32
crc = binascii.crc32(data) & 0xFFFFFFFF

print("CRC32 =", hex(crc))

# Send file data
ser.write(data)
time.sleep(0.1)

# Send 4-byte CRC (little endian)
ser.write(crc.to_bytes(4, byteorder="little"))

print("File + CRC sent")
ser.close()
