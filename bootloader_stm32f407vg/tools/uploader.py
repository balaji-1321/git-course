
import serial, sys, time

ser = serial.Serial("COM3",115200,timeout=1)

def send(cmd):
    ser.write((cmd+"\r\n").encode())
    print(ser.readline().decode(), end='')

send("flash erase_update")

data = open("app.bin","rb").read()
offset = 0

for i in range(0,len(data),32):
    chunk = data[i:i+32]
    hexstr = chunk.hex()
    send(f"flash write_update {offset} {hexstr}")
    offset += 32

send("update commit")
print("Done. Reset device.")
