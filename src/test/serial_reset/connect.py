#!../../../.env/bin/python
import sys, serial, termios

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

# Disable reset after hangup
with open(port) as f:
    attrs = termios.tcgetattr(f)
    attrs[2] = attrs[2] & ~termios.HUPCL
    termios.tcsetattr(f, termios.TCSAFLUSH, attrs)

# ser = serial.Serial()
# ser.baudrate = 57600
# ser.port = port
# ser.parity=serial.PARITY_NONE
# ser.stopbits=serial.STOPBITS_ONE
# ser.bytesize=serial.EIGHTBITS
# ser.dsrdtr = False
# ser.open()

ser = serial.Serial(
    #"/dev/ttyACM0",
    port,
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    dsrdtr=False,
)

try:
    while True:
        print ser.readline().strip()
except KeyboardInterrupt:
    #ser.close()
    pass
