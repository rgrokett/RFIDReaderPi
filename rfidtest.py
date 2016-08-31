import serial

# For PiB+ 2, Zero use:
#ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

# For Pi3 use
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

while True:
    string = ser.read(12)

    if len(string) == 0:
        print "Please wave a tag"

    else:
        string = string[1:11]   # Strip header/trailer
        print "string",string
        if string == 'E0043DBB52':
            print "hello Joe, what do you know?"


