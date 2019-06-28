#!/usr/bin/python
# 
# RFID READER 2
# version 1.0
# r.grokett
#
# Watches for RFID card.
# When detected, looks for ID# in CARDS list
# If ID# is found, turns on LED,buzzer,relay 
#
# Note that relay can be either GPIO driven type or I2C driven type
#
# THIS VERSION INCLUDES:
#   Get card IDs from external source (file /home/pi/cards.dat)
#   Add User name to card ID
#   Write Date/Time, Card ID, Name to log when detected
#   Restart program on Raspi boot up
#
# NOTE: Append below line to /etc/rc.local to start at boot 
#   /home/pi/RFIDReaderPi/reader2.py &

import smbus
import time 
import serial
import sys, signal, os, datetime
import RPi.GPIO as GPIO


# DEFAULTS
DEBUG = True    # Set to 'False' for normal operation
CARDFILE = "/home/pi/cards.dat"
LOGFILE  = "/home/pi/rfiduser.log"
ON  = 1
OFF = 0

# SET RELAY TYPE (default is GPIO)
I2C = OFF       # Turn this ON if using an I2C Relay
            # else it uses GPIO Relay

# Set up GPIO Pins (change as needed)
LED = 23
BUZZER  = 24
RELAY   = 25
BUTTON  = 4


# BEGIN INITIALIZATION
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(RELAY, GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Get I2C bus
bus = smbus.SMBus(1)
i2c_addr = 0x41     # I2C Address of Relay

# GO TO MAIN



############################
# METHODS
############################
# TURN ON/OFF LED
def led(value):
    if (DEBUG): 
        print("led()")
    if (value):
        GPIO.output(LED, GPIO.HIGH)
    if (DEBUG): 
        print("LED state is : HIGH")
    else:
        GPIO.output(LED, GPIO.LOW)
    if (DEBUG): 
        print("LED state is : LOW")

# TURN ON/OFF BUZZER
def buzzer(value):
    if (DEBUG): 
        print("buzzer()")
    if (value):
        GPIO.output(BUZZER, GPIO.HIGH)
    if (DEBUG): 
        print("BUZZER state is : HIGH")
    else:
        GPIO.output(BUZZER, GPIO.LOW)
    if (DEBUG): 
        print("BUZZER state is : LOW")

# TURN ON/OFF GPIO RELAY
def relay(value):
    if (DEBUG): 
        print("relay()")
    if (value):
        GPIO.output(RELAY, GPIO.HIGH)
    if (DEBUG): 
        print("RELAY state is : HIGH")
    else:
        GPIO.output(RELAY, GPIO.LOW)
    if (DEBUG): 
        print("RELAY state is : LOW")

# TURN ON/OFF I2C RELAY
def i2c_relay(value):
    if (DEBUG): 
        print("i2c_relay()")
    if not I2C:
        return

    if (value):
        # PCA9536_R11 address, i2c_addr(65)
        # Select output port register, 0x01(01)
        bus.write_byte_data(i2c_addr, 0x01, 0x01)
        if (DEBUG): 
            print("Relay-1 state is : HIGH")
    else:
        # PCA9536_R11 address, i2c_addr(65)
        # Select output port register, 0x01(01)
        bus.write_byte_data(i2c_addr, 0x01, 0x00)
        if (DEBUG): 
            print("Relay-1 state is : LOW")

# UNLOCK DOOR
def unlock_door():
    if (DEBUG): 
        print("unlock_door()")
    led(ON)
    buzzer(ON)
    relay(ON)
    i2c_relay(ON) 

# LOCK DOOR
def lock_door():
    if (DEBUG): 
        print("lock_door()")
    led(OFF)
    buzzer(OFF)
    relay(OFF)
    i2c_relay(OFF) 

# WRITE LOG
def write_log(id,name):
    if (DEBUG): 
        print("write_log()")
    f = open(LOGFILE, 'a')
    cur_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    log_entry = cur_time+" "+str(id)+" "+str(name)+"\n"
    f.write(log_entry)
    f.close()

# EXIT
def signal_handler(signal, frame):
  if (DEBUG): 
        print("Exiting!")
  write_log('00000000','Stop...' )
  relay(ON) # NOTE! UNLOCKS DOOR ON FAILURE/EXIT (Jurassic Park!)
  i2c_relay(ON)
  GPIO.cleanup()
  ser.close()
  sys.exit(0)


############################
# MAIN
############################
if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    # Select configuration register, 0x03(03)
    # 0x00(00)  Set all pins as OUTPUT
    if (I2C):
        bus.write_byte_data(i2c_addr, 0x03, 0x00)
    time.sleep(0.5) 

    ser = 0
    # Selects which version of Raspi you have 
    if (GPIO.RPI_REVISION == 3):
        # For Pi3 use
        if (DEBUG): 
            print("This is a Pi 3")
        ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    else:
        # For Pi2, B+, Zero use
        if (DEBUG): 
            print("This is a Pi B+, 2, or Zero")
        ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)


    # READ CARD LIST
    CARDS = dict()
    try:
      cnt = 0
      with open(CARDFILE) as lines:
        for line in lines:
          if '#' in line: 
            continue
          if '=' in line:
            key, value = line.split('=',1)
            CARDS[key] = value
            cnt = cnt + 1
          else:
            pass
        if (DEBUG): 
            print("Loaded "+str(cnt)+" cards")
    except:
        if (DEBUG): 
            print("Could not read ", CARDFILE)
        CARDS = {'00000000':'Dummy Card'}  # Set a dummy default    
        write_log('00000000','Start...' )

    # LOCK THE DOOR ON STARTUP
    lock_door()

    # MAIN LOOP
    while True:
        buffer = ser.read(12)
        
        if len(buffer) == 0:
          if (DEBUG): 
            print("Please wave a tag")
        else:
            card_id = buffer[1:11]  # Strip header/trailer
            if (DEBUG): 
                print("Card ID : ", card_id)
            if (card_id in dict.keys(CARDS)):
              if (DEBUG): 
                print("FOUND CARD ID: ", card_id)
              write_log(card_id,CARDS[card_id])
              unlock_door()
              time.sleep(5)
              lock_door()
            else:
              if (DEBUG): 
                print("CARD NOT IN LIST!")
              write_log(card_id,'NOT FOUND')

        # EXIT THE BUILDING
        # Add an indoor button
        if not GPIO.input(BUTTON):
            if (DEBUG): 
                print("Button Pressed")
            write_log('00000000','Someone Exited' )
            unlock_door()
            time.sleep(5)
            lock_door()
    
        time.sleep(0.1)
    # END WHILE
# END MAIN

