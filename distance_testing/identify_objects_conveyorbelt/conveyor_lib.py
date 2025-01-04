#!/usr/bin/env python
import RPi.GPIO as GPIO

channel = 17


class Conveyor:
    def __init__(self):
        # The script as below using BCM GPIO 00..nn numbers
        GPIO.setmode(GPIO.BCM)
        # Set relay pins as output
        GPIO.setup(channel, GPIO.OUT)
        
    def turn_on(self):
        #Turn on relay
        GPIO.output(channel, GPIO.HIGH)
        
    def turn_off(self):
        GPIO.output(channel, GPIO.LOW)



if __name__ == "__main__":
    conveyor = Conveyor()
    conveyor.turn_on()
#GPIO.output(channel, GPIO.LOW)
#GPIO.cleanup()
