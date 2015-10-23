# -*- coding:utf-8 -*-
"""
import pygame
from pygame.locals import *
from sys import exit
"""

import time
import serial

#serial_setup
imu = serial.Serial('/dev/tty.usbmodem1412' , 115200)
#pygame.init()

def main():

	while True:
		"""
		for event in pygame.event.get():
			if event.type == QUIT:
				pygame.quit()
				imu.close()
				exit()
"""

		data = imu.readline().rstrip()
		print data



if __name__ == "__main__":
    main()