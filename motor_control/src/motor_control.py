#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    This is a driver program in ros to run the EX-06+ dynamixel motor 

    Angle Convention :

    + positive angle - CW direction 
    - negative angle - CCW direction

    #Author: Shashank Prasad (prasha@seas.upenn.edu)
"""




import rospy 
import os
import time 
from signal import signal, SIGINT
import sys 
import numpy as np


from dynamixel_sdk import *                    # Uses Dynamixel SDK library

##### Addresses for the control table for the motor ######## 

ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_OPERATION_MODE     = 11
ADDR_MX_CW_LIMIT_L         = 6
ADDR_MX_CW_LIMIT_H         = 7
ADDR_MX_CCW_LIMIT_L        = 8 
ADDR_MX_CCW_LIMIT_H        = 9
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_MOVING_SPEED_H     = 33
ADDR_MX_TORQUE_LIMIT       = 34 
ADDR_MX_PRESENT_SPEED      = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = "/dev/ttyUSB0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
DXL_MOVING_SPEED            = 80                # Dynamixel moving speed range 0-1023
DXL_MOVING_DIRECTION        = 1                 # 1 for CW and 0 for CCW Direction


class MotorControl:

	def __init__(self):
		signal(SIGINT,self.handler)
		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(PROTOCOL_VERSION)

		# Open port
		if self.portHandler.openPort():
			rospy.logdebug("Succeeded to open the port")
		else:
			rospy.logdebug("Failed to open the port")
			exit()
		self.init_speed = rospy.get_param('speed')
		self.current_speed = 0.0
		self.current_direction = 1



	def execute_2byte_command(self, command):
		# Enable wheel mode
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, command[0], command[1], command[2])
		if dxl_comm_result != COMM_SUCCESS:
			rospy.logdebug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			rospy.logdebug("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			rospy.logdebug("Executed command successfully!")		

	def enable_wheel_mode(self):
	    command = [1, ADDR_MX_CCW_LIMIT_L, 0]
	    self.execute_2byte_command(command)
	    command[1] = ADDR_MX_CW_LIMIT_L
	    command[2] = 0 
	    self.execute_2byte_command(command)
	    rospy.loginfo_once('Set operation mode to wheel mode')
	    return


	def initialize_motor(self):

		self.enable_wheel_mode()
		speed = (self.speed&0x3FF) | (DXL_MOVING_DIRECTION << 10)
		speed_command = [ DXL_ID, ADDR_MX_MOVING_SPEED, speed]
		self.execute_2byte_command(speed_command)
		self.current_speed = speed
		self.current_direction = DXL_MOVING_DIRECTION
		self.stop_motor()



	def start_motor(self):
	# Enable Dynamixel Torque
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			rospy.logdebug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			rospy.logdebug("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			rospy.logdebug("Torque enable has been set")

	def stop_motor(self):
    # # Disable Dynamixel Torque
	    dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
	    if dxl_comm_result != COMM_SUCCESS:
	        rospy.logdebug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
	    elif dxl_error != 0:
	        rospy.logdebug("%s" % self.packetHandler.getRxPacketError(dxl_error))



	def set_motor_dir_CW(self):
		self.stop_motor()
		rospy.logdebug('Setting direction to clockwise')
		speed = (DXL_MOVING_SPEED&0x3FF) | (1 << 10)
		speed_command = [ DXL_ID, ADDR_MX_MOVING_SPEED, speed]
		self.execute_2byte_command(speed_command)
		self.stop_motor()

	def set_motor_dir_CCW(self):
		self.stop_motor()
		rospy.logdebug('Setting direction counter clockwise')
		speed = (DXL_MOVING_SPEED&0x3FF) | (0 << 10)
		speed_command = [ DXL_ID, ADDR_MX_MOVING_SPEED, speed]
		self.execute_2byte_command(speed_command) 
		self.stop_motor()

	def read_2byte_command(self,address):
	#Reads two bytes of data from the address given 	
		dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, address)
		if dxl_comm_result != COMM_SUCCESS:
			rospy.logdebug("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			return -1 
		elif dxl_error != 0:
			rospy.logdebug("%s" % self.packetHandler.getRxPacketError(dxl_error))
			return -1
		return dxl_data


	def handler(self,signal_received, frame):
		self.stop_motor()
		# Close port
		rospy.loginfo('GoodBye!')
		self.portHandler.closePort()
		exit()



	def get_current_position(self):
	# According to data sheet the unit of measurement for this value is 0.06 degrees
	# But this is still position of the rotary encoder. 

	    return self.read_2byte_command(ADDR_MX_PRESENT_POSITION)*0.06 

	def get_rpm(self):
	#The approximation as per the datasheet is the 
	#Current rpm (last 10 bits of data / 9) 

		return (self.read_2byte_command(ADDR_MX_MOVING_SPEED)&0x3ff)/9.0

	def get_current_direction(self):
    # The 10 th bit of the motor speed gives the direction of the motor 
    #If 10 th bit is set to 1 - CW direction, else CCW drection

    #Returns: 1 for CW direction and 0 for CCW direction 

		self.current_speed = self.read_2byte_command(ADDR_MX_MOVING_SPEED)
		self.current_direction = 1 if (self.current_speed & 0x400)>0 else 0

		return self.current_direction


	def calculate_time(self,rpm, angle):
	#Given rpm and angle find the time to activate the motor
	# Found by experimentation that the angle needs to be scaled by:
	# CW - angle*1.58
	# CCW - angle*1.5275
	# THis is to account for losses due to motor error and friction
	# TODO: figure out the experimental values

		if angle > 0 :  
		    return (angle*1.58 / (rpm*360))*60
		else: 
		    return (angle*1.5275 / (rpm*360))*60



	def move_motor_angle(self,angle):
	#This is to move the motor a certain angle 
		self.get_current_direction()

		if self.current_direction == -1 :
		    print('Reading direction failed. Exiting!')
		    exit()


		rpm = self.get_rpm()
		rospy.logdebug('RPM of the motor {}'.format(rpm))

		if self.current_direction and angle < 0: 
			self.current_speed = self.set_motor_dir_CCW()


		if not self.current_direction and angle > 0:
			self.current_speed = self.set_motor_dir_CW()

		motor_time = self.calculate_time(rpm,abs(angle))
		rospy.logdebug('Calculated motor time {}'.format(motor_time))
		self.start_motor()
		time.sleep(motor_time)
		self.stop_motor()

def main():
	obj = MotorControl()
	rospy.init_node('motor_controller')
	while not rospy.is_shutdown():
		angle = input('Enter angle for motor +ve for CW -ve for CCW \n')
		obj.move_motor_angle(int(angle))


if __name__ == '__main__':
	main()












