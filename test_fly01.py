#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
 Copyright 2015-2016, 3D Robotics.
channel_overrides.py: 
Demonstrates how set and clear channel-override information.
# NOTE: 
Channel overrides (a.k.a "RC overrides") are highly discommended (they are primarily implemented 
for simulating user input and when implementing certain types of joystick control).
"""
from dronekit import connect

# 连接到SITL

vehicle = connect('/dev/myusb1', baud = 115200, wait_ready = True)
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative



def arm_and_takeoff():
    print("Basic pre-arm checks")
    print("Arming motors")
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
 
    # 发送起飞指令
    time.sleep(1)
    print("Taking off!")



#vehicle.channels.overrides = {'5':1144}
#try:
#	while True:
#	# 显示全部通道的读数
#		print( "Read channels individually:")
#		print( " Ch1: %s" % vehicle.channels['1'])
#		print( " Ch2: %s" % vehicle.channels['2'])
#		print (" Ch3: %s" % vehicle.channels['3'])
#		print( " Ch4: %s" % vehicle.channels['4'])
#		print( " Ch5: %s" % vehicle.channels['5'])
#		print( " Ch6: %s" % vehicle.channels['6'])
#		print( " Ch7: %s" % vehicle.channels['7'])
#		print( " Ch8: %s" % vehicle.channels['8'])
#		print( "Number of channels: %s" % len(vehicle.channels))
#		time.sleep(0.5)
#except:
#	pass
MAX_CH=525
def contrel_ch(key):
	begin_ch= vehicle.channels[key]
	for i in range(200,MAX_CH,50):
		vehicle.armed = True
		vehicle.channels.overrides = {key:begin_ch+i}
		time.sleep(1)
		print (f" Ch{key}: %s" % vehicle.channels[key])
		time.sleep(1)
	vehicle.channels.overrides = {key:begin_ch+MAX_CH}
	print (f"====")
	time.sleep(2)
	vehicle.channels.overrides = {key:begin_ch+MAX_CH}
	print (f"====")
	time.sleep(2)
#	vehicle.channels.overrides = {key:begin_ch+MAX_CH}
#	print (f"====")
#	time.sleep(2)
	for i in range(MAX_CH,200,-50):
		vehicle.armed = True
		vehicle.channels.overrides = {key:begin_ch+i}
		time.sleep(1)
		print (f" Ch{key}: %s" % vehicle.channels[key])
		time.sleep(1)




print( "Read channels individually:")
print( " Ch1: %s" % vehicle.channels['1'])
print( " Ch2: %s" % vehicle.channels['2'])
print (" Ch3: %s" % vehicle.channels['3'])
print( " Ch4: %s" % vehicle.channels['4'])
print( " Ch5: %s" % vehicle.channels['5'])
print( " Ch6: %s" % vehicle.channels['6'])
print( " Ch7: %s" % vehicle.channels['7'])
print( " Ch8: %s" % vehicle.channels['8'])
print( "Number of channels: %s" % len(vehicle.channels))
time.sleep(0.5)

vehicle.mode = VehicleMode("LOITER")
print( "Mode: %s" % vehicle.mode.name)
#arm_and_takeoff()
#contrel_ch('3')

#
#
#
#
#vehicle.disarmed = False


vehicle.close()

print("Completed")
