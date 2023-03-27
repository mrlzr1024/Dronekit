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

from dronekit import connect, VehicleMode
from pymavlink import mavutil
# 连接到SITL
print("连接中")
vehicle = connect('/dev/myusb1', baud = 115200, wait_ready = True)
print("连接成功")
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative



def arm_and_takeoff():
    print("Basic pre-arm checks")
    print("Arming motors")
#    vehicle.mode = VehicleMode("ALT_HOLD")
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



def dispaly_info():
	print( "Read channels individually:")
	print( " Ch1: %s" % vehicle.channels['1'])
	print( " Ch2: %s" % vehicle.channels['2'])
	print (" Ch3: <===== %s" % vehicle.channels['3'])
	print( " Ch4: %s" % vehicle.channels['4'])
	print( " Ch5: %s" % vehicle.channels['5'])
	print( " Ch6: %s" % vehicle.channels['6'])
	print( " Ch7: %s" % vehicle.channels['7'])
	print( " Ch8: %s" % vehicle.channels['8'])
	print( "Number of channels: %s" % len(vehicle.channels))
	vx = vehicle.velocity[0]  # 无人机在x轴方向上的速度
	vy = vehicle.velocity[1]  # 无人机在y轴方向上的速度
	vz = vehicle.velocity[2]  # 无人机在z轴方向上的速度
	time.sleep(0.1)
	print("当前速度：vx = %.2f, vy = %.2f, vz = %.1f" % (vx, vy, vz))
	print("当前高度: %s m"%vehicle.location.global_relative_frame.alt)
	print("电池电量:%s"%vehicle.battery)



	
MAX_CH=580
def contrel_ch(key,alt):
	begin_ch = vehicle.channels[key]
	begin_alt = vehicle.location.global_relative_frame.alt
	# ===== 起飞 =============== #
	for i in range(300,MAX_CH,200):
		vehicle.armed = True
		vehicle.channels.overrides = {key:begin_ch+i}
		dispaly_info()
		time.sleep(1)
		print (f" Ch{key}: %s" % vehicle.channels[key])
	vehicle.channels.overrides = {key:begin_ch+MAX_CH}
	print (f"====")
	dispaly_info()
#	time.sleep(2)
	curr_alt = vehicle.location.global_relative_frame.alt
	print(f"====      ====>   alt= {curr_alt-begin_alt} set to alt = {alt *0.9}")
	try :
		while curr_alt-begin_alt < alt *0.9:
			curr_alt = vehicle.location.global_relative_frame.alt
			print(f"alt= {curr_alt-begin_alt} set to alt = {alt *0.9}")
			vehicle.channels.overrides = {key:begin_ch+MAX_CH}
			time.sleep(0.5)
		print(f"alt= {curr_alt-begin_alt} set to alt = {alt *0.9}")
	# ===== 悬停 ================== #
		while True:
			vehicle.channels.overrides = {key:1500}
			print (f"====") 
			dispaly_info()
			time.sleep(2)
	except:
		pass
#	vehicle.channels.overrides = {key:begin_ch+MAX_CH}
#	print (f"====")
#	time.sleep(2)
	# ===== 降落 ================== #
	print("正在降落....")
	for i in range(1500,200,-200):
		vehicle.armed = True
		vehicle.channels.overrides = {key:begin_ch+i}
		time.sleep(1)
		print (f" Ch{key}: %s" % vehicle.channels[key])








time.sleep(0.5)

#vehicle.mode = VehicleMode("LOITER")
print( "Mode: %s" % vehicle.mode.name)


arm_and_takeoff()
contrel_ch('3',1.3)

#
#
#
#
#vehicle.disarmed = False


vehicle.close()

print("Completed")
