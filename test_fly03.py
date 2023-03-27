from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('/dev/myusb1', baud = 115200, wait_ready = True)
def armed_test():
	print("Arming motors")
	# 将无人机的飞行模式切换成"GUIDED"（一般建议在GUIDED模式下控制无人机）
	vehicle.mode = VehicleMode("GUIDED")
	# 通过设置vehicle.armed状态变量为True，解锁无人机
	vehicle.armed = True
	time.sleep(1)
	vehicle.disarmed = True
try:
	while True:
		print(vehicle.location.global_relative_frame.alt)
		time.sleep(0.5)
except:
	pass

vehicle.close()