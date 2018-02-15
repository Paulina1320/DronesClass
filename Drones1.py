#Class name: Drones
#Author: Paulina Ortega
#Connects to dronekit and attracts data
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

#The drone will begin to take off, but first it makes a process to check if the conditions to fly
def arm_and_takeoff(TargetAltitude):
	print ("Executing Takeoff")

	while not drone.is_armable:
		print ("Vehicle is not armable, waiting...")
		time.sleep(1)

	print("Ready to arm")
	#The GUIDE mode is activated to take off
	drone.mode = VehicleMode ("GUIDED")
	drone.armed =True

	while not drone.armed:
		print ("Waiting for arming...")
	
		time.sleep(1)

	print("Ready for takeoff, taking off...")
	drone.simple_takeoff(TargetAltitude)
d
	while True:
		Altitude = drone.location.global_relative_frame.alt
		print("Altitude: ", Altitude)
		time.sleep(1)

		if Altitude >= TargetAltitude * 0.95:
				print("Altitude reached")
				break
	
	
#Vehicle Connection
drone = connect('127.0.0.1:14551', wait_ready=True)
arm_and_takeoff(20)
#A defined speed is stipulated
drone.airspeed = 10 
#The address of the drone is stipulated in the dronekit terminals with the home commands
#The specific points to which the drone will fly are programmed through the coordinates of the Local Global Relative
#With time.sleep the time required to get from one point to another is approaching
a_location = LocationGlobalRelative(20.736212, -103.456114,20) 
drone.simple_goto(a_location) 

time.sleep(12)

a_location = LocationGlobalRelative(20.736237, -103.456639,20)
drone.simple_goto(a_location)

time.sleep(20)

a_location = LocationGlobalRelative(20.735529, -103.456709,20)
drone.simple_goto(a_location)

time.sleep(15)

a_location = LocationGlobalRelative(20.735507, -103.456191,20)
drone.simple_goto(a_location)

time.sleep(20)

#Change the vehicle mode to RTL so that the drone makes a landing from its starting point
drone.mode = VehicleMode ("RTL")
#By means of the battery voltage, the drone.battery.voltage command is used to print the battery voltage
print " Battery Voltage: %s v" % drone.battery.voltage
