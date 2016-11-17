#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math as m
import numpy as np
import random as rd

class Ray:
	def __init__(self,value=None, index=None):
		self.value = value
		self.index = index
		self.angle = None						# Angle with the mid ray

class Robot:	# This class is mainly used to storing the "t-1" state of the robot
	def __init__(self, pub, dist, vel, dir, k1, kangle, data=None, infinity=None):
		self.direction = 1						# On which side was the last wall that robot was following (r:1/l:-1)
		self.distance = 0					# Current distance
		self.dist_prev = 0
		self.min_dist = dist 					# Distance that we want the robot to be from the wall
		self.velx = vel
		self.angle = 0							# Angle between robot and wall
		self.angle_prev = 0
		self.pub = pub
		self.data = data
		self.start = True
		self.ray_right = None
		self.ray_left = None
		self.min_ray = None
		self.ranges = None
		self.size = None
		self.kangle = kangle
		self.infinity = infinity				# Value for not taking as a minimum the nan values (out of range rays) 
		self.is_blind = True
		self.following = False					# True if the robot was fallowing a wall before getting blind
		self.pi = 3.1415926
		self.new_ray = None
		self.temp = None
		self.error = 0
		self.wallShape = 0.45					# Shape of the wall and laser position error
		self.min_angle = None					# Min distance between two rays to calculate robot angle 
		self.k1 = k1							# Allows to set the impact of the angle between robot and wall on the angular speed 
		
	def callback(self,data):
		self.data = data
		self.ranges = data.ranges
		self.size = len(self.ranges)
		self.angle_prev = self.angle
		self.setMinRays()						# Finding min length rays of each size of sensor data
		aux = self.direction
		self.setDirection()
		if self.is_blind: 						# Robot didn't recieved useful info by sensors
			self.blind()
		else:
			self.notBlind()
		self.is_blind = True
		
	def blind(self):
		msg = Twist()
		if self.following:
			msg.linear.x = self.velx #NUEVO		# Proporcional to the distance to the wall for making the robot moves forward when it stops 
												# seeing the wall. If it was far away from the wall, it's going to get is_blindbefore than if 
												# it was closer
			print("BLIND: ",self.temp)
			interval = self.distance / 15*self.velx
			self.temp = self.temp - interval
			if self.temp <= 0:
				self.following = False
		else:
			if self.start:
				msg.linear.x = rd.uniform(-1,1)
				msg.angular.z = rd.uniform(-1,1)
				self.start = False
			else:	
				msg.angular.z = -1 * self.direction * self.kangle
		#msg.angular.z = 0 #NUEVO
		self.pub.publish(msg)

	def notBlind(self):
		if self.min_ray.index in [0,self.size-1]: 	# If the shortest ray is a border ray the robot is close to be aligned with the wall
			self.follow()						# Calculate angle between robot and wall
		else:									# If not, the robot has to turn fast
			self.avoid()						# Calculate angle between robot and wall

	def avoid(self):
		self.following = True
		self.min_ray.angle = self.getAngle(self.size/2,self.min_ray.index)  # Angle between center ray and perpendicular ray to the wall
		self.angle = self.direction*(self.pi/2 - self.direction * self.min_ray.angle)	# Angle between robot and wall
		self.react()

	def follow(self):
		self.new_ray = Ray()
		self.new_ray.index = self.getRay()
		if self.new_ray.index is None:
			self.blind()
		else:
			if not m.isnan(self.ranges[self.new_ray.index]):
				self.new_ray.value = self.ranges[self.new_ray.index]
				self.following = True
				ray = self.new_ray.value
				alpha = self.getAngle(self.new_ray.index,self.min_ray.index)
				side = m.sqrt(self.min_ray.value**2+ray**2-2*self.min_ray.value*ray*m.cos(abs(alpha)))
				eps = m.asin(self.min_ray.value*m.sin(abs(alpha))/side)
				beta = abs(self.getAngle(self.size/2,self.new_ray.index))	# Angle between the second ray and parallel ray to the robot
				self.angle = self.direction * abs(eps - beta)
				self.react()
			else:
				self.blind()

	def react(self):
		self.dist_prev = self.distance 		# Storing previous distance
		self.distance = self.getDistance()
		self.temp = self.distance
		mid_ray = self.ranges[self.size/2]
		msg = Twist()
		print("Min ray ",self.min_ray.value,self.min_ray.index)
		if self.min_dist <= self.distance:
			self.error = -1 * (self.distance - self.min_dist)
			if self.error > -1:
				msg.angular.z = (self.kangle*(self.angle) + self.direction * 
								(self.k1 * self.error))
			else:
				msg.angular.z = (self.kangle*(self.angle) + self.direction * 
								(self.k1 * self.error) / self.distance)
		else:
			self.error = self.min_dist - self.distance
			if self.error < 1:
				msg.angular.z = (self.kangle*(self.angle) + self.direction * 
								(self.k1 * self.error))
			else:
				msg.angular.z = (self.kangle*(self.angle) + self.direction * 
								(self.k1 * self.error) / self.min_dist)
		self.mid_ray = self.ranges[self.size/2]
		if m.isnan(self.mid_ray):
			self.mid_ray = self.infinity
		if self.mid_ray < self.distance:
		  msg.linear.x = 0
		elif self.mid_ray < self.distance * 2:
		  msg.linear.x = 0.5*self.velx
		else:
		  msg.linear.x = self.velx
		self.pub.publish(msg)

	def setMinRays(self):
		self.min_ray = Ray(index=0)
		for i in range(0,self.size/2):
			ray = self.ranges[i]
			if m.isnan(self.ranges[self.min_ray.index]):
				self.min_ray.index = i
			elif not m.isnan(ray):
				if ray <= self.ranges[self.min_ray.index] and ray > 0.05:				# To discard possible errors
					self.min_ray.index = i
		self.min_ray.value = self.ranges[self.min_ray.index]
		if not m.isnan(self.min_ray.value):
			self.is_blind = False

	def getRay(self):										# Returns the index of a ray that will be used for calculating wall angle
		self.min_angle = 5
		n = self.min_ray.index								# For going over the list of rays starting on the last last or first element
		index = None
		for i in range(0,self.size/2):						# depending on the case
			if not m.isnan(self.ranges[abs(n-i)]) and self.min_angle <= 0:
				index = abs(n-i)
			self.min_angle = self.min_angle - 1				# This avoids to calculate W using two rays that are two close
		return index

	def getDistance(self):
		if self.min_ray.index in [0,self.size-1]:			# Then min_ray is NOT the distance to the wall
			return m.cos(self.pi/3) * self.min_ray.value - self.wallShape
		else:												# The min_ray is the distance to the wall
			return self.min_ray.value - self.wallShape

	def getAngle(self,pos1,pos2):							# Returns the angle between two rays by using their position in the ray list
		return (pos1 - pos2)*self.data.angle_increment

	def setDirection(self):
		if self.min_ray.index >= self.size/2:				# Robot is closer to a wall on its left
			self.direction = -1
		else:												# Robot is closer to a wall on its right
			self.direction = 1

if __name__ == '__main__':
	try:
		dist = 0.5	# Distance to the wall from the first ray that reaches it that usually is the angle_min or angle_max ray 
					# Taking into account this, the real distance could be calculated using the angle pi/2 - [angle_min, angle_max]
		vel = 0.5
		kangle = 1
		e = 0
		k1 = 0.5
		rospy.init_node('omap', anonymous=True)
		pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=100)
		wf = Robot(pub, dist, vel, dir, k1, kangle, infinity=float("inf")) # Set distance to wall changing wall distance parameter
		rospy.Subscriber("/scan", LaserScan, wf.callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


"""
* Hay que ponerlo en el centro de un cuadrado de cualquier tamano?
* Hice cambios en las laminas (agregue lamina de coclusiones)
* Hablarle de la primera idea con angulos, cosenos y arcos
* OBTENER EL MINIMO DE LA OTRA MITAD DE LOS RAYOS Y TOMARLA EN CUENTA PARA LOS GIROS. PUEDE SOLUCIONAR EL PROBLEMA DE LAS ESQUINAS
Comparar minimo del lado contrario con el angulo del centro y decidir si usar este como new_ray
"""
