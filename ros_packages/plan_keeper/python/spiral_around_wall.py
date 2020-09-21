#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
from math import cos, sin, tan
import sys, copy

PLOT=True


NUM_ROTATIONS = 1.5
ROT_STEP = 0.2

START_RADIUS = 0
MAX_RADIUS = 1
RADIUS_STEP = (MAX_RADIUS-START_RADIUS) / (2.0 * math.pi * (NUM_ROTATIONS-1.0)/ROT_STEP)

start_direction = math.pi/2

def get_spiral_points(pos,start_radius, max_radius,radius_step ,start_direction, rot_step,num_rotations):
	rot = start_direction
	radius = start_radius
	#speed = len_step/duration_s
		
	num_points = (2.0*math.pi*num_rotations)/rot_step
	spiral_points = []

	if PLOT:
		plt.plot([pos[0]],[pos[0]],'og')

	for i in range(int(num_points)):
		
		radius += radius_step
		radius = min(radius,max_radius)
		radius = max(radius,start_radius)
		
		rot += rot_step

		posx = pos[0] + radius * cos(rot)
		posy = pos[1] + radius * sin(rot)
		spiral_points.append([posx,posy])
		
		if PLOT:
			plt.plot([posx],[posy],'.r')

	if PLOT:
		plt.axis('equal')
		plt.show()
		
	return spiral_points

if __name__ == "__main__":
	current_pos = [0,0,0]
	get_spiral_points(current_pos,START_RADIUS,MAX_RADIUS,RADIUS_STEP,start_direction,ROT_STEP,NUM_ROTATIONS)
	
	
	
	