#!/usr/bin/python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from arena_scan_planner.srv import ArenaScanTrajectory
from mrs_msgs.msg import Reference
import math
import os
import copy
import numpy as np
import dubins
import yaml
from os.path import expanduser

user_home = expanduser("~")
this_script_path = os.path.dirname(__file__)

DESIRED_SPEED = 3.0;
DESIRED_MAX_ACC = 2.0;
TURNING_SPEED = 1.5;
DESIRED_ALTITUDE = 4.5;
DESIRED_T_SAMPLING = 0.2 #SAMPLE_DIST/DESIRED_SPEED;
NUM_ROBOTS = 1
INITIAL_SPEED = 0 #DESIRED_SPEED;
WORLD_FILE = "wall_arena"
ARENA_CONFIG = os.path.join(user_home,"mrs_workspace/src/uav_core/ros_packages/mrs_general/config/world_"+WORLD_FILE+".yaml")

LOOP = False;

SERVICE_NAME='/uav1/arena_scan_planner/plan'

def dist_euclidean_squared(coord1, coord2):
    """ euclidean distance between coord1 and coord2"""
    (x1, y1) = coord1
    (x2, y2) = coord2
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

def dist_euclidean(coord1, coord2):
    """ euclidean distance between coord1 and coord2"""
    return math.sqrt(dist_euclidean_squared((coord1.x,coord1.y), (coord2.x,coord2.y)))

def read_world_file_arena(filename):
    arena_corners = []
    with open(filename, 'r') as f:
        data_loaded = yaml.safe_load(f)
        for idx in range(len(data_loaded['safety_area']['safety_area'])):
            loc = data_loaded['safety_area']['safety_area'][idx]
            if idx % 2 == 0:
                arena_corners.append([])
            arena_corners[-1].append(loc)
        return arena_corners   

def plot_velocity_profile(samples, color='k'):
    TIME_SAMPLE = 0.2
    figsize = (8, 5)
    fig = plt.figure(figsize=figsize)
    ax = fig.gca()
    ax.set_title('Velocity profile')
    ax.set_ylabel('velocity [m/s]')
    ax.set_xlabel('time [s]')

    #velocities = [INIT_VELOCITY]
    velocities = []
    for i in range(1, len(samples)):
        dist = dist_euclidean(samples[i-1], samples[i])
        if i == 1:
            velocities.append(dist / TIME_SAMPLE)
        velocities.append(dist / TIME_SAMPLE)
    velocities_time = [TIME_SAMPLE * i for i in range(len(velocities))]

    accelerations = [0]
    for i in range(1, len(velocities)):
        vel_change = velocities[i] - velocities[i - 1]
        accelerations.append(vel_change / TIME_SAMPLE)

    accelerations_time = [TIME_SAMPLE * i for i in range(len(accelerations))]

    plt.axhline(DESIRED_SPEED, 0, len(velocities), ls='-', color='k')
    plt.plot(velocities_time, velocities, '-', color=color, label='velocity')
    plt.axhline(DESIRED_MAX_ACC, 0, len(accelerations), ls='-.', color='k')
    plt.axhline(-DESIRED_MAX_ACC, 0, len(accelerations), ls='-.', color='k')
    plt.plot(accelerations_time, accelerations, '-.', color=color, label='acc')
    ax.legend(loc='upper right')

def test_trajectories():
    print("wait_for_service "+SERVICE_NAME)
    rospy.wait_for_service(SERVICE_NAME)
    """"
    float32 speed
    float32 acc
    float32 altitude
    float32 time_sample_period
    int32 robot_id
    ---
    bool ok
    geometry_msgs/PoseArray sweep_path
    """
        
    arena_corners = read_world_file_arena(os.path.join(this_script_path, ARENA_CONFIG))
    

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_ylabel('x [m]')
    ax.set_xlabel('y [m]')
    
    #plt.plot([arena_corners[i][0] for i in range(-1, len(arena_corners))], [arena_corners[i][1] for i in range(-1, len(arena_corners))],[0 for i in range(-1, len(arena_corners))], 'c-',label='fly area')
    plt.plot([arena_corners[i][0] for i in range(-1, len(arena_corners))], [arena_corners[i][1] for i in range(-1, len(arena_corners))], 'c-',label='fly area')
 

    
    #plt.axis('equal')
    for done_id in range(1,NUM_ROBOTS+1):
        try:
            sweep_planner = rospy.ServiceProxy(SERVICE_NAME, ArenaScanTrajectory)
            result = sweep_planner(DESIRED_SPEED, DESIRED_MAX_ACC,TURNING_SPEED, DESIRED_ALTITUDE, INITIAL_SPEED, LOOP, done_id,NUM_ROBOTS)
            # print(result.sweep_path)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        
        # print(result.sweep_path)
        # help(result.sweep_path) 
        x_positions = []
        y_positions = []
        z_positions = []
        yaw_positions = []
        x_dirs = []
        y_dirs = []
        z_dirs = []
        arrow_size = 0.05
        
        print("points for robot",done_id)
        for position in result.sweep_path:
            print("-------------------")
            print(position.x)
            print(position.y) 
            print(position.z)
            print(position.yaw)
            x_positions.append(position.x)
            y_positions.append(position.y)
            z_positions.append(position.z)
            yaw_positions.append(position.yaw)
            x_dirs.append(arrow_size*math.cos(position.yaw))
            y_dirs.append(arrow_size*math.sin(position.yaw))
            z_dirs.append(0)
    
        print("plot for robot %d" % done_id)
        print("first point for robot %d is %f %f %f %f" % (done_id, x_positions[0],y_positions[0],z_positions[0], yaw_positions[0]))
        z_positions = [0 for z in z_positions]
        every = 4 #plot only every n-th position
        #ax.scatter(x_positions , y_positions, z_positions)
        #ax.quiver(x_positions[0::every] , y_positions[0::every], z_positions[0::every],x_dirs[0::every],y_dirs[0::every],z_dirs[0::every])
        ax.quiver(x_positions[0::every] , y_positions[0::every],x_dirs[0::every],y_dirs[0::every],width=0.003)

        ax.plot([x_positions[0]],[y_positions[0]],'ro')
        ax.plot(x_positions[:],y_positions[:],'-g')

        plt.axis('equal')

        #ax.set_zlim3d(0, 2*DESIRED_ALTITUDE) 

        plot_velocity_profile(result.sweep_path)
    plt.show()
    
    
if __name__ == "__main__":
    try:
        test_trajectories() 
    except rospy.ROSInterruptException:
        pass
