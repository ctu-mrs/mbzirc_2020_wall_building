# wall.txt contains:
# x y z 

#!!!!x y z are of center position

# BRICK
import numpy as np
from math import cos, sin
import matplotlib as mpl
import matplotlib.pyplot as plt
import copy
import yaml
import os, sys
from mpl_toolkits.mplot3d import axes3d, Axes3D  # <-- Note the capitalization! 
import os, sys
from sympy.logic.boolalg import false
wall_utils_path = os.path.dirname(__file__)

config_file = os.path.join(wall_utils_path , "../config/planner.yaml")
with open(config_file, 'r') as f:
    loaded_config = yaml.safe_load(f)
    print("using default params from config file", config_file, ":", loaded_config)
    
['NO_BRICK']
['RED_BRICK']
['GREEN_BRICK']
['BLUE_BRICK']
['ORANGE_BRICK']

loaded_config['brick_ids']
loaded_config['brick_rewards']
loaded_config['brick_durations']
loaded_config['brick_num_robot_requirements']
loaded_config['brick_reservoir_time_from_wall_s']

#RED_BRICK_SIZE_CM = [30, 20, 20]
#GREEN_BRICK_SIZE_CM = [60, 20, 20]
#BLUE_BRICK_SIZE_CM = [120, 20, 20]
#ORANGE_BRICK_SIZE_CM = [180, 20, 20]

BRICK_TYPES = {1:"R",
               2:"G",
               3:"B",
               4:"O"}


BRICK_COLOR_IDS = {"R":1,
                   "G":2,
                   "B":3,
                   "O":4}


ROW_BRICK_PROBABILITIES = {0:{1: 0.5, 2: 0.5, 3: 0, 4: 0},
                           1:{1: 0.3, 2: 0.3, 3: 0.2, 4: 0.2},
                           2:{1: 0.25, 2: 0.25, 3: 0.25, 4: 0.25}, }

BRICK_TYPES_RGB_COLOR = {1:[1.0, 0.0, 0.0, 1],
                         2:[0.0, 1.0, 0.0, 1],
                         3:[0.0, 0.0, 1.0, 1],
                         4:[1.0, 0.5, 0.0, 1]}


BRICK_TYPE_SIZES_CM = {1: loaded_config['brick_size_length_depth_height_cm']['RED_BRICK'],
                       2: loaded_config['brick_size_length_depth_height_cm']['GREEN_BRICK'],
                       3: loaded_config['brick_size_length_depth_height_cm']['BLUE_BRICK'],
                       4: loaded_config['brick_size_length_depth_height_cm']['ORANGE_BRICK']}

BRICK_REWARDS = {1: loaded_config['brick_rewards']['RED_BRICK'],
                 2: loaded_config['brick_rewards']['GREEN_BRICK'],
                 3: loaded_config['brick_rewards']['BLUE_BRICK'],
                 4: loaded_config['brick_rewards']['ORANGE_BRICK']}

BRICK_DURATIONS_S = {1: loaded_config['brick_durations']['RED_BRICK'],
                     2: loaded_config['brick_durations']['GREEN_BRICK'],
                     3: loaded_config['brick_durations']['BLUE_BRICK'],
                     4: loaded_config['brick_durations']['ORANGE_BRICK'] }

BRICK_NUM_ROBOT_REQUIREMENTS = {1:loaded_config['brick_num_robot_requirements']['RED_BRICK'],
                                2:loaded_config['brick_num_robot_requirements']['GREEN_BRICK'],
                                3:loaded_config['brick_num_robot_requirements']['BLUE_BRICK'],
                                4:loaded_config['brick_num_robot_requirements']['ORANGE_BRICK'] }

wall_1_pos = [-4.242,  2.212, -0.785]
wall_2_pos = [-1.410,  2.000,  0.785]
wall_3_pos = [ 1.410,  2.212, -0.785]
wall_4_pos = [ 4.242,  2.000,  0.785]
WALLS = [wall_1_pos,wall_2_pos,wall_3_pos,wall_4_pos]

LAYER_LEN = 4.0
BRICK_LAYER_LEN = 3.6
BRICK_DISPLACEMENT = (LAYER_LEN - BRICK_LAYER_LEN)/7.0
WALL_HEIGHT = 1.7

def wall_definition_to_3d_popsitions(wall_definition):
    wall_xyz_yaw = []
    
    for layer in wall_definition:
        for channel in wall_definition[layer]:
            
            channel_middle = WALLS[channel-1]
            ch_yaw = WALLS[channel-1][2]
            num_brick = 0
            next_brick_start = -LAYER_LEN/2.0 + BRICK_DISPLACEMENT/2.0
            print("next_brick_start",next_brick_start)
            for brick in wall_definition[layer][channel]:
                brick_color = brick
                brick_type = BRICK_COLOR_IDS[brick_color]
                brick_len = BRICK_TYPE_SIZES_CM[brick_type][0]
                brick_height = BRICK_TYPE_SIZES_CM[brick_type][2]
                
                print("brick_len",brick_len)
                pos_x = next_brick_start + brick_len/200.0
                next_brick_start += brick_len/100.0 + BRICK_DISPLACEMENT
                print("next_brick_start",next_brick_start)
                pos_y = 0
                pos_z = WALL_HEIGHT + brick_height/200.0 + (layer-1)*brick_height/100.0
                pos_x_rot = pos_x * cos(ch_yaw) - pos_y * sin(ch_yaw) + channel_middle[0]
                pos_y_rot = pos_x * sin(ch_yaw) + pos_y * cos(ch_yaw) + channel_middle[1]
                
                wall_xyz_yaw.append([brick_type,[pos_x_rot,pos_y_rot,pos_z,ch_yaw]]) 
                #print(pos_x,pos_y)
                num_brick += 1
            #break
        #break
    
    #quit()
    return wall_xyz_yaw
                
                
def plot_wall(wall,ax=None,show=True):
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
    for brick_id in range(len(wall)):
        print("brickid",brick_id,wall[brick_id],"type", wall[brick_id][0])
        plot_brick(ax, brick_id , wall[brick_id][0], wall[brick_id][1])
    
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(0, 2.2)
    ax.view_init(elev=14, azim=45)
    #ax.set_aspect('equal','box')
    if show:
        plt.show()


def plot_brick(ax,brick_id, type, position):
    if type == 0:
        return 
    
    edgeface_color = copy.deepcopy(BRICK_TYPES_RGB_COLOR[type])
    edgeface_color[-1] = 0.2
    edgeface_edgecolor = copy.deepcopy(BRICK_TYPES_RGB_COLOR[type])
    edgeface_edgecolor[-1] = 0.8
    
    x_diam = BRICK_TYPE_SIZES_CM[type][0] / 100.0
    y_diam = BRICK_TYPE_SIZES_CM[type][1] / 100.0
    z_diam = BRICK_TYPE_SIZES_CM[type][2] / 100.0
    x = position[0] 
    y = position[1] 
    z = position[2] 
    yaw = position[3]
    
    #print("y_diam",y_diam)
    
    vertices = [ [ x_diam / 2, y_diam / 2, z_diam / 2],
                 [ x_diam / 2, -y_diam / 2, z_diam / 2],
                 [-x_diam / 2, y_diam / 2, z_diam / 2],
                 [ -x_diam / 2, -y_diam / 2, z_diam / 2],
                 [ +x_diam / 2, y_diam / 2, -z_diam / 2],
                 [ +x_diam / 2, -y_diam / 2, -z_diam / 2],
                 [ -x_diam / 2, y_diam / 2, -z_diam / 2],
                 [ -x_diam / 2, -y_diam / 2, -z_diam / 2]]
    
    for v_id in range(len(vertices)):
        vert = copy.deepcopy(vertices[v_id])
        x_rot =  vert[0]*cos(yaw) - vert[1]*sin(yaw)
        y_rot =  vert[0]*sin(yaw) + vert[1]*cos(yaw)
        #print("vert rot",x_rot,y_rot)
        vertices[v_id][0] = x_rot + x
        vertices[v_id][1] = y_rot + y
        vertices[v_id][2] += z 
    
    print("vertices", vertices)
    
    triangles = [[0 , 1 , 2], [ 2, 1, 3],  # top
                 [4 , 5 , 6], [ 6, 5 , 7],  # bottom
                 [0 , 1 , 4], [ 4, 1 , 5],  # left
                 [2 , 3 , 6], [ 6, 3 , 7],  # right
                 [0 , 2 , 4], [ 4, 2 , 6],  # front
                 [1 , 3 , 5], [ 5, 3 , 7],  # rear
                 ]
    
    ax.text(x,y,z,str(brick_id), ha='center', va='center')
    #ax.plot([x], [y], [z], 'o', color=edgeface_color, markeredgecolor='black')
    ax.plot_trisurf([v[0] for v in vertices] , [v[1] for v in vertices], [v[2] for v in vertices], triangles=triangles, color=edgeface_color, edgecolor=edgeface_edgecolor, linewidth=0.2)

    
def save_wall_to_file(filename, wall):
    with open(filename, 'w', encoding='utf-8') as f:
        print("opened file", filename, "for saving wall")
        f.write("#brick_id brick_type brick_x_pos brick_y_pos brick_z_pos brick_yaw_rotated" + os.linesep)
        for brick_id in range(len(wall)):
            brick_type = wall[brick_id][0]
            x = wall[brick_id][1][0] / 100.0
            y = wall[brick_id][1][1] / 100.0
            z = wall[brick_id][1][2] / 100.0
            f.write(str(brick_id + 1) + " " + str(brick_type) + " " + "%.2f" % x + " " + "%.2f" % y + " " + "%.2f" % z + " " + str(wall[brick_id][1][3]) + os.linesep)

        
def load_wall_to_file(filename):
    wall = []
    
    start = [0, (5, 5, 0, 0)]
    wall.append(start)
    
    line_num = 0
    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            line_num += 1
            trimed_line = line.strip()
            if trimed_line[0] != "#":
                splited_line = trimed_line.split()
                if len(splited_line) != 6:
                    print("bad number of variables at line", line_num, "with:", line)
                    return wall
                else:
                    id = int(splited_line[0])
                    brick_type = int(splited_line[1])
                    x = float(splited_line[2]) * 100.0
                    y = float(splited_line[3]) * 100.0
                    z = float(splited_line[4]) * 100.0
                    rotated = bool(int(splited_line[5]))
                    
                    wall.append([brick_type, (x, y, z, rotated)])
    
    goal = [0, (5, 5, 0, 0)]
    wall.append(goal)
    
    
    print(wall)            
    return wall
          
