#!/usr/bin/env python3

from wall_utils import *
import random, os
import copy
from cmath import pi


WALL_FILENAME_UAV = "../config/wall_mbzirc.txt"

NUM_RANDOM_WALLS_UAV = 3
NUM_LAYERS_WALL_UAV = 2
NUM_ORANGE_WALLS_UAV = 1

#total brick size is 360cm
BRICK_STACK_RANDOM = ["R", "R", "R", "R", "G", "G", "B"]
BRICK_STACK_ORAGNE = ["O", "O"]

# wall is wall[layer][channel]

def get_wall_str(wall):
    """example:B G R R R R G"""
    wall_str = ""
    num_layers = 0
    
    layers = list(wall.keys())
    layers.sort(reverse=True)
    print("layers:",layers)
    for layer in layers:
        line = ""
        num_bricks = 0
        for channel in wall[layer]:
            for b in wall[layer][channel]:
                if b == "R" or b == "G" or b == "B":
                    if num_bricks > 0:
                        line += " "
                    line += b
                    num_bricks += 1
                
        if num_layers > 0:
            wall_str += os.linesep
        wall_str += line
        
        num_layers += 1
    return wall_str

def print_wall(wall):
    wall_str = get_wall_str(wall)
    print(wall_str)
    
def save_wall(filename,wall):
    wall_str = get_wall_str(wall)
    with open(WALL_FILENAME_UAV,'w') as f:
        f.write(wall_str)
    
def generate_wall_uav():    
    wall_definition = {}
    last_channel = 0
    
    
    for layer in range(1,NUM_LAYERS_WALL_UAV+1):
        for channel in range(1,NUM_RANDOM_WALLS_UAV+1):
            last_channel = channel
        
            
            bricks_fill = copy.deepcopy(BRICK_STACK_RANDOM)
            random.shuffle(bricks_fill)
            
            if wall_definition.get(layer) is None:
                wall_definition[layer] = {}
            wall_definition[layer][channel] = []
            
            for b in bricks_fill:
                wall_definition[layer][channel].append(b)
        
        for channel in range(last_channel+1,last_channel+NUM_ORANGE_WALLS_UAV+1):
            bricks_fill = copy.deepcopy(BRICK_STACK_ORAGNE)
            random.shuffle(bricks_fill)
            
            if wall_definition.get(layer) is None:
                wall_definition[layer] = {}
            wall_definition[layer][channel] = []
            
            for b in bricks_fill:
                wall_definition[layer][channel].append(b)
    
    return wall_definition
        
        
if __name__ == "__main__":
    wall_definition = generate_wall_uav()
    print_wall(wall_definition)
    definition3d = wall_definition_to_3d_popsitions(wall_definition)
    print(definition3d)
    save_wall(WALL_FILENAME_UAV, wall_definition)
    plot_wall(definition3d)
    


#
    


#save_wall_to_file(WALL_FILENAME, wall)
#plot_wall(wall)

