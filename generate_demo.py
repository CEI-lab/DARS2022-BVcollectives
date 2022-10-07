import numpy as np
import random
import pandas as pd
import time
import os

# custom imports
import utils
from robots import *

########################## PARAMETERS ###########################################

previous_time_test = time.time()

DEMO = 0

#variables for saving data
sim_data = []

tag = "curiosity"
type = "dir_dir"


global_filename = "configs/global_config.yaml"
c_filename = "configs/" + tag + ".yaml"
e_filename = "configs/env_config.yaml"
# metric = np.array(["nnd"])
metric = np.array([])

datadir = './data'
if not os.path.exists(datadir):
    os.mkdir(datadir)


############################### MAIN ##############################################

sim_time, ss, num = utils.load_global_config(global_filename)
if(DEMO):
    obstacles = utils.load_environment_config(e_filename)

# init robots
robots = Robots(global_filename, c_filename, type, ss, DEMO)

group_list = [np.zeros(robots.num)]

# sim loop
for t in range(sim_time):
    # print(robots.lights)

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    for r in range(robots.num):
        c = robots.coords[r].copy() # save previous position in case of collision
        # update robot positions
        robots.update_movement(r, robots.noise_factor)

        # returns true if there was a collision
        # and moves robot back to original position and reorients
        if(DEMO):
            retval = robots.check_collision(r, obstacles, c)

        # update light
        return_data = robots.update_light(r, robots.num, robots.coords[r], big_coords.copy(), big_angles.copy(), big_lights.copy(), group_list, metric, robots.lim_angle, robots.lim_distance, robots.influence_scale, robots.split)
    
    sim_data.append([robots.coords[:,0].copy(), robots.coords[:,1].copy(), robots.angles.copy(), robots.lights.copy()])
    # print(robots.lights)


data = pd.DataFrame(data = sim_data, columns = ["x","y","theta","light"])

outdat = os.path.join(datadir, "demo.csv")

data.to_csv(outdat, line_terminator = "")
