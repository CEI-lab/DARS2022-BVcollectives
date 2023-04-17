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

tag = "aggression"
type = "dir_omni"


global_filename = "configs/global_config.yaml"
c_filename = "configs/" + tag + ".yaml"
e_filename = "configs/env_config.yaml"
metric = np.array([]) # leaving for future integration w/ QD algorithms

datadir = './data'
if not os.path.exists(datadir):
    os.mkdir(datadir)


############################### MAIN ##############################################

### Load Configs

gparams = utils.load_config(global_filename)
sim_time, ss = gparams["sim_time"], gparams["screen_size"]

if(DEMO):
    dparams = utils.load_config(e_filename)
    obstacles = dparams["obstacles"]

robots = Robots(global_filename, c_filename, type, DEMO)

group_list = [np.zeros(robots.num)]

### Simulation Loop

for t in range(sim_time):

    big_coords, big_stimuli, big_angles = utils.setup_big_arrays(robots)

    for r in range(robots.num):
        c = robots.coords[r].copy() # save previous position in case of collision
        # update robot positions
        robots.update_movement(r, robots.noise_factor)

        # returns true if there was a collision
        # and moves robot back to original position and reorients
        if(DEMO):
            retval = robots.check_collision(r, obstacles, c)

        # update stimuli
        return_data = robots.update_stim(r, robots.num, robots.coords[r], big_coords.copy(), big_angles.copy(), big_stimuli.copy(), group_list, metric, robots.lim_angle, robots.lim_distance, robots.influence_scale, robots.split)

    sim_data.append([robots.coords[:,0].copy(), robots.coords[:,1].copy(), robots.angles.copy(), robots.stimuli.copy()])


data = pd.DataFrame(data = sim_data, columns = ["x","y","theta","stim"])

outdat = os.path.join(datadir, "demo.csv")

data.to_csv(outdat, lineterminator = "")
