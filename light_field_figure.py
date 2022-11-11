from operator import inv
import numpy as np
import matplotlib.pyplot as plt

import utils
import robots as rb

# TODO: import utils and robots and use those functions instead

########################## PARAMETERS ###########################################

#variables for saving data
sim_data = []

global_filename = "configs/figure_config.yaml"
c_filename = "configs/love.yaml"
type = "dir_dir"
metric = np.array([])
split = 0.5

############################### MAIN ##############################################

# ss = 250 #screen size
sim_time, ss, n = utils.load_global_config(global_filename)


# init robots
robots = rb.Robots(global_filename, c_filename, type, ss)

# params
rob_x = 205
rob_y = int(ss/2)
rob_theta = 0#np.pi/2

robots.coords[0] = np.array([ss/2,125])

# set default value to middle. Should be gray
mat = np.full((ss,ss), 127.)



for y in range(ss):
    for x in range(ss):
        robots.coords[1] = np.array([x,y])
        robots.lights = np.full(2,255)
        robots.angles = np.full(2,np.pi/2)
        big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)
        
        light = robots.update_light(1, robots.num, np.array([x,y]), big_coords, big_angles, big_lights, [], ["fig"], robots.lim_angle, robots.lim_distance, robots.influence_scale, 0.5)
        # print(light)
        mat[y][x] = light
        # calc_light(x,y,0)

# print(mat)

im = plt.imshow(mat[:][0:150], extent=[0, ss, 0, 150], origin='lower',
           cmap='cividis')
plt.colorbar(im,fraction=0.03, pad=0.04)
im.set_clim(0,1)
# plt.axis(aspect='image')

plt.show()
