from multiprocessing import connection
import numpy as np
import pygame
import random
import pandas as pd
import os
import glob

import utils
from robots import *

########################## PARAMETERS ###########################################

SAVE_VID = True
FPS = 55
DEMO = 0
tag = "curiosity"
type = "dir_dir"
vid_name = tag+"_" + type + ".mp4"
viddir = './videos'
global_filename = "configs/global_config.yaml"
c_filename = "configs/"+tag+".yaml"
e_filename = "configs/env_config.yaml"

metric = np.array([])

########################## SETUP ##########################

if not os.path.exists(viddir):
    os.mkdir(viddir)
vid_out = os.path.join(viddir, vid_name)

# set up folder for saving frames
if SAVE_VID:
    try:
        os.makedirs(tag+"_frames")
    except OSError:
        pass

sim_time, ss, num = utils.load_global_config(global_filename)
obstacles = utils.load_environment_config(e_filename)
width = ss # for vid
height = ss # for vid

# set up env
pygame.init()
screen = pygame.display.set_mode([ss,ss])
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)
# set up plot

#stats = pygame.Surface((300,200 ))
#stats.fill((150,150,150))
#stats.set_alpha(230)
#stats_pos = (50,25)

# import the data
sim_data = pd.read_csv("data/test.csv",dtype=object)

x_list = []
y_list = []
theta_list = []
light_list = []
for i in range(sim_data['x'].shape[0]):
    x_list.append(list(map(int,sim_data['x'][i][1:-1].replace(" \n", "").split())))
    y_list.append(list(map(int,sim_data['y'][i][1:-1].replace(" \n", "").split())))
    theta_list.append(list(map(float,sim_data['theta'][i][1:-1].replace(" \n", "").split())))
    light_list.append(list(map(int,sim_data['light'][i][1:-1].replace(" \n", "").split())))


########################## MAIN  ###########################################3

# init robots
robots = Robots(global_filename, c_filename, type, ss, 1)

robots.coords = np.array([x_list,y_list]).T
robots.angles = np.array(theta_list)
robots.lights = np.array(light_list)

running = True

# sim loop
framenum = 0
for time in range(sim_time):
    if(not running):
        break
    # did the user click the close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the background with white
    screen.fill((255,255,255))

    # draw obstacles
    if(DEMO):
        for o in obstacles:
            pts = list(zip(o, o[1:]))+[(o[-1], o[0])]
            for (p1, p2) in pts:
                pygame.draw.line(screen, (100,100,100), p1, p2)

    # add axis labels for plot
    #x_label = font.render('Time (sec)', True, (0,0,0))
    #screen.blit(x_label, (100, 200))
    #y_label = font.render('Intensity', True, (0,0,0))
    #y_label = pygame.transform.rotate(y_label, 90)
    #screen.blit(y_label, (2,70))

    # update robot positions
    for r in range(robots.num):
        
        c = robots.coords[r,time]
        l = robots.lights[time,r]

        # draw a solid blue circle in the center
        pygame.draw.circle(screen, (0,0,l), np.ceil(c), 5)

        # draw a line to show orientation
        pygame.draw.line(screen, (0,0,l), np.ceil(c), np.ceil(c+15*np.array([np.cos(robots.angles[time,r]),np.sin(robots.angles[time,r])])), 3)

    # update stats

    ############ light intensity plot #################
    # plot_light(robots, time)

    ############ connectivity graph ###################
    # print(time)
    #update_conn_matrix(conn_list[time].copy())

    ############ length of chain plot #################
    # plot_max_path(conn_list[time])
    
    ############ eigenvalue centrality plot ###########
    #plot_eig_centrality_series(conn_list[time].copy(), time)

    # update the fps counter
    #screen.blit(utils.update_fps(), (10,0))

    clock.tick(60)

    # save frame to disk
    if SAVE_VID:
        fname = tag + "_frames/%04d.png" % framenum
        pygame.image.save(screen, fname)
        framenum += 1

    # update the display
    pygame.display.flip()

# quit
pygame.quit()

########################## SAVE TO VID ###########################################

if SAVE_VID:
    cmd = str(f"ffmpeg -r {FPS} -f image2 -i {tag}_frames/%04d.png -y -qscale 0 -s {width}x{height} {vid_out}")
    os.system(cmd)

    # remove frames when done: python should wait...
    # TODO switch to using subprocess lib
    # or maybe pathlib
    files = glob.glob('./'+tag+'_frames/*.png')
    for f in files:
        try:
            os.unlink(f)
        except OSError as e:
            print("Error: %s : %s" % (f, e.strerror))
            pass

    try:
        os.rmdir('./'+tag+'_frames')
    except OSError as e:
        pass
