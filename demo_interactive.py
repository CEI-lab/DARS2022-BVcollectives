import numpy as np
import pandas as pd
import time
import pygame
import os

# custom imports
import utils
from robots import *

########################## PARAMETERS ###########################################

SAVE_DATA = False
VIZGRID = False
DEMO = True
SAVE_VID = True
FPS = 15

previous_time_test = time.time()

#variables for saving data

tag = "aggression"
type = "dir_omni"

vid_outname = "videos/demo_video.mp4"
global_filename = "configs/global_demo_config.yaml"
c_filename = "configs/" + tag + ".yaml"
e_filename = "configs/env_config.yaml"
metric = np.array([])

# set up folder for saving frames
if SAVE_VID:
    try:
        os.makedirs("demo_frames")
    except OSError:
        pass

############################### MAIN ##############################################

sim_data = []
params = utils.load_config(global_filename)
obstacles = utils.load_env(e_filename)
# init robots
robots = Robots(global_filename, c_filename, type, DEMO)
# make sure controllable robot has influence
robots.stimuli[0] = 1000
robots.v[0] = 0.5
#obstacles = []
width = robots.ss # for vid
height = robots.ss # for vid

# set up env
pygame.init()
screen = pygame.display.set_mode([robots.ss,robots.ss])
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)

# gen colors
colors = utils.gen_colors(robots.num)


group_list = [np.zeros(robots.num)]

running = True
t=0

# write initial config to cmd
print("behavior: " + tag)

# sim loop
# for t in range(sim_time):
framenum = 0
while(running):
    t+=1
    # handle user input
    for event in pygame.event.get():
        # did the user click the close button?
        if event.type == pygame.QUIT:
            running = False
        # move controlable agent
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT or event.key == ord('a'):
                robots.angles[0] -= np.pi/12
                robots.angles[0] %= (2*np.pi)
                print("theta: " + str(robots.angles[0]))
            if event.key == pygame.K_RIGHT or event.key == ord('d'):
                robots.angles[0] += np.pi/12
                robots.angles[0] %= (2*np.pi)
                print("theta: " + str(robots.angles[0]))
            if event.key == pygame.K_UP or event.key == ord('w'):
                if(robots.v[0]<=3.5):
                    robots.v[0]+=0.1
                print("velocity: " + str(robots.v[0]))
            if event.key == pygame.K_DOWN or event.key == ord('s'):
                if(robots.v[0]>=0.1):
                    robots.v[0]-=0.1
                print("velocity: " + str(robots.v[0]))
            # change behavior
            if event.key == ord('1'):
                if(robots.attract == 0):
                    robots.attract = 1
                else:
                    robots.attract = 0
                print("attract: " + str(robots.attract))
            if event.key == ord('2'):
                if(robots.speed_up == 0):
                    robots.speed_up = 1
                else:
                    robots.speed_up = 0
                print("speed up: " + str(robots.speed_up))


    # fill the background with white
    screen.fill((255,255,255))

    # draw obstacles
    for o in obstacles:
        pts = list(zip(o, o[1:]))+[(o[-1], o[0])]
        for (p1, p2) in pts:
            pygame.draw.line(screen, (100,100,100), p1, p2)

    # setup big arrays for each frame
    big_coords, big_stimuli, big_angles = utils.setup_big_arrays(robots)

    # update controllable robot
    c = robots.coords[0].copy()
    robots.update_movement(0, robots.noise_factor)
    retval = robots.check_collision(0, obstacles, c)
    if retval:
        print(f"Agent 0 bounced, theta = {robots.angles[0]}")
    cc = robots.disp_coords[0]
    pygame.draw.circle(screen, np.minimum(255,robots.stimuli[0])*colors[0], np.ceil(cc), 8)
    pygame.draw.line(screen, np.minimum(255,robots.stimuli[0])*colors[0], np.ceil(cc), np.ceil(np.array(cc)+15*np.array([np.cos(robots.angles[0]),np.sin(robots.angles[0])])), 3)


    # update robot positions
    for r in range(1,robots.num):
        # save previous position in case of collision
        c = robots.coords[r].copy()

        # update movements
        robots.update_movement(r, robots.noise_factor)

        # returns true if there was a collision
        # and moves robot back to original position and reorients
        retval = robots.check_collision(r, obstacles, c)

        # draw a solid blue circle in the center
        cdisp = robots.disp_coords[r]
        pygame.draw.circle(screen, robots.stimuli[r]*colors[r], np.ceil(cdisp), 5)

        # draw a line to show orientation
        pygame.draw.line(screen, robots.stimuli[r]*colors[r], np.ceil(cdisp), np.ceil(np.array(cdisp)+15*np.array([np.cos(robots.angles[r]),np.sin(robots.angles[r])])), 3)

        # update stim
        return_data = robots.update_stim(r, robots.num, robots.coords[r], big_coords.copy(), big_angles.copy(),
big_stimuli.copy(), group_list, metric, robots.lim_angle, robots.lim_distance, robots.influence_scale, robots.split)
        # print(robots.stimuli[r])

    # save data
    if(SAVE_DATA):
        sim_data.append([robots.coords[:,0].copy(), robots.coords[:,1].copy(), robots.angles.copy(),
robots.stimuli.copy()])

    # update the fps counter
    # screen.blit(utils.update_fps(clock,font), (10,0))

    # print behavior
    screen.blit(utils.print_behavior(robots, font), (20,20))

    clock.tick(60)

    # save frame to disk
    if SAVE_VID:
        fname = "demo_frames/%04d.png" % framenum
        pygame.image.save(screen, fname)
        framenum += 1

    # update the display
    pygame.display.flip()

if(SAVE_DATA):
    data = pd.DataFrame(data = sim_data, columns = ["x","y","theta","stim"])
    data.to_csv("data/demo.csv", line_terminator = "")

if SAVE_VID:
    cmd = str(f"ffmpeg -r {FPS} -f image2 -i demo_frames/%04d.png -y -qscale 0 -s {width}x{height} {vid_outname}")
    os.system(cmd)


# quit
pygame.quit()
