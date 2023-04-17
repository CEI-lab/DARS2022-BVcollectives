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
VIZGRID = False
FPS = 55
DEMO = 0
tag = "aggression"
type = "dir_omni"
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

params = utils.load_config(global_filename)
sim_time, ss, num = params["sim_time"], params["screen_size"], params["num_robots"]
obstacles = utils.load_env(e_filename)
width = ss # for vid
height = ss # for vid

# set up env
pygame.init()
screen = pygame.display.set_mode([ss,ss])
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)

# import the data
sim_data = pd.read_csv("data/demo.csv",dtype=object)

x_list = []
y_list = []
theta_list = []
stimuli_list = []
for i in range(sim_data['x'].shape[0]):
    x_list.append(list(map(float,sim_data['x'][i][1:-1].replace(" \n", "").split())))
    y_list.append(list(map(float,sim_data['y'][i][1:-1].replace(" \n", "").split())))
    theta_list.append(list(map(float,sim_data['theta'][i][1:-1].replace(" \n", "").split())))
    stimuli_list.append(list(map(float,sim_data['stim'][i][1:-1].replace(" \n", "").split())))


########################## MAIN  ###########################################3

# init robots
robots = Robots(global_filename, c_filename, type, DEMO)

robots.coords = np.array([x_list,y_list]).T
robots.angles = np.array(theta_list)
robots.stimuli = np.array(stimuli_list)

running = True

# sim loop
framenum = 0
robots.initRobotsPer()
robots.initPrevRobotsPer()
robots.initPrevShade()
robots.initFading()
#robots.prevShade = robots.robotsPer
for time in range(sim_time):
    if(not running):
        break
    # did the user click the close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the background with white
    screen.fill((255,255,255))

    # robots.initRobotsPer()

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

    ############################################################################
    #NEW PS
    #robots.robotsPer.clear()
    #print("Reset")
    # for x in robots.robotsPer:
    #     # if robots.robotsPer[x] == 0:
    #     #     del robots.robotsPer[x]
    #     # else:
    #         robots.robotsPer[x] = 0
    #print(robots.num)

    #robots.prevRobotsPer = robots.robotsPer
    # print(time)
    # print(robots.robotsPer)
    # print(robots.prevRobotsPer)
    #print(robots.prevShade)


    for r in range(robots.num):
        c = robots.coords[r,time]
        robotsPerGrid = robots.robotUpdates(c)
    ##print("new")
    #print(robots.robotsPer)
    # print("now draw")
    for boxes in robots.robotsPer:
            #fading = False
            fade = 10

            #robots.changePer[boxes] = robots.robotsPer[boxes] - robots.prevRobotsPer[boxes]
            # print("new")
            # print(robots.robotsPer)
            # print(robots.prevRobotsPer)
            # print(robots.changePer)

            concentration = robots.robotsPer[boxes]
            shade = 255 - (5*concentration)
        #print(str(shading) + " " + str(shade) + " "  + str(concentration))
            if shade < 0:
                shade = 0

            # find way to make it stop fading, if robot enters again

            interval = robots.ss/robots.grid_num

            xHold = boxes%robots.grid_num
            yHold = boxes//robots.grid_num

            x = xHold*interval
            y = yHold*interval

            # if robots.fading[boxes] != False:
            #     timeChange = time - robots.fading[boxes]
            #     shade += timeChange*fade
            #     if shade > 255:
            #         shade = 255

            #decay by x percent
            #implement a constant decay rate (linear decay)
            # also do: x% of total stimulus per timestep

            # if boxes == 6 and time > 0:
            #     print("new")
            #     print(robots.prevShade[boxes])
            #     print(shade)
            #
            # if time > 0:
            #     temp = robots.prevShade[boxes]
            #
            # robots.prevShade[boxes] = shade
            #             #if concentration == 0 and time > 0:
            # if time > 0 and shade == temp and shade != 255:
            #     shade = robots.prevShade[boxes] + fade
            #     robots.prevShade[boxes] = shade
            #     if robots.fading[boxes] == False:
            #         robots.fading[boxes] = time
            #     if boxes == 6:
            #         print("here")
            #         print(robots.fading[boxes])
            #
            #
            #
            #
            #
            #
            # if shade > 255:
            #     shade = 255
            pygame.draw.rect(screen, (255, shade, 255), (x,y,interval,interval), 0)
    #print(robots.robotsPer)
    # print(robots.prevRobotsPer)
    # print(robots.changePer)

    #robots.prevRobotsPer = robots.robotsPer

    # Draws all of the lines needed to make the grid, prints the box numbers (starting at 1)
    # and draws small dots at all of the intersections of the gridlines
    if VIZGRID:
        for points in range(robots.grid_num):
            interval = robots.ss/robots.grid_num
            inter = 0
            cornerNum = 1
            pt = float(points*interval)
            font = pygame.font.SysFont(None, 15)

            pygame.draw.line(screen, (255, 0, 0), (pt, 0), (pt, robots.ss), width=1)
            pygame.draw.line(screen, (255, 0, 0), (0, pt), (robots.ss, pt), width=1)

            for points2 in range(robots.grid_num):
                corner = int(cornerNum + (inter/interval))
                pygame.draw.circle(screen, (255, 0, 0), (pt, inter), 3)

                img = font.render(str((points2*robots.grid_num)+points), True, (255, 0, 0))
                screen.blit(img, (pt +5 , inter +5))
                cornerNum += 1
                inter += interval
            #extra row on the far right
            pygame.draw.circle(screen, (255, 0, 0), (robots.ss, pt), 3)
    ############################################################################



    # update robot positions
    for r in range(robots.num):

        c = robots.coords[r,time]
        l = robots.stimuli[time,r]

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
