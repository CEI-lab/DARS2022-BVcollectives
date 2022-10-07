from statistics import median
import numpy as np
import random
import pandas as pd
import matplotlib.pyplot as plt
import sys, os
import time

# custom imports
import utils
from robots import *

np.set_printoptions(threshold=sys.maxsize)

########################## PARAMETERS ###########################################

# vary these values to produce different behavior
# influence_scale = 1
noise_factor = 0.01388888 # 0.013888pi = 5 degrees in radians

previous_time_test = time.time()


datadir = './data'
figdir = './plots'
if not os.path.exists(datadir):
    os.mkdir(datadir)
if not os.path.exists(figdir):
    os.mkdir(figdir)

########################## FUNCTIONS ###########################################3

def iteration(metric, global_filename, c_filename, type):

    sim_time, ssize, num_bots = utils.load_global_config(global_filename)
    t = 0
    # list to hold all of the adjacency matrices
    data_list = [] 

    group_list_record = []

    # init robots
    robots = Robots(global_filename, c_filename, type, ssize, False)
    robots.lim_angle = lim_angle
    robots.influence_scale = influence_scale
    robots.noise_factor = noise_factor

    #variables for saving data
    sim_data = []
    group_list = [np.zeros(robots.num)]
    angle_metric_list = []
    dist_metric_list = []
    energy_metric_list = []
    nnd_metric_list = []

    # sim loop
    # while(t<sim_time-1):
    for t in range(sim_time):
        # increment time
        # t+=1
        # print(robots.lights)
        # temporary variable to accumulate the row influence
        data_accum = np.zeros((robots.num,robots.num))

        big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

        # update robot positions
        for r in range(robots.num):
            # update movements
            robots.update_movement(r, noise_factor)

            # update light
            return_data = robots.update_light(r, robots.num, robots.coords[r], big_coords.copy(), big_angles.copy(), big_lights.copy(), group_list, metric, lim_angle, robots.lim_distance, influence_scale, robots.split)

            if("dist" in metric):
                data_accum[r] = return_data
            if("nnd" in metric):
                data_accum[r] = return_data

       
        if("dist" in metric):
            dist_metric_list.append(np.mean(data_accum))
        if("nnd" in metric):
            nnd_metric_list.append(np.mean(data_accum))
        if("energy" in metric):
            energy_metric_list.append(np.mean(np.square(robots.v)))
        

    return_metrics = []
    if("dist" in metric):
        return_metrics.append(dist_metric_list)
    if("nnd" in metric):
        return_metrics.append(nnd_metric_list)
    if("energy" in metric):
        return_metrics.append(energy_metric_list)

    return np.array(return_metrics)


def sweep(filename, config, metric, variables, type, iterations=20):
    # make these modifiable
    global lim_angle
    global influence_scale
    global noise_factor

    sim_time, ssize, num_bots = utils.load_global_config(filename)

    # initialize timing variable
    previous_time = time.time()

    config_filename = "configs/" + config + ".yaml"

    # run sweep
    # angle_list = []
    mean_accum = np.zeros((10,10,len(metric)))
    std_accum = np.zeros((10,10,len(metric)))
    for n in range(10):
        if(variables == "angle_influence"):
            lim_angle = np.pi/2-3*n*np.pi/72
        elif(variables == "influence_noise"):
            # influence_scale = 1+4*n/10
            noise_factor = 0.45 - 0.05*n
        for m in range(10):
            influence_scale = 0.25*(m+1)
            # noise_factor = 0.02*m
            data_list = []
            for p in range(iterations):
                iter_data = iteration(metric, filename, config_filename, type)
                data_list.append(np.mean(iter_data[:,sim_time-10:sim_time],axis=1))
            print("square " + str(10*n+m) + " of 100, value: " + str(np.mean(data_list,axis=0)) + ", time: " + str(time.time()-previous_time))
            previous_time=time.time()
            mean_accum[n,m] = np.mean(data_list,axis=0)
            std_accum[n,m] = np.std(data_list,axis=0)

    # plot heatmap
    # TODO: parameterize this to work for different numbers of plots
    for d in range(len(metric)):
        fig, ax = plt.subplots()
        im = ax.imshow(mean_accum[:,:,d])
        if(variables == "angle_influence"):
            plt.ylabel("Angle (rad)")
            ax.set_yticks(np.arange(10), labels=["$\pi$/2","","5$\pi$/12","","$\pi$/3","","$\pi$/4","","$\pi$/6",""]) # lowest value is 0.275pi
        # ax.set_yticks(np.arange(9), labels=["1, pi/2","","1/2, 7pi/16","","1/3, 3pi/8","","1/4, 5pi/16","","1/5, pi/4"])
        elif(variables == "influence_noise"):
            plt.ylabel("Noise")
            ax.set_yticks(np.arange(10), labels=["","0.4","","0.3","","0.2","","0.1","","0.0"])
        plt.xlabel("Influence Scaler")
        ax.set_xticks(np.arange(10), labels=["","0.5","","1","","1.5","","2","","2.5"])
        # ax.set_xticks(np.arange(10), labels=["0.0","","0.04","","0.08","","0.12","","0.16",""])
        cbar = plt.colorbar(im)
        if(metric[d] == "dist"):
            plt.title("Distance Between Agents")
            im.set_clim(0,270)
            cbar.set_label("Average Distance Between Agents at Convergence")
        if(metric[d] == "nnd"):
            plt.title("Nearest Neighbor Distance Between Agents")
            im.set_clim(0,55)
            cbar.set_label("NND Between Agents at Convergence")
        if(metric[d] == "energy"):
            plt.title("Kinetic Energy")
            im.set_clim(0,15)
            cbar.set_label("Average Kinetic Energy of system at Convergence")
        # save the plot with a unique title for its configuration
        outname = variables + '_' + config +  "_" + type + "_" + metric[d] + "_" + str(iterations)
        plt.savefig("plots/" + outname +  "_plot.png")

        # save the data as a backup so we can generate a new plot later if we want to change purely cosmetic things
        data_mean = pd.DataFrame(data = mean_accum[:,:,d].flatten(), columns = [metric[d]])
        data_mean.to_csv("data/" + outname + "_mean_data.csv", line_terminator = "")
        data_std = pd.DataFrame(data = std_accum[:,:,d].flatten(), columns = [metric[d]])
        data_std.to_csv("data/" + outname + "_std_data.csv", line_terminator = "")

def sweep_noise(filename, configs, metric, variables, type, iterations=20):
    # assert len(metric) == 4  # because right now the code is not parameterized correctly for any other number
    # make these modifiable
    global lim_angle
    global influence_scale
    global noise_factor

    influence_scale = 1
    lim_angle = np.pi/3
    noise_factor = 0.01

    sim_time, ssize, num_bots = utils.load_global_config(filename)

    fig, ax = plt.subplots(1,2, figsize=(14, 6))

    noise = np.flip(np.arange(0.00,0.1,0.01))

    lines = []

    for config in configs:

        # initialize timing variable
        previous_time = time.time()

        config_filename = "configs/" + config + ".yaml"

        # run sweep
        # angle_list = []
        mean_accum = np.zeros((10,2))
        std_accum = np.zeros((10,2))
        for m in range(10):
            noise_factor = 0.09 - 0.01*m
            # noise_factor = 0.01*m
            data_list = []
            for p in range(iterations):
                iter_data = iteration(metric, filename, config_filename, type)
                data_list.append(np.mean(iter_data[:,sim_time-10:sim_time],axis=1))
            print("iter " + str(m) + " of 10, value: " + str(np.mean(data_list,axis=0)) + ", time: " + str(time.time()-previous_time))
            previous_time=time.time()
            mean_accum[m] = np.mean(data_list,axis=0)
            std_accum[m] = np.std(data_list,axis=0)

        # plot distance
        line1, = ax[0].plot(noise, mean_accum[:,0])
        ax[0].fill_between(noise, mean_accum[:,0]-std_accum[:,0], mean_accum[:,0]+std_accum[:,0], alpha=0.3)
        # plot kinetic energy
        line2 = ax[1].plot(noise, mean_accum[:,1])
        ax[1].fill_between(noise, mean_accum[:,1]-std_accum[:,1], mean_accum[:,1]+std_accum[:,1], alpha=0.3)

        # save data
        outname = variables + '_' + config +  "_" + type + "_" + str(iterations) + ".csv"
        data = pd.DataFrame(data=mean_accum[:,0], columns=["mean nnd"])
        data["mean ke"] = mean_accum[:,1]
        data["std nnd"] = std_accum[:,0]
        data["st ke"] = std_accum[:,1]
        outdat = os.path.join(datadir, outname)
        data.to_csv(outdat)

        lines.append(line1)

    ax[0].set_title("Distance Between Agents")
    ax[0].set_ylabel("Average Distance Between Agents at Convergence (pixels)")
    ax[0].set_xlabel("Noise")

    ax[1].set_title("Kinetic Energy")
    ax[1].set_ylabel("Average Kinetic Energy of system at Convergence")
    ax[1].set_xlabel("Noise")

    plt.legend(handles=lines,labels=configs)
        
    outplot = os.path.join(datadir, variables + "_" + str(iterations) + "_" + type + "_plot.png")
    plt.savefig(outplot)


if __name__ == '__main__':

    sim_time, ssize, num_bots = utils.load_global_config("configs/global_config.yaml")
    metrics = np.array(["nnd", "energy"])
    sweep("configs/global_config.yaml", "love", metrics, "angle_influence", "dir_dir", 2)
