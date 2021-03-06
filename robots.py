from turtle import distance, heading, right
import numpy as np
import random

import utils

class robot_class():
    def __init__(self, filename, c_filename, ss, DEMO=0):
        params = utils.load_robot_config(filename)
        num = params["num_robots"]

        self.ss = ss
        self.num = num
        # coords = np.random.choice([x for x in range(int(np.ceil(ss*0.2)),int(np.ceil(ss*0.8)),1)],num*2)
        if(DEMO):
            coords = utils.hardcoded_gen_coords(ss,num)
        else:
            coords = np.random.choice([x for x in range(self.ss)],num*2)
            coords.resize(num,2)
        self.coords = np.array(coords)
        self.v = np.full(num,params["robot_vel"])
        self.angles = np.random.random(num)*2*np.pi
        # self.angles = np.full(num,np.pi/4)
        # self.lights = np.zeros(num)
        self.lights = np.full(num, 128)
        self.temp_for_print = 0

        # params
        self.max_vel = params["robot_vel"]
        self.lim_distance = params["lim_distance"]
        self.lim_angle = params["lim_angle"]
        self.angle_incr = params["angle_incr"]
        self.rep_range = params["rep_range"]
        self.rep_factor = params["rep_factor"]
        self.lightscale = params["lightscale"]
        self.influence_scale = params["influence_scale"]
        self.noise_factor = params["noise_factor"]
        self.split = params["split"]
        self.full_omni = params["full_omni"]

        params = utils.load_configuration_config(c_filename)
        self.attract = params["attract"]
        self.speed_up = params["speed_up"]
        self.directional_light = params["directional_light"]

        self.DEMO = DEMO
        if(DEMO):
            self.influence_skew = np.array(([10000]+[1]*(num-1))*5)
        else:
            self.influence_skew = np.full(num*5,1.)

    def update_movement(self, r, noise_factor):
        c = self.coords[r]
        # print(c)
        # add movement noise
        v = self.v[r]+np.random.normal(0,noise_factor*self.max_vel)
        # cap value
        if(v>4):
            v=4
        elif(v<0):
            v=0
        theta = self.angles[r]+np.random.normal(0,noise_factor*np.pi)
        xnew = c[0] + v*np.cos(theta)
        ynew = c[1] + v*np.sin(theta)

        # update coords, respects torus
        self.coords[r] = np.array(utils.wrap_pt([xnew, ynew], self.ss, self.ss, offset=(0,0)))

    def distance_calc(self, diff, r, lim_distance):
        distances = np.linalg.norm(diff,axis=1)
        # the value for the current robot doesn't matter because it is multiplied by 0 in the next step
        distances[r] = 1
        # invert 
        with np.errstate(divide='ignore'):
            inv_distances = 1-np.square(distances/lim_distance)
        inv_distances[r] = 0
        distances[r]=1000
        # zero out distances past the limit, set limit to half of the screen size minus a small cushion of 10 pixels for safety
        valid_dist_ind = np.where(distances<lim_distance)

        inv_distances[inv_distances>1] = 1
        return inv_distances, valid_dist_ind[0]

    def angle_calc(self, r, lim_angle, big_coords, big_angles):
        # calc angles between robot and light sources
        # first calc the difference in headings
        heading_angles = (big_angles - self.angles[r])
        # then calc the angle between agents from their positions, subtract pi/2 to correct reference frame
        coord_angles = np.arctan2((big_coords[:,1]-big_coords[r,1]),(big_coords[:,0] - big_coords[r,0])) - np.pi/2
        # the effective angle between the light of the lead robot to the following robot is the sum of the position based angle and the difference in their headings, the abs is importnat
        angles = np.abs(coord_angles + heading_angles)
        valid_angle_ind = np.where(angles<lim_angle)[0]
        
        if(self.directional_light):
            angle_scale = 1-np.square(angles/lim_angle)
            angle_scale[r] = 0
        # semi-omni light
        else:
            # if lantern light, then make the angles scale 1 because the influence is independent of angles, and make valid_angle_ind a complete list because all angles are valid
            angle_scale = np.full(shape=self.num*5,fill_value=1)

        # full omni
        if(not self.directional_light and self.full_omni):
            valid_angle_ind = np.where(angle_scale>0)[0]

        angle_scale[angle_scale>1] = 1

        return angle_scale, valid_angle_ind


    def direction_calc(self, heading, diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, lights):
        valid_ind = np.intersect1d(valid_angle_ind, valid_dist_ind)
        rind, lind = utils.pts_left_right(heading, diff)
        rind = np.intersect1d(rind, valid_ind)
        lind = np.intersect1d(lind, valid_ind)

        # calc right and left
        # right_tot = np.sum(lights[rind]*inv_distances[rind]*angle_scale[rind])/(influence_scale)
        # left_tot = np.sum(lights[lind]*inv_distances[lind]*angle_scale[lind])/(influence_scale)
        right_tot = np.sum(self.influence_skew[rind]*inv_distances[rind]*angle_scale[rind])/(influence_scale)
        left_tot = np.sum(self.influence_skew[lind]*inv_distances[lind]*angle_scale[lind])/(influence_scale)

        return right_tot, left_tot, valid_ind



    # assume polygon obstacles do not have holes
    def check_collision(self, r, obstacles, prev_c):
        MADE_CHANGE = False
        c = self.coords[r]
        loc_wrapped = utils.wrap_pt(c, self.ss, self.ss) # just in case? obstacle checking assumes we're in (0,ss)x(0,ss)

        # shoot ray back the way to both
        # check if we're inside obstacle
        # and find which edge we collide with in case of collision
        ray_out = prev_c - c
        theta = np.arctan2(ray_out[1], ray_out[0])
        for o in obstacles: # list of CCW points
            inpoly = False
            # shoot one ray for each obstacle, then check edges individually if inside
            try:
                inpoly, data = utils.IsInPolyNoHoles(loc_wrapped, o, theta)
            except: # incoming vector was parallel to polygon edge
                raise(ValueError, "in poly check not working")

            if inpoly:
                closest_edge = data[0][1:]
                dist = 100000000
                # following check not really necessary for our purposes
                # but ray may intersect multiple edges if obstacle nonconvex
                for pt, v1, v2 in data:
                    if np.linalg.norm(pt-c) < dist:
                        dist = np.linalg.norm(pt-c)
                        closest_edge = (v1, v2)
                        theta = utils.bounce(closest_edge, prev_c, c)
                # move to previous location and rotate in place
                self.angles[r] = theta
                self.coords[r] = prev_c
                MADE_CHANGE = True
                break # assume only in one obstacle at a time lol
            else:
                pass

        return MADE_CHANGE

    def update_light(self, r, n, c, big_coords, big_angles, big_lights, group_list, metric, lim_angle, lim_distance, influence_scale, split):
        # find the relative coords of the matrix to the current point
        diff = big_coords - c

        # create a list of distances between current robot and others (reuse the variable diff from above)
        inv_distances, valid_dist_ind = self.distance_calc(diff, r, lim_distance)

        # include directionality of light
        angle_scale, valid_angle_ind = self.angle_calc(r, lim_angle, big_coords, big_angles)

        right_tot, left_tot, valid_ind = self.direction_calc(self.angles[r], diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, big_lights)

        # set the light value to the sum
        light = (left_tot+right_tot)/self.lightscale

        # cap values
        if(light>1):
            light=1
        if(right_tot>1):
            right_tot=1
        if(left_tot>1):
            left_tot=1

        # then move the agents
        if(self.attract):
            if(left_tot>right_tot):
                self.angles[r] += left_tot*self.angle_incr
            elif(left_tot<right_tot):
                self.angles[r] -= right_tot*self.angle_incr
        else:
            if(left_tot>right_tot):
                self.angles[r] -= left_tot*self.angle_incr
            elif(left_tot<right_tot):
                self.angles[r] += right_tot*self.angle_incr

        # normalize
        self.angles[r] = self.angles[r]%(2*np.pi)

        # update velocity value of this robot
        if(self.speed_up):
            self.v[r] = (light)*self.max_vel
        else:
            self.v[r] = (1-light)*self.max_vel
        # print(robots.v[r])

        

        # normalize and set
        # print(light)
        self.lights[r] = np.ceil(255*light)

        ###### record chain lengths #######
        if("chain" in metric):
            # create a list of the values that should be zeroed out
            # ind = np.intersect1d(valid_dist_ind, valid_angle_ind)
            bad_ind = np.setdiff1d(np.arange(5*n), valid_ind)  # inverse the ind list
            # create lists with these values zeroed out
            temp_light = big_lights.copy()
            temp_light[bad_ind] = 0
            temp_dist = inv_distances.copy()
            temp_dist[bad_ind] = 0
            temp_angle = angle_scale.copy()
            temp_angle[bad_ind] = 0
            # then compile down into length n arrays
            temp_light = temp_light[0:n]+temp_light[n:2*n]+temp_light[2*n:3*n]+temp_light[3*n:4*n]+temp_light[4*n:5*n]
            temp_dist = temp_dist[0:n]+temp_dist[n:2*n]+temp_dist[2*n:3*n]+temp_dist[3*n:4*n]+temp_dist[4*n:5*n]
            temp_angle = temp_angle[0:n]+temp_angle[n:2*n]+temp_angle[2*n:3*n]+temp_angle[3*n:4*n]+temp_angle[4*n:5*n]
            # calculate total influence
            total = (temp_light*temp_dist*temp_angle)/(influence_scale)
            # unweight the graph
            total = np.array(total)
            total[total<split] = 0
            total[total>=split] = 1
            # add the agents in total to every group where they intersect
            gi = np.where(np.sum(np.logical_and(group_list,total),axis=1)[0]>0)[0]
            # print(gi)
            if(np.any(gi)):
                group_list[gi] = np.logical_or(group_list[gi],total)
            # if they don't interesct anywhere, create a new group
            else:
                group_list.append(total)
        if("dist" in metric):
            dist = np.linalg.norm(self.coords - c,axis=1)
            return dist
        if("fig" in metric):
            return light
        else:
            return 0



