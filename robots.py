from turtle import distance, heading, right
import numpy as np
import random

import utils

class Robots():
    def __init__(self, filename, c_filename, type, ss, DEMO=0):
        params = utils.load_robot_config(filename)
        num = params["num_robots"]

        self.ss = ss
        self.num = num
        if(DEMO):
            coords = utils.hardcoded_gen_coords(ss,num)
        else:
            coords = np.random.Generator.choice([x for x in range(self.ss)],num*2)
            coords.reshape((num,2))
        self.coords = np.array(coords, dtype=float)
        self.disp_coords = np.array(coords, dtype=int)
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

        all_params = utils.load_behavior_config(c_filename)
        params = {k: v for d in all_params[type] for k, v in d.items()} # unpack yaml...
        self.attract = params["attract"]
        self.speed_up = params["speed_up"]
        self.directional_light = params["directional_stimulus"]
        self.full_omni = not bool(params["directional_sense"])

        self.DEMO = DEMO
        if(DEMO):
            self.influence_skew = np.array(([10000]+[1]*(num-1))*5)
        else:
            self.influence_skew = np.full(num*5,1.)

    def update_movement(self, r, noise_factor):
        c = self.coords[r]
        # add movement noise
        self.v[r] = self.v[r]+np.random.normal(0,noise_factor*self.max_vel)
        # cap value
        if(self.v[r]>4):
            self.v[r]=4.0
        elif(self.v[r]<0):
            self.v[r]=0.0
        self.angles[r] = (self.angles[r]+np.random.normal(0,noise_factor*np.pi))%(2*np.pi)
        xnew = c[0] + self.v[r]*np.cos(self.angles[r])
        ynew = c[1] - self.v[r]*np.sin(self.angles[r])

        # update coords, respects torus
        self.coords[r] = np.array(utils.wrap_pt([xnew, ynew], self.ss, self.ss, offset=(0,0)))
        self.disp_coords[r] = self.coords[r].astype(int)

    def distance_calc(self, diff, r, lim_distance):
        distances = np.linalg.norm(diff,axis=1)
        distances[r] = lim_distance
        valid_dist_ind = np.where(distances<lim_distance)
        # the value for the current robot doesn't matter because it is multiplied by 0 in the next steps
        distances[distances>lim_distance] = lim_distance
        # print(distances)
        # invert 
        with np.errstate(divide='ignore'):
            inv_distances = 1-np.square(distances/lim_distance)
        # inv_distances[r] = 0
        # distances[r]=1000
        # print(inv_distances)
        # print(inv_distances[valid_dist_ind])

        inv_distances[inv_distances>1] = 1
        return inv_distances, valid_dist_ind[0], distances

    # get indices of agents "above" and "below" in global coordinates
    def agents_above_below(self, r, th_coords):

        up_all = np.where(th_coords <= np.pi)
        down_all = np.setdiff1d(np.arange(self.num), up_all) # all that is not up is down

        up = np.setdiff1d(up_all, np.array([r])) # remove self if present
        down = np.setdiff1d(down_all, np.array([r])) # remove self if present

        return up, down


    def calc_alpha(self, r, angles, th_coords):
        alpha = np.zeros(len(angles))

        # this can be vectorized using the same method as above
        for i in range(len(angles)):
            # check exceptions
            if ((np.pi <= th_coords[i] and th_coords[i] < 2*np.pi) and (0 <= angles[r] and angles[r] < th_coords[i] - np.pi)):
                alpha[i] = th_coords[i] - np.pi - angles[r]
            elif ((np.pi <= th_coords[i] and th_coords[i] < 2*np.pi) and (np.pi - th_coords[i] <= angles[r] and angles[r] < th_coords[i])):
                alpha[i] = th_coords[i] - angles[r]
            else:
                alpha[i] = th_coords[i] + np.pi - angles[r]

            if alpha[i] > np.pi:
                alpha[i] -= 2*np.pi

        return alpha

    # beta is the angular coordinate of the leader in the follower's reference frame
    # beta = 0 when follower is aimed directly at leader
    def calc_beta(self, r, angles, th_coords):
        beta = np.zeros(len(angles))

        # this can be vectorized using the same method as above
        for i in range(len(angles)):
            # check exceptions
            if (0 <= th_coords[i] and th_coords[i] < np.pi and (th_coords[i]+np.pi <= angles[r] and angles[r] < 2*np.pi)):
                beta[i] = th_coords[i] + 2*np.pi - angles[r]
            elif (np.pi <= th_coords[i] and th_coords[i] < 2*np.pi and 0 <= angles[r] and angles[r] < th_coords[i]-np.pi):
                beta[i] = th_coords[i] - 2*np.pi - angles[r]
            else:
                beta[i] = th_coords[i] - angles[r]

        return beta

    # determines visibility from agent r to all other agents
    # under directional light / sensing constraints
    # valid_leader is a list of agent indices such that agent r is in each agent i's light cone
    # valid_follower is a list of agent indices where each agent i is in agent r's sensing cone
    def valid_angle(self, diff, r, lim_angle, coords, angles):

        # diff is vector from possible "follower" (agent r) to possible "leader" (agent i)
        # diff = coords[:] - coords[r]
        # print(diff)
        # print(np.arctan2(diff[:,1],diff[:,0]))
        # print(np.arctan2(diff[:,1],diff[:,0])%(2*np.pi))
        th_coords = np.arctan2(diff[:,1],diff[:,0])%(2*np.pi)

        valid_sum = []
        beta = self.calc_beta(r, angles, th_coords)
        alpha = self.calc_alpha(r, angles, th_coords)

        # th_coords[th_coords > 2*np.pi-lim_angle] = th_coords - 2*np.pi
        valid_leader = np.setdiff1d(np.where(np.abs(angles - th_coords) <= lim_angle)[0], [r]) # the [0] is because of the weird return of np.where
        valid_leader_phase_adjusted = np.setdiff1d(np.where(np.abs(angles - th_coords-2*np.pi) <= lim_angle)[0], [r])
        valid_leader = np.union1d(valid_leader, valid_leader_phase_adjusted)
        valid_follower = np.setdiff1d(np.where(np.abs(angles[r] - th_coords) <= lim_angle)[0], [r]) # the setdiff1d is to remove self from the list
        valid_follower_phase_adjusted = np.setdiff1d(np.where(np.abs(angles[r] - th_coords - 2*np.pi) <= lim_angle)[0], [r])
        valid_follower = np.union1d(valid_follower, valid_follower_phase_adjusted)
        # valid_sum = np.union1d(valid_leader, valid_follower) # dummy variable to conform to the return statement of valid_angle()

        # print(angles*180/np.pi)
        # print(th_coords)
        # print(angles)
        # print(np.abs(angles - th_coords))
        # print(valid_leader)
        # print(np.abs(angles[r] - th_coords))
        # print(valid_follower)
        
        return valid_leader, valid_follower, beta

    def angle_calc(self, diff, r, lim_angle, big_coords, big_angles):
        # calc angles between robot and light sources
        valid_leader, valid_follower, beta = self.valid_angle(diff, r, lim_angle, big_coords, big_angles)
        
        # valid_angle_ind = np.intersect1d(valid_sum, valid_angle_ind)

        # directional: constant stimulus within the cone
        # percieved light is a function of incident light angle on sensor (beta)
        if(self.directional_light):
            valid_angle_ind = np.intersect1d(valid_leader, valid_follower)
            # angle_scale = np.pi + big_angles - (np.arctan2((big_coords[r,1]-big_coords[:,1]),(big_coords[r,0] - big_coords[:,0])) % (2*np.pi))
            # angle_scale = 1-np.square(beta/lim_angle)
            angle_scale = 1-np.square(beta/lim_angle)
            angle_scale[r] = 0
            
        # semi-omni; constrain follower windshield but have omnidirectional stimulus
        elif(not self.full_omni):
            valid_angle_ind = valid_follower
            # angle scale is 1 everywhere because the influence is independent of incoming light angle
            angle_scale = np.full(shape=self.num*5,fill_value=1)

        # full omni
        else:
            angle_scale = np.full(shape=self.num*5,fill_value=1)
            valid_angle_ind = np.where(angle_scale>0)[0]
            

        angle_scale[angle_scale>1] = 1

        return angle_scale, valid_angle_ind


    # TODO: what is influence skew? *this is for the demo where one of the agents has significantly greater influence than the other agents
    def direction_calc(self, heading, diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, lights):
        valid_ind = np.intersect1d(valid_angle_ind, valid_dist_ind)
        rind_all, lind_all = utils.pts_left_right(heading, diff)
        rind = np.intersect1d(rind_all, valid_ind)
        lind = np.intersect1d(lind_all, valid_ind)

        # calc right and left
        right_tot = np.sum(self.influence_skew[rind]*inv_distances[rind]*angle_scale[rind])*(influence_scale)
        left_tot = np.sum(self.influence_skew[lind]*inv_distances[lind]*angle_scale[lind])*(influence_scale)

        return right_tot, left_tot, valid_ind


    # assume polygon obstacles do not have holes
    def check_collision(self, r, obstacles, prev_c):
        MADE_CHANGE = False
        c = self.coords[r]
        loc_wrapped = utils.wrap_pt(c, self.ss, self.ss) # just in case? obstacle checking assumes we're in (0,ss)x(0,ss)

        # shoot ray back the way agent moved in last time step to both:
        # 1. check if we're inside obstacle
        # 2. and find which edge we collide with in case of collision
        ray_out = prev_c - c
        theta = np.arctan2(ray_out[1], ray_out[0])
        for o in obstacles: # list of CCW points
            inpoly = False
            # shoot one ray for each obstacle, then check edges individually if inside
            try:
                inpoly, data = utils.IsInPolyNoHoles(loc_wrapped, o, self.ss, theta)
            except:
                # will raise exception if ray is parallel to polygon edge
                # TODO handle gracefully
                raise(ValueError, "in poly check not working")

            if inpoly:
                closest_edge = data[0][1:]
                dist = 100000000
                # following check for closest edge not really necessary for our purposes
                # but ray may intersect multiple edges if obstacle nonconvex
                for pt, v1, v2 in data:
                    if np.linalg.norm(pt-c) < dist:
                        dist = np.linalg.norm(pt-c)
                        closest_edge = (v1, v2)
                        # reorient according to elastic collision law
                        theta = utils.bounce(closest_edge, prev_c, c, self.ss)
                # move to previous location and rotate in place
                self.angles[r] = theta
                self.coords[r] = prev_c
                MADE_CHANGE = True
                break # assumes obstacles do not overlap
            else:
                pass

        return MADE_CHANGE

    def update_light(self, r, n, c, big_coords, big_angles, big_lights, group_list, metric, lim_angle, lim_distance, influence_scale, split):
        # find the relative coords of the matrix to the current point
        diff = big_coords - c

        # create a list of distances between current robot and others
        inv_distances, valid_dist_ind, distances = self.distance_calc(diff, r, lim_distance)

        # include directional constraints as needed and distance/angle scaling
        angle_scale, valid_angle_ind = self.angle_calc(diff, r, lim_angle, big_coords, big_angles)
        right_tot, left_tot, valid_ind = self.direction_calc(self.angles[r], diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, big_lights)

        # set the light value to the sum of stimulus intensity for visualization
        light = (left_tot+right_tot)*self.lightscale

        # cap values
        if(light>1):
            light=1
        if(light<0):
            light=0 # light started going negative for some reason?
        if(right_tot>1):
            right_tot=1
        if(left_tot>1):
            left_tot=1

        # then update heading left or right
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
        self.angles[r] %= (2*np.pi)

        # update velocity value of this robot
        if(self.speed_up):
            self.v[r] = (light)*self.max_vel
        else:
            self.v[r] = (1-light)*self.max_vel

        # normalize and set
        self.lights[r] = np.ceil(255*light)

        # TODO: this only will work if we have only one of these metrics in the
        # list of metrics
        # and it throws an error ("element-wise comparison failed")
        if("dist" in metric):
            dist = np.linalg.norm(self.coords - c,axis=1)
            return dist
        if("fig" in metric):
            return light
        if("nnd" in metric):
            return np.amin(distances)
        else:
            return 0


# not using, may revisit this concept later
def chain_calc(self, metric, big_lights, valid_ind, inv_distances, angle_scale):

    # currently not using
    if metric == "chain":
        # create a list of the values that should be zeroed out
        # ind = np.intersect1d(valid_dist_ind, valid_angle_ind)
        bad_ind = np.setdiff1d(np.arange(5*self.num), valid_ind)  # inverse the ind list
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
        return group_list
