from turtle import distance, heading, right
import numpy as np
import random

import utils
import environment

class Robots():
    def __init__(self, filename, config_filename, type, DEMO=0):

        # load basics
        rconfig = utils.load_config(filename)
        self.ss = int(rconfig["screen_size"])
        self.env = environment.Environment(self.ss)
        self.num = int(rconfig["num_robots"])

        # set up coordinates
        if(DEMO):
            coords = self.env.safeStartTwoRooms(self.num)
        else:
            rng = np.random.default_rng()
            coords = rng.choice(np.arange(self.ss),size=self.num*2)
            coords = np.reshape(coords,(self.num,2))
        self.coords = np.array(coords, dtype=float)
        self.disp_coords = np.array(coords, dtype=int)
        self.v = np.full(self.num,rconfig["robot_vel"])
        self.angles = np.random.random(self.num)*2*np.pi
        self.stimuli = np.full(self.num, 128)

        # robot design parameters
        self.max_vel            = float(rconfig["robot_vel"])
        self.lim_distance       = float(rconfig["lim_distance"])
        self.lim_angle          = float(rconfig["lim_angle"])
        self.angle_incr         = float(rconfig["angle_incr"])
        self.rep_range          = float(rconfig["rep_range"])
        self.rep_factor         = float(rconfig["rep_factor"])
        self.stimscale          = float(rconfig["stimscale"])
        self.influence_scale    = float(rconfig["influence_scale"])
        self.noise_factor       = float(rconfig["noise_factor"])
        self.split              = float(rconfig["split"])

        # behavior mode
        bparams = utils.load_typed_config(config_filename, type)
        self.attract                = bool(bparams["attract"])
        self.speed_up               = bool(bparams["speed_up"])
        self.directional_stim      = bool(bparams["directional_stimulus"])
        self.full_omni              = not bool(bparams["directional_sense"])

        # stigmergy
        self.grid_num = rconfig["grid_num"]
        self.x_coords = np.arange(0, self.grid_num, dtype = float)
        self.new_grid = np.arange(0, self.grid_num*self.grid_num, dtype = tuple)
        self.perGrid = np.arange(0, self.grid_num*self.grid_num, dtype = int)
        self.robotsPer = {}
        self.prevRobotsPer = {}
        self.changePer = {}
        self.prevShade = {}
        self.fading = {}

        # overpowered agent for demo
        self.DEMO = DEMO
        if(DEMO):
            self.influence_skew = np.array(([10000]+[1]*(self.num-1))*5)
        else:
            self.influence_skew = np.full(self.num*5,1.)

    def update_movement(self, r, noise_factor):
        c = self.coords[r]
        # add movement noise
        self.v[r] = self.v[r]+np.random.normal(0,noise_factor*self.max_vel)
        # cap value
        if(self.v[r]>4):
            self.v[r]=4.0
        elif(self.v[r]<0):
            self.v[r]=0.0
        self.angles[r] = (self.angles[r]+np.random.normal(0,noise_factor*np.pi))
        xnew = c[0] + self.v[r]*np.cos(self.angles[r])
        ynew = c[1] + self.v[r]*np.sin(self.angles[r])

        # update coords, respects torus
        self.coords[r] = np.array(utils.wrap_pt([xnew, ynew], self.ss, self.ss, offset=(0,0)))
        self.disp_coords[r] = self.coords[r].astype(int)

    def distance_calc(self, diff, r, lim_distance):
        distances = np.linalg.norm(diff,axis=1)
        distances[r] = lim_distance
        valid_dist_ind = np.where(distances<lim_distance)
        # the value for the current robot doesn't matter
        # it is multiplied by 0 in the next steps
        distances[distances>lim_distance] = lim_distance
        # invert 
        with np.errstate(divide='ignore'):
            inv_distances = 1-np.square(distances/lim_distance)

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
    # under directional stim / sensing constraints
    # valid_leader is a list of agent indices such that agent r is in each agent i's sensing cone
    # valid_follower is a list of agent indices where each agent i is in agent r's sensing cone
    def valid_angle(self, diff, r, lim_angle, coords, angles):

        # diff is vector from possible "follower" (agent r) to possible "leader" (agent i)
        # diff = coords[:] - coords[r]
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
        
        return valid_leader, valid_follower, beta

    def angle_calc(self, diff, r, lim_angle, big_coords, big_angles):
        # calc angles between robot and stim sources
        valid_leader, valid_follower, beta = self.valid_angle(diff, r, lim_angle, big_coords, big_angles)
        
        # valid_angle_ind = np.intersect1d(valid_sum, valid_angle_ind)

        # directional: constant stimulus within the cone
        # percieved stim is a function of incident stim angle on sensor (beta)
        if(self.directional_stim):
            valid_angle_ind = np.intersect1d(valid_leader, valid_follower)
            # angle_scale = np.pi + big_angles - (np.arctan2((big_coords[r,1]-big_coords[:,1]),(big_coords[r,0] - big_coords[:,0])) % (2*np.pi))
            # angle_scale = 1-np.square(beta/lim_angle)
            angle_scale = 1-np.square(beta/lim_angle)
            angle_scale[r] = 0
            
        # semi-omni; constrain follower windshield but have omnidirectional stimulus
        elif(not self.full_omni):
            valid_angle_ind = valid_follower
            # angle scale is 1 everywhere because the influence is independent of source angle
            angle_scale = np.full(shape=self.num*5,fill_value=1)

        # full omni
        else:
            angle_scale = np.full(shape=self.num*5,fill_value=1)
            valid_angle_ind = np.where(angle_scale>0)[0]
            

        angle_scale[angle_scale>1] = 1

        return angle_scale, valid_angle_ind

    # compute agents visible in right and left hand sides of "windshield"
    def direction_calc(self, heading, diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, stimuli):
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
                inpoly, data = utils.IsInPolyNoHoles(loc_wrapped, o, theta)
            except:
                # will raise exception if ray is parallel to polygon edge
                # TODO handle gracefully
                raise(ValueError, "in poly check not working")

            # if collision found, do billiard bounce
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
                        theta = utils.bounce(closest_edge, prev_c, c)
                # move to previous location and rotate in place
                self.angles[r] = theta
                self.coords[r] = prev_c
                MADE_CHANGE = True
                break # assumes obstacles do not overlap
            else:
                pass

        return MADE_CHANGE

    def update_stim(self, r, n, c, big_coords, big_angles, big_stimuli, group_list, metric, lim_angle, lim_distance, influence_scale, split):
        # find the relative coords of the matrix to the current point
        diff = big_coords - c

        # create a list of distances between current robot and others
        inv_distances, valid_dist_ind, distances = self.distance_calc(diff, r, lim_distance)

        # include directional constraints as needed and distance/angle scaling
        angle_scale, valid_angle_ind = self.angle_calc(diff, r, lim_angle, big_coords, big_angles)
        right_tot, left_tot, valid_ind = self.direction_calc(self.angles[r], diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale,
big_stimuli)

        # set the stim value to the sum of stimulus intensity for visualization
        stim = (left_tot+right_tot)*self.stimscale

        # cap values
        if(stim>1):
            stim=1
        if(stim<0):
            stim=0 # stim started going negative for some reason?
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
            self.v[r] = (stim)*self.max_vel
        else:
            self.v[r] = (1-stim)*self.max_vel

        # normalize and set
        self.stimuli[r] = np.ceil(255*stim)

        # TODO: this only will work if we have only one of these metrics in the
        # list of metrics
        # and it throws an error ("element-wise comparison failed")
        # if("dist" in metric):
        #     dist = np.linalg.norm(self.coords - c,axis=1)
        #     return dist
        # if("fig" in metric):
        #     return stim
        # if("nnd" in metric):
        #     return np.amin(distances)
        return 0

    # Stigmergy utils
    #####################################################################################
    #Creates array of tuples (x, y, index, stimulus) to represent the corners of the grid
    #and sets "new_grid" equal to the array and returns the array
    def make_grid(self, grid_num):
        interval = self.ss/grid_num
        grid_corners = np.arange(0, grid_num*grid_num, dtype = tuple)
        index = 0

        for x_vals in range(grid_num):
            x = float(x_vals*interval)
            self.x_coords[x_vals] = float(x)
            for y_vals in range(grid_num):
                y = float(y_vals*interval)
                grid_corners[index] = (x, y, index, 0.0)
                index += 1
        new_grid = grid_corners.reshape(grid_num, grid_num)
        self.new_grid = new_grid

        return new_grid

    # Upsates all of the stimulus with the value fed into the function
    def update_stimulus(self, stimulus):
        print(stimulus)
        for points in range(len(self.new_grid)):
            self.new_grid[3] = stimulus
            print(self.new_grid[points])

    def shadeFade(self, shade, concentration):
        #shade = 255 - (5*robotsPerGrid[boxes])
        fadeFactor = 5
        newShade = shade + fadeFactor
        if newShade > 255:
            newShade = 255
        if newShade < 0:
            newShade = 0

        if concentration > 0:
            return newShade
        else:
            return shade



    #Takes in the coordinate that the robot is at, and returns the corresponding number
    # to the box in the grid it is in
    def coord_near(self, grid_num, robotCoord):
        interval = self.ss/grid_num
        x_ind = 0
        y_ind = 0

        for x_coord in range(grid_num):
            if (robotCoord[0] >= self.new_grid[x_coord][0][0] and robotCoord[0] < self.new_grid[x_coord][0][0]+interval):
                x_ind = x_coord

        for y_coord in range(grid_num):
            if (robotCoord[1] >= self.new_grid[x_ind][y_coord][1] and robotCoord[1] < self.new_grid[x_ind][y_coord][1]+interval):
                y_ind = y_coord

        box = (y_ind*self.grid_num)+x_ind
        return box

    # Function to run all the needed updates in play_demo
    def robotUpdates(self, c):
        self.make_grid(self.grid_num)
        boxIn = self.coord_near(self.grid_num, c)
        if boxIn in self.robotsPer:
            self.robotsPer[boxIn] += 1
        # else:
        #     self.robotsPer[boxIn] = 1
        return(self.robotsPer)

    def initRobotsPer(self):
        #self.prevRobotsPer = self.robotsPer
        boxes = self.grid_num*self.grid_num
        for box in range(boxes):
            self.robotsPer[box] = 0

        #print(self.robotsPer)

    def initPrevRobotsPer(self):
        #self.prevRobotsPer = self.robotsPer
        boxes = self.grid_num*self.grid_num
        for box in range(boxes):
            self.prevRobotsPer[box] = 0

    def initPrevShade(self):
        #self.prevRobotsPer = self.robotsPer
        boxes = self.grid_num*self.grid_num
        for box in range(boxes):
            self.prevShade[box] = 0

    def initFading(self):
        #self.prevRobotsPer = self.robotsPer
        boxes = self.grid_num*self.grid_num
        for box in range(boxes):
            self.fading[box] = False


# not using, may revisit this concept later
def chain_calc(self, metric, big_stimuli, valid_ind, inv_distances, angle_scale):

    # currently not using
    if metric == "chain":
        # create a list of the values that should be zeroed out
        # ind = np.intersect1d(valid_dist_ind, valid_angle_ind)
        bad_ind = np.setdiff1d(np.arange(5*self.num), valid_ind)  # inverse the ind list
        # create lists with these values zeroed out
        temp_stim = big_stimuli.copy()
        temp_stim[bad_ind] = 0
        temp_dist = inv_distances.copy()
        temp_dist[bad_ind] = 0
        temp_angle = angle_scale.copy()
        temp_angle[bad_ind] = 0
        # then compile down into length n arrays
        temp_stim = temp_stim[0:n]+temp_stim[n:2*n]+temp_stim[2*n:3*n]+temp_stim[3*n:4*n]+temp_stim[4*n:5*n]
        temp_dist = temp_dist[0:n]+temp_dist[n:2*n]+temp_dist[2*n:3*n]+temp_dist[3*n:4*n]+temp_dist[4*n:5*n]
        temp_angle = temp_angle[0:n]+temp_angle[n:2*n]+temp_angle[2*n:3*n]+temp_angle[3*n:4*n]+temp_angle[4*n:5*n]
        # calculate total influence
        total = (temp_stim*temp_dist*temp_angle)/(influence_scale)
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
