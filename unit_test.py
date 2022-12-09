import numpy as np
import matplotlib.pyplot as plt
import yaml

import utils
import robots as rb

# test if points within the range with no offset remain unchanged
def test_utils_wrap_pt_1():
    pt = [0,0]
    w, h = 500, 500
    assert utils.wrap_pt(pt,w,h) == [0,0], "wrap_pt test1 failed"

    pt = [100,0]
    w, h = 500, 500
    assert utils.wrap_pt(pt,w,h) == [100,0], "wrap_pt test1 failed"

    pt = [250,250]
    w, h = 500, 500
    assert utils.wrap_pt(pt,w,h) == [250,250], "wrap_pt test1 failed"

    pt = [499,300]
    w, h = 500, 500
    assert utils.wrap_pt(pt,w,h) == [499,300], "wrap_pt test1 failed"

# test if points outside of the range get wrapped correctly with no offset
def test_utils_wrap_pt_2():
    pt = [600,0]
    w, h = 500, 500
    new_pt = utils.wrap_pt(pt,w,h)
    assert new_pt == [100,0], "wrap_pt test2 gave"+str(new_pt)+"instead of [100, 0]"

    pt = [0,600]
    new_pt = utils.wrap_pt(pt,w,h)
    assert new_pt == [0,100], "wrap_pt test2 gave"+str(new_pt)+"instead of [0, 100]"

    pt = [-300,0]
    new_pt = utils.wrap_pt(pt,w,h)
    assert new_pt == [200,0], "wrap_pt test2 gave"+str(new_pt)+"instead of [200, 0]"

    pt = [w,-100]
    new_pt = utils.wrap_pt(pt,w,h)
    assert new_pt == [w,400], "wrap_pt test2 gave"+str(new_pt)+"instead of [500, 400]"

def test_utils_load_global_config():
    d = {'world' :
           {'sim_time' : 200,
            'screen_size' : 500},
         'robot' :
           {'num_robots' : 20,
            'robot_vel' : 4,
            'lim_distance' : 100,
            'lim_angle' : 0.25,
            'angle_incr' : 0.01,
            'rep_range' : 0,
            'rep_factor' : 3,
            'lightscale' : 1}}

    with open('configs/test_config.yaml', 'w') as yaml_file:
        yaml.dump(d, yaml_file, default_flow_style=False)

    sim_time, ss, num = utils.load_global_config('configs/test_config.yaml')

    assert sim_time == 200, "load_global_config test1 failed"
    assert ss == 500, "load_global_config test1 failed"
    assert num == 20, "load_global_config test1 failed"

# TODO: implement
def test_utils_load_robot_config():
    pass

# TODO: implement
def test_utils_load_behavior_config():
    pass

# TODO: implement
def test_utils_setup_big_arrays_1():
    # init robots
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/4,np.pi/4])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    assert np.array_equal(big_coords,np.array([[0,0],[10,0],[0,10],[500,0],[510,0],[500,10],[0,500],[10,500],[0,510],[-500,0],[-490,0],[-500,10],[0,-500],[10,-500],[0,-490]]))
    assert np.array_equal(big_lights,np.full(15,255))
    assert np.array_equal(big_angles,np.array([np.pi/2,np.pi/4,np.pi/4]*5))


# TODO: implement
def test_utils_setup_big_arrays_2():
    pass

# basic test
def test_utils_bounce_1():
    edge = ((-10,0),(10,0))
    prev_c = np.array([-1,np.sqrt(3)])
    new_c = np.array([1,-np.sqrt(3)])
    angle = utils.bounce(edge, prev_c, new_c)
    np.testing.assert_almost_equal(angle, np.pi/3.)

def test_utils_bounce_2():
    edge = ((-10,0),(10,0))
    prev_c = np.array([0,-1])
    new_c = np.array([0,1])
    angle = utils.bounce(edge, prev_c, new_c)
    a = angle % (2*np.pi)
    np.testing.assert_almost_equal(a, 3*np.pi/2.)

def test_utils_bounce_3():
    edge = ((-10,0),(10,0)) # edge along x axis
    prev_c = np.array([1,-1])
    new_c = np.array([-1,1]) # should bounce at 45 degrees
    angle = utils.bounce(edge, prev_c, new_c)
    a = angle % (2*np.pi)
    a_correct = (5*np.pi/4)
    np.testing.assert_almost_equal(a, a_correct)


def test_utils_inpoly_1():
    poly = [[-100, -100], [100, -100], [100, 100], [-100, 100]]
    pt_in = [0,0]
    pt_out = [0, 400]

    retval1, data1 = utils.IsInPolyNoHoles(pt_in, poly, theta = np.pi/2+0.01)
    retval2, data2 = utils.IsInPolyNoHoles(pt_out, poly, theta = np.pi/2+0.01)
    edge = [data1[0][1], data1[0][2]] # data is a list of lists

    assert retval1 == True
    assert retval2 == False
    assert np.array_equal(np.array(edge), np.array([[100,100],[-100,100]]))


def test_utils_inpoly_2():
    poly = [[200,200], [300, 200], [300, 300], [200, 300]]

    pt_in = [295, 285]

    retval, data = utils.IsInPolyNoHoles(pt_in, poly, theta = np.random.random()*2*np.pi)

    assert retval == True

################### ROBOT TESTS ##############################

def test_robots_init():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression.yaml"
    type = "omni_omni"

    sim_time, ss, num = utils.load_global_config(global_filename)

    # init robots
    robots = rb.Robots(global_filename, c_filename, type, ss)
    robots.lim_distance = 200
    return robots

# test basic calc
def test_robots_distance_calc_1():
    robots = test_robots_init()
    coords = np.array([[0,0],[100,0],[150,0]])
    c = np.array([0,0])

    diff = coords - c
    inv_distances, valid_dist_ind, distances = robots.distance_calc(diff, 0, robots.lim_distance)

    assert np.array_equal(inv_distances,np.array([0.,0.75,0.4375]))
    assert np.array_equal(valid_dist_ind,np.array([1,2]))
    
# test robot out of bounds
def test_robots_distance_calc_2():
    robots = test_robots_init()

    coords = np.array([[0,0],[50,0],[100,0],[400,0],[0,200]])
    c = np.array([0,0])

    diff = coords - c
    inv_distances, valid_dist_ind, distances = robots.distance_calc(diff, 0, robots.lim_distance)

    assert np.array_equal(inv_distances,np.array([0,0.9375,0.75,0,0]))
    assert np.array_equal(valid_dist_ind,np.array([1,2]))

# test distnace in different direction, nonzero starting value
def test_robots_distance_calc_3():
    robots = test_robots_init()

    coords = np.array([[100,0],[150,0],[175,0],[500,0],[0,0]])
    c = np.array([100,0])

    diff = coords - c
    inv_distances, valid_dist_ind, distances = robots.distance_calc(diff, 0, robots.lim_distance)

    assert np.array_equal(inv_distances,np.array([0,0.9375,0.859375,0,0.75]))
    assert np.array_equal(valid_dist_ind,np.array([1,2,4]))

# TODO: implement, test distance around torus
def test_robots_distance_calc_4():
    robots = test_robots_init()

    robots.coords = np.array([[0,0],[100,0],[150,0]])
    c = np.array([0,0])

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    diff = big_coords - c
    inv_distances, valid_dist_ind, distances = robots.distance_calc(diff, 0, robots.lim_distance)

    # assert np.array_equal(inv_distances,np.array([0.,0.75,0.4375]))
    # assert np.array_equal(valid_dist_ind,np.array([1,2]))

def test_robots_coords_above_below():
    robots = test_robots_init()
    robots.num = 3
    coords = np.array([[0,0],[100,100],[150,150]])
    r = 1
    diff = coords[:] - coords[r]
    th_coords = np.arctan2(diff[:,1],diff[:,0]) % (2*np.pi)
    above, below = robots.agents_above_below(r, th_coords)
    assert np.array_equal(above, np.array([2]))
    assert np.array_equal(below, np.array([0]))


# test headings only
def test_robots_valid_angle_1():
    robots = test_robots_init()
    robots.lim_angle = np.pi/4
    robots.num = 2

    coords = np.array([[0,0],[0,10]]) 
    angles = np.array([np.pi/2,np.pi/2])
    r = 0

    c = np.array([0,0])
    diff = coords - c

    valid_leader, valid_follower, beta = robots.valid_angle(diff, r, robots.lim_angle, coords, angles)

    # print(valid_leader)
    # print(valid_follower)

    assert np.array_equal(valid_leader, np.array([1]))
    assert np.array_equal(valid_follower, np.array([1]))

# test leader header and follower header mismatch
def test_robots_valid_angle_2():
    robots = test_robots_init()
    robots.lim_angle = np.pi/4
    robots.num = 2

    coords = np.array([[0,0],[0,10]])
    angles = np.array([0,np.pi/2])
    r = 0

    c = np.array([0,0])
    diff = coords - c

    valid_leader, valid_follower, beta = robots.valid_angle(diff, r, robots.lim_angle, coords, angles)

    # print(valid_leader)
    # print(valid_follower)

    assert np.array_equal(valid_leader, np.array([1]))
    assert np.array_equal(valid_follower, np.array([]))

# test heading and coords successful
def test_robots_valid_angle_3():
    robots = test_robots_init()
    robots.lim_angle = np.pi/4
    robots.num = 2

    coords = np.array([[0,0],[10,10*np.sqrt(3)]])
    angles = np.array([np.pi/2,np.pi/3])
    r = 0

    c = np.array([0,0])
    diff = coords - c

    valid_leader, valid_follower, beta = robots.valid_angle(diff, r, robots.lim_angle, coords, angles)

    # print("results")
    # print(valid_leader)
    # print(valid_follower)

    assert np.array_equal(valid_leader, np.array([1]))
    assert np.array_equal(valid_follower, np.array([1]))

# test heading and coords unsuccessful
def test_robots_valid_angle_4():
    robots = test_robots_init()
    robots.lim_angle = np.pi/4
    robots.num = 2

    coords = np.array([[0,0],[0,10]])
    angles = np.array([np.pi/2,3*np.pi/4])
    r = 0

    c = np.array([0,0])
    diff = coords - c

    valid_leader, valid_follower, beta = robots.valid_angle(diff, r, robots.lim_angle, coords, angles)

    # print("results")
    # print(valid_leader)
    # print(valid_follower)

    assert np.array_equal(valid_leader, np.array([1]))
    assert np.array_equal(valid_follower, np.array([1]))


# test calc on normal, directional light, headings only
def test_robots_angle_calc_1():
    robots = test_robots_init()
    # setup test parameters
    robots.coords = np.array([[0,0],[0,10],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/3,0])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    c = np.array([0,0])
    diff = big_coords - c

    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(valid_angle_ind)

    assert np.allclose(angle_scale[0:2],np.array([0,1])) # this is necessary so that I can input 1/3 and it works instead of matching the exact number of 3's in 0.33333
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1]))

# test lantern light
def test_robots_angle_calc_2():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,10],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/4,0])
    robots.lights = np.array([255,255,255])
    robots.directional_light = 0
    robots.full_omni = 1
    robots.num = 3
    # robots.lim_angle = 0.5

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    c = np.array([0,0])
    diff = big_coords - c

    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(np.full(robots.num*5,1))
    # print(valid_angle_ind)
    # print(np.arange(0,robots.num*5))

    assert np.array_equal(angle_scale,np.full(robots.num*5,1))
    assert np.array_equal(valid_angle_ind,np.arange(0,robots.num*5),1)

# test directional same heading coordinate only
def test_robots_angle_calc_3():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5],[5,5*np.sqrt(3)]])
    robots.angles = np.array([np.pi/2,np.pi/2,np.pi/2])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    c = np.array([0,0])
    diff = big_coords - c

    # angles are 0, 45, 60
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(valid_angle_ind)

    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.allclose(angle_scale[0:3],np.array([0,0.75,8/9])) 
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1,2]))

# test directional heading and coordinate
def test_robots_angle_calc_4():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5],[5,5*np.sqrt(3)]])
    robots.angles = np.array([np.pi/2,7*np.pi/12,5*np.pi/12])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    c = np.array([0,0])
    diff = big_coords - c

    # coord angles are 0, pi/4, pi/3, heading angles pi/12, -pi/12
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(valid_angle_ind)

    assert np.allclose(angle_scale[0:3],np.array([0,0.75,8/9]))
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1,2]))

# test coords with wrapping
def test_robots_angle_calc_5():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/2,np.pi/2])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = 4*np.pi/9
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    c = np.array([0,0])
    diff = big_coords - c

    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, big_coords, big_angles)

    # print(valid_angle_ind)

    assert np.array_equal(valid_angle_ind,np.array([2,6,7,8]))  # [2,5,6,7,8,11] was originally correct, should double check this

# test directional same heading coordinate only, with different starting angle
def test_robots_angle_calc_6():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5],[5,5*np.sqrt(3)]])
    robots.angles = np.array([np.pi/4,np.pi/4,np.pi/4])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots)

    c = np.array([0,0])
    diff = big_coords - c

    # angles are 0, 45, 60
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, big_coords, big_angles)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.allclose(angle_scale[0:3],np.array([0,1,35/36])) 
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1,2]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_1():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0]])
    robots.angles = np.array([0,0])
    th_coords = np.array([0,0])  # theta_C - the angle from follower to leader in global frame [0,2pi]


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == 0
    assert alpha[1] == np.pi
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([1]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_2():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0]])
    robots.angles = np.array([np.pi/2,np.pi/2])
    th_coords = np.array([0,0])  # theta_C = the angle from follower to leader in global frame [0,2pi]


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(alpha)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == -np.pi/2
    assert alpha[1] == np.pi/2
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([1]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_3():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0]])
    robots.angles = np.array([-np.pi/3,-np.pi/3])
    th_coords = np.array([0,0])  # theta_C - the angle from follower to leader in global frame [0,2pi]


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == np.pi/3
    assert np.isclose(alpha[1],-2*np.pi/3)
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_4():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,10]])
    th_coords = np.array([0,np.pi/2])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([0,0])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == np.pi/2
    assert alpha[1] == -np.pi/2
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([1]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_5():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,10]])
    th_coords = np.array([0,np.pi/2])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([np.pi/3,np.pi/3])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(angle_scale)

    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/6)
    assert np.isclose(alpha[1],-5*np.pi/6)
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.75]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_6():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,10]])
    th_coords = np.array([0,np.pi/2])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([2*np.pi/5,2*np.pi/5])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == np.pi/10
    assert alpha[1] == -9*np.pi/10
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.91]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_7():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5]])
    th_coords = np.array([0,np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([np.pi/4,np.pi/2])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(alpha)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == 0
    assert alpha[1] == 3*np.pi/4
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([1]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_8():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5]])
    th_coords = np.array([0,np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([np.pi/4,np.pi/8])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(1, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert beta[1] == 0
    assert alpha[1] == -7*np.pi/8
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([1]))

def test_robots_more_angle_tests_9():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-5,5]])
    th_coords = np.array([0,3*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([2*np.pi/3,2*np.pi/3])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/12)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.9375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_10():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-5,5]])
    th_coords = np.array([0,3*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([2*np.pi/3,3*np.pi/4])


    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/12)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.9375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_11():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-10,0]])
    th_coords = np.array([0,np.pi])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([5*np.pi/6,np.pi/2])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/6)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([0.9375]))

    # tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_12():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-10,0]])
    th_coords = np.array([0,np.pi])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([5*np.pi/6,np.pi])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/6)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.75]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_13():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-5,-5]])
    th_coords = np.array([0,5*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([4*np.pi/3,4*np.pi/3])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],-np.pi/12)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.9375]))

def test_robots_more_angle_tests_14():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-5,-5]])
    th_coords = np.array([0,5*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([3*np.pi/2,5*np.pi/4])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],-np.pi/4)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.4375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_15():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[-5,-5]])
    th_coords = np.array([0,5*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([np.pi/4,np.pi/2])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([0.9375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_16():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,-10]])
    th_coords = np.array([0,3*np.pi/2])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([11*np.pi/8,7*np.pi/4])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/8)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.859375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_17():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,-10]])
    th_coords = np.array([0,3*np.pi/2])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([np.pi,np.pi])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/2)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([0.4375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_18():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[0,-10]])
    th_coords = np.array([0,3*np.pi/2])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([5*np.pi/4,np.pi])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/4)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([0.4375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_19():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,-5]])
    th_coords = np.array([0,7*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([7*np.pi/4,2*np.pi])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2

    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],0)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([1]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_20():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,-5]])
    th_coords = np.array([0,7*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([3*np.pi/2,7*np.pi/4])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],np.pi/4)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([1]))
    assert np.allclose(angle_scale[1:2],np.array([0.4375]))

# tests several parts of the angle calc for different configurations
def test_robots_more_angle_tests_21():
    robots = test_robots_init()

    # setup test parameters
    robots.coords = np.array([[0,0],[5,-5]])
    th_coords = np.array([0,7*np.pi/4])  # theta_C - the angle from follower to leader in global frame [0,2pi]
    robots.angles = np.array([np.pi/4,np.pi/2])

    robots.lights = np.array([255,255])
    robots.lim_angle = np.pi/3
    robots.directional_light = 1
    robots.num = 2


    c = np.array([0,0])
    diff = robots.coords - c

    beta = robots.calc_beta(0, robots.angles, th_coords)
    # alpha = robots.calc_alpha(0, robots.angles, th_coords)
    angle_scale, valid_angle_ind = robots.angle_calc(diff, 0, robots.lim_angle, robots.coords, robots.angles)

    # print(beta)
    # print(alpha)
    # print(valid_angle_ind)
    
    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.isclose(beta[1],-np.pi/2)
    # assert alpha[1] == -11*np.pi/12
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=1],np.array([]))
    # assert np.allclose(angle_scale[1:2],np.array([1]))

# basic test of vector geometry
def test_robots_direction_calc_1():

    heading = np.pi/2
    pts_left =  [[np.random.randint(-100,-1), np.random.randint(-100, 100)] for i in range(5)]
    pts_right = [[np.random.randint(1,100),   np.random.randint(-100, 100)] for i in range(5)]
    c = [[0, 0]]
    pts = np.array(c + pts_left + pts_right)

    rind, lind = utils.pts_left_right(heading, pts)

    rpts = np.array(pts[rind])
    lpts = np.array(pts[lind])

    assert np.array_equal(np.sort(rpts), np.sort(np.array(c+pts_right)))
    assert np.array_equal(np.sort(lpts), np.sort(np.array(pts_left)))

# basic test of direction calc with robot context
def test_robots_direction_calc_2():
    robots = test_robots_init()

    coords = np.array([[0,0],[100,100],[-100,100]])
    robots.coords = coords
    angles = np.array([np.pi/2, np.pi/2, np.pi/2])
    lights = np.array([255,255,255])
    robots.angles = angles
    c = np.array([0,0])
    heading = np.pi/2 # heading of egocentric robot straight "ahead" up y axis
    r = 0

    diff = coords - c
    inv_distances, valid_dist_ind = robots.distance_calc(diff, r, robots.lim_distance)
    angle_scale, valid_angle_ind = robots.angle_calc(r, robots.lim_angle, coords, angles)
    print(valid_angle_ind)
    influence_scale = 1

    right_tot, left_tot, valid_ind = robots.direction_calc(heading, diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, lights)

    print(right_tot)
    print(inv_distances[2],angle_scale[2])
    assert np.array_equal(right_tot,inv_distances[2]*angle_scale[2])
    assert np.array_equal(left_tot,inv_distances[1]*angle_scale[1])
    assert np.array_equal(valid_ind, np.array([1,2]))

# TODO: implement
def test_robots_direction_calc_3():
    pass

# TODO: implement
def test_robots_direction_calc_4():
    pass

# TODO: implement
def test_robots_direction_calc_5():
    pass



######### run tests ##############

print("starting utils tests")
test_utils_wrap_pt_1()
test_utils_wrap_pt_2()
print("passed wrap_pt tests")

test_utils_load_global_config()
test_utils_load_robot_config()
test_utils_load_behavior_config()
print("passed load_config tests")

test_utils_setup_big_arrays_1()
test_utils_setup_big_arrays_2()
print("passed big_array tests")

test_utils_bounce_1()
test_utils_bounce_2()
test_utils_bounce_3()
test_utils_inpoly_1()
test_utils_inpoly_2()
print("passed utils tests\n")

print("starting robots tests")
test_robots_init()
test_robots_distance_calc_1() # distance calcs are wrong now that I changed dist calc...TODO: fix this
test_robots_distance_calc_2()
test_robots_distance_calc_3()
test_robots_coords_above_below()
test_robots_valid_angle_1()
test_robots_valid_angle_2()
test_robots_valid_angle_3()
test_robots_valid_angle_4()
test_robots_angle_calc_1()
test_robots_angle_calc_2()
test_robots_angle_calc_3()
test_robots_angle_calc_4()
test_robots_angle_calc_5()
test_robots_angle_calc_6()
test_robots_more_angle_tests_1()
test_robots_more_angle_tests_2()
test_robots_more_angle_tests_3()
test_robots_more_angle_tests_4()
test_robots_more_angle_tests_5()
test_robots_more_angle_tests_6()
test_robots_more_angle_tests_7()
test_robots_more_angle_tests_8()
test_robots_more_angle_tests_9()
test_robots_more_angle_tests_10()
test_robots_more_angle_tests_11()
test_robots_more_angle_tests_12()
test_robots_more_angle_tests_13()
test_robots_more_angle_tests_14()
test_robots_more_angle_tests_15()
test_robots_more_angle_tests_16()
test_robots_more_angle_tests_17()
test_robots_more_angle_tests_18()
test_robots_more_angle_tests_19()
test_robots_more_angle_tests_20()
test_robots_more_angle_tests_21()

# test_robots_direction_calc_2()
# test_robots_direction_calc_3()
# test_robots_direction_calc_4()
# test_robots_direction_calc_5()
print("passed all robots tests\n")

print("passed all tests")
