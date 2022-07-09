import numpy as np
import matplotlib.pyplot as plt
import yaml

import utils
import robots as rb

# test if points within the range with no offset remain unchanged
def test_utils_wrap_pt_1():
    pt = [0,0]
    w = 500
    h = 500
    offset = (0,0)
    assert utils.wrap_pt(pt,w,h,offset) == [0,0], "wrap_pt test1 failed"

    pt = [100,0]
    w = 500
    h = 500
    offset = (0,0)
    assert utils.wrap_pt(pt,w,h,offset) == [100,0], "wrap_pt test1 failed"

    pt = [250,250]
    w = 500
    h = 500
    offset = (0,0)
    assert utils.wrap_pt(pt,w,h,offset) == [250,250], "wrap_pt test1 failed"

    pt = [499,300]
    w = 500
    h = 500
    offset = (0,0)
    assert utils.wrap_pt(pt,w,h,offset) == [499,300], "wrap_pt test1 failed"

# test if points outside of the range get wrapped correctly with no offset
def test_utils_wrap_pt_2():
    pt = [600,0]
    w, h = 500, 500
    offset = (0,0)
    new_pt = utils.wrap_pt(pt,w,h,offset)
    assert new_pt == [100,0], "wrap_pt test2 gave"+str(new_pt)+"instead of [100, 0]"

    pt = [0,600]
    offset = (0,0)
    new_pt = utils.wrap_pt(pt,w,h,offset)
    assert new_pt == [0,100], "wrap_pt test2 gave"+str(new_pt)+"instead of [0, 100]"

    pt = [-300,0]
    offset = (0,0)
    new_pt = utils.wrap_pt(pt,w,h,offset)
    assert new_pt == [200,0], "wrap_pt test2 gave"+str(new_pt)+"instead of [200, 0]"

    pt = [w,-100]
    offset = (0,0)
    new_pt = utils.wrap_pt(pt,w,h,offset)
    assert new_pt == [w,400], "wrap_pt test2 gave"+str(new_pt)+"instead of [500, 400]"

# world:
#   sim_time: 200
#   screen_size: 500

# robot:
#   num_robots: 20
#   robot_vel: 4
#   lim_distance: 100
#   lim_angle: 0.25
#   angle_incr: 0.01
#   rep_range: 0
#   rep_factor: 3
#   lightscale: 1

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

    sim_time, ss = utils.load_global_config('configs/test_config.yaml')

    assert sim_time == 200, "load_global_config test1 failed"
    assert ss == 500, "load_global_config test1 failed"

# TODO: implement
def test_utils_load_robot_config():
    pass

# TODO: implement
def test_utils_load_configuration_config():
    pass

# TODO: implement
def test_utils_setup_big_arrays_1():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/4,np.pi/4])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

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

# TODO: implement
def test_robots_init():
    pass

# test basic calc
def test_robots_distance_calc_1():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)
    robots.lim_distance = 200

    coords = np.array([[0,0],[100,0],[150,0]])
    c = np.array([0,0])

    diff = coords - c
    inv_distances, valid_dist_ind = robots.distance_calc(diff, 0, robots.lim_distance)

    assert np.array_equal(inv_distances,np.array([0.,0.75,0.4375]))
    assert np.array_equal(valid_dist_ind,np.array([1,2]))
    
# test robot out of bounds
def test_robots_distance_calc_2():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss)
    robots.lim_distance = 200

    coords = np.array([[0,0],[50,0],[100,0],[400,0],[0,200]])
    c = np.array([0,0])

    diff = coords - c
    inv_distances, valid_dist_ind = robots.distance_calc(diff, 0, robots.lim_distance)

    assert np.array_equal(inv_distances,np.array([0,0.9375,0.75,-3,0]))
    assert np.array_equal(valid_dist_ind,np.array([1,2]))

# test distnace in different direction, nonzero starting value
def test_robots_distance_calc_3():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)
    robots.lim_distance = 200

    coords = np.array([[100,0],[150,0],[175,0],[500,0],[0,0]])
    c = np.array([100,0])

    diff = coords - c
    inv_distances, valid_dist_ind = robots.distance_calc(diff, 0, robots.lim_distance)

    assert np.array_equal(inv_distances,np.array([0,0.9375,0.859375,-3,0.75]))
    assert np.array_equal(valid_dist_ind,np.array([1,2,4]))

# TODO: implement, test distnace around torus
def test_robots_distance_calc_4():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)


    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)
    robots.lim_distance = 200

    robots.coords = np.array([[0,0],[100,0],[150,0]])
    c = np.array([0,0])

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    diff = big_coords - c
    inv_distances, valid_dist_ind = robots.distance_calc(diff, 0, robots.lim_distance)

    # assert np.array_equal(inv_distances,np.array([0.,0.75,0.4375]))
    # assert np.array_equal(valid_dist_ind,np.array([1,2]))


# test calc on normal, directional light, headings only
def test_robots_angle_calc_1():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)

    # setup test parameters
    robots.coords = np.array([[0,0],[0,10],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/4,0])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    angle_scale, valid_angle_ind = robots.angle_calc(0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(valid_angle_ind)

    assert np.allclose(angle_scale[0:2],np.array([0,0.75])) # this is necessary so that I can input 1/3 and it works instead of matching the exact number of 3's in 0.33333
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1]))

# test lantern light
def test_robots_angle_calc_2():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)

    # setup test parameters
    robots.coords = np.array([[0,0],[0,10],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/4,0])
    robots.lights = np.array([255,255,255])
    robots.directional_light = 0
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    angle_scale, valid_angle_ind = robots.angle_calc(0, robots.lim_angle, big_coords, big_angles)

    assert np.array_equal(angle_scale,np.full(robots.num*5,1))
    assert np.array_equal(valid_angle_ind,np.arange(0,robots.num*5),1)

# test directional same heading coordinate only
def test_robots_angle_calc_3():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5],[5,5*np.sqrt(3)]])
    robots.angles = np.array([np.pi/2,np.pi/2,np.pi/2])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    # angles are 0, 45, 60
    angle_scale, valid_angle_ind = robots.angle_calc(0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(valid_angle_ind)

    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.allclose(angle_scale[0:3],np.array([0,0.75,8/9])) 
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1,2]))

# test directional heading and coordinate
def test_robots_angle_calc_4():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5],[5,5*np.sqrt(3)]])
    robots.angles = np.array([np.pi/2,7*np.pi/12,5*np.pi/12])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    # coord angles are 0, pi/4, pi/3, heading angles pi/12, -pi/12
    angle_scale, valid_angle_ind = robots.angle_calc(0, robots.lim_angle, big_coords, big_angles)

    # print(angle_scale)
    # print(valid_angle_ind)

    assert np.allclose(angle_scale[0:3],np.array([0,5/9,35/36]))
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1,2]))

# test coords with wrapping
def test_robots_angle_calc_5():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss, 0)

    # setup test parameters
    robots.coords = np.array([[0,0],[10,0],[0,10]])
    robots.angles = np.array([np.pi/2,np.pi/2,np.pi/2])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    angle_scale, valid_angle_ind = robots.angle_calc(0, robots.lim_angle, big_coords, big_angles)

    assert np.array_equal(valid_angle_ind,np.array([2,5,6,7,8,11]))

# test directional same heading coordinate only, with different starting angle
def test_robots_angle_calc_6():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss)

    # setup test parameters
    robots.coords = np.array([[0,0],[5,5],[5,5*np.sqrt(3)]])
    robots.angles = np.array([np.pi/4,np.pi/4,np.pi/4])
    robots.lights = np.array([255,255,255])
    robots.lim_angle = np.pi/2
    robots.directional_light = 1
    robots.num = 3

    big_coords, big_lights, big_angles = utils.setup_big_arrays(robots, ss)

    # angles are 0, 45, 60
    angle_scale, valid_angle_ind = robots.angle_calc(0, robots.lim_angle, big_coords, big_angles)
    
    # print(angle_scale)
    # print(valid_angle_ind)

    # assert np.allclose(angle_scale[0:3],np.array([0,1,2])) 
    assert np.allclose(angle_scale[0:3],np.array([0,1,35/36])) 
    assert np.array_equal(valid_angle_ind[valid_angle_ind<=2],np.array([1,2]))

# basic test of vector geometry
def test_robots_direction_calc_1():

    heading = np.pi/2
    pts_left =  [[np.random.randint(-100,-1), np.random.randint(-100, 100)] for i in range(5)]
    pts_right = [[np.random.randint(1,100),   np.random.randint(-100, 100)] for i in range(5)]
    c = [[0, 0]]
    pts = np.array(c + pts_left + pts_right)

    rind, lind = utils.pts_left_right(heading, pts)

    rpts = np.array([pts[i] for i in rind])
    lpts = np.array([pts[i] for i in lind])

    assert np.array_equal(np.sort(rpts), np.sort(np.array(c+pts_right)))
    assert np.array_equal(np.sort(lpts), np.sort(np.array(pts_left)))

# basic test of direction calc with robot context
def test_robots_direction_calc_2():
    global_filename = "configs/global_config.yaml"
    c_filename = "configs/aggression_config.yaml"

    sim_time, ss = utils.load_global_config(global_filename)

    # init robots
    robots = rb.robot_class(global_filename, c_filename, ss)
    robots.lim_distance = 200

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
    influence_scale = 1

    right_tot, left_tot, valid_ind = robots.direction_calc(heading, diff, valid_angle_ind, valid_dist_ind, inv_distances, angle_scale, influence_scale, lights)

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
test_utils_load_configuration_config()
print("passed load_config tests")

test_utils_setup_big_arrays_1()
test_utils_setup_big_arrays_2()
print("passed big_array tests")

test_utils_bounce_1()
test_utils_inpoly_1()
test_utils_inpoly_2()
print("passed utils tests\n")

print("starting robots tests")
test_robots_init()
test_robots_distance_calc_1() # distance calcs are wrong now that I changed dist calc...TODO: fix this
test_robots_distance_calc_2()
test_robots_distance_calc_3()
test_robots_angle_calc_1()
test_robots_angle_calc_2()
test_robots_angle_calc_3()
test_robots_angle_calc_4()
test_robots_angle_calc_5()
test_robots_angle_calc_6()
test_robots_direction_calc_1()
test_robots_direction_calc_2()
test_robots_direction_calc_3()
test_robots_direction_calc_4()
test_robots_direction_calc_5()
print("passed all robots tests\n")

print("passed all tests")
