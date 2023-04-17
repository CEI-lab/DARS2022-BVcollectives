#! /usr/bin/python3
#
import numpy as np


class Environment():

    def __init__(self, ss, torus=True):
        self.ss = ss

    # generate a set of random starting positions for the robots in a subset of the env
    # in this case with hardcoded parameters for the two rooms sim
    def safeStartTwoRooms(self, num):
        first_half = np.array((np.random.choice([x for x in range(20,235)],num),
                               np.random.choice([x for x in range(20,480)],num))).T
        return first_half
