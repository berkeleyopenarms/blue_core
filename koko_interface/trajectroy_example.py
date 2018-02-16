#!/usr/bin/env python
from koko_interface import KokoInterface
import csv
import numpy as np
import time

if __name__== '__main__':

    reader = csv.reader(open("trajectory_vert.csv", "rb"), delimiter=",")
    x = list(reader)
    traj = np.array(x).astype("float")

    koko = KokoInterface('hekate.cs.berkeley.edu')
    while True:
        for col in traj.T:
            koko.set_joint_angle_target(col)
            print('New Joint Message Published')
            time.sleep(0.1)


