#!/usr/bin/env python3
##### import of libs #####
import rospy
import os
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

##### setting up the path for the data file and folder #####
path = str(Path.home().joinpath("test_data", "static_log%s.csv"))
path_output = str(Path.home().joinpath("test_data", "static_output%s.csv"))
folder_path = str(Path.home().joinpath("test_data"))


def main():
    global path, folder_path
    number_of_files = 0

    while os.path.exists(path % number_of_files):
        local_path = path %  number_of_files
        data = pd.read_csv(path)
        with_x = data["with_x"]
        with_y = data["with_y"]
        with_v = ["with_v"]
        with_hx = ["with_hx"]
        with_hy = ["with_hy"]
        without_x = ["without_x"]
        without_y = ["without_y"]
        without_v = ["without_v"]
        without_hx = ["without_hx"]
        without_hy = ["without_hy"]
        acc_x = ["acc_x"]
        acc_y = ["acc_y"]
        v = ["acc_z"]
        gyro_x = ["gyro_x"]
        gyro_y = ["gyro_y"]
        gyro_z = ["gyro_z"]
        ID = ["ID"]
        distance = ["distance"]
        linear_speed = ["linear_speed"]
        angular_speed = ["angular_speed"]

        number_of_files = number_of_files + 1



if __name__ == '__main__':
    main()
