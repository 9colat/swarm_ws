#!/usr/bin/env python3
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethiryeight')
path = Path.home().joinpath("test_data", "pose.csv")
x_vals = []
y_vals = []

index = count()

def animate(i):
    data = pd.read_csv(path)
    x = data["x - coordinate"]
    y = data["y - coordinate"]
    z = data["z - coordinate"]

    plt.cla()

    plt.plot(x, y,)
    plt.tight_layout()

ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()
