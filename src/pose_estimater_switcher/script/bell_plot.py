import numpy as np
import math
import matplotlib.pyplot as plt
from pathlib import Path

output_path = str(Path.home()) + '/' + "figure/1_filtered_data/"

plt.style.use('fivethirtyeight')
fig1 = plt.figure(figsize=(15,10))
fig1.suptitle('Gaussian density curve',fontsize=20)
ax1 = fig1.add_subplot(1,1,1)
ax1.set_xlabel("Occlusion height [m]",fontsize=16)
ax1.set_ylabel("Likelihood", fontsize=16)

def main():
    STD = 0.7887537272
    mean = 1.464210526
    i = np.array([0])
    c = np.array([0])
    for j in range(0, 4500):
        x=j/1000
        y = 1/(STD * math.sqrt(2*math.pi))*math.exp(-pow((x-mean),2)/(2*pow(STD,2)))
        c = np.append(c,y)
        i = np.append(i,x)

    ax1.plot(i, c)
    name_of_file_1 = 'Bell_curve_plot.png'


    fig1.savefig(output_path + name_of_file_1,transparent=True)

if __name__ == '__main__':
    main()
