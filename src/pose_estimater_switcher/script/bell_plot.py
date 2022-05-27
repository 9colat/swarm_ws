import numpy as np
import math
import matplotlib.pyplot as plt

plt.style.use('fivethirtyeight')
fig1 = plt.figure(figsize=(15,10))
fig1.suptitle('Bell curve')
ax1 = fig1.add_subplot(1,1,1)
ax1.set_xlabel("Occlusion height [m]",fontsize=13)
ax1.set_ylabel("Likelihood", fontsize=13)

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
    plt.show()

if __name__ == '__main__':
    main()
