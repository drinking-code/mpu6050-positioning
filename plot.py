import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from main import listen_to_accel

data = None
matplotlib.use('TkAgg')


# function to update the data
def add_data(i):
    # get data
    if len(pos_x) > 20:
        pos_x.pop(0)
    pos_x.append(data['position'][0])
    # pos_x.append(i)

    if len(pos_y) > 20:
        pos_y.pop(0)
    pos_y.append(data['position'][1])
    # pos_y.append(i)

    # clear axis
    ax.cla()
    ax1.cla()
    # plot pos_x
    ax.plot(pos_x)
    ax.scatter(len(pos_x) - 1, pos_x[-1])
    ax.text(len(pos_x) - 1, pos_x[-1] + 2, "{}".format(pos_x[-1]))
    # ax.set_ylim(0, 100)
    # plot pos_y
    ax1.plot(pos_y)
    ax1.scatter(len(pos_y) - 1, pos_y[-1])
    ax1.text(len(pos_y) - 1, pos_y[-1] + 2, "{}".format(pos_y[-1]))
    # ax1.set_ylim(0, 100)


if __name__ == '__main__':
    data, stop = listen_to_accel()
    # start collections with zeros
    pos_x = []
    pos_y = []
    # define and adjust figure
    fig = plt.figure(figsize=(4, 2), facecolor='#DEDEDE')
    ax = plt.subplot(121)
    ax1 = plt.subplot(122)
    ax.set_facecolor('#DEDEDE')
    ax1.set_facecolor('#DEDEDE')
    # animate
    ani = FuncAnimation(fig, add_data, interval=5)
    print('showing plot')
    plt.show()
