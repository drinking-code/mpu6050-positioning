import json
import matplotlib.pyplot as plt

plt.style.use('ggplot')
plt.ion()
fig, axs = plt.subplots(3, 3, figsize=(32, 12))

line_width = 1

with open('output.json', 'r') as outfile:
    data = json.load(outfile)

    names = ['X', 'Y', 'Z']

    for i in range(0, 3):
        axs[0][i].plot(data['timestamp'], list(map(lambda x: x[i], data['acceleration'])),
                       label="Acceleration " + names[i],
                       color=plt.cm.Set1(0), linewidth=line_width)
        axs[1][i].plot(data['timestamp'], list(map(lambda x: x[i], data['velocity'])),
                       label="Velocity " + names[i],
                       color=plt.cm.Set1(1), linewidth=line_width)
        axs[2][i].plot(data['timestamp'], list(map(lambda x: x[i], data['position'])),
                       label="Position " + names[i],
                       color=plt.cm.Set1(2), linewidth=line_width)

        [axs[ii][i].legend() for ii in range(0, len(axs))]
        axis_min_limit = .5
        y_lim_accel_bottom, y_lim_accel_top = axs[0][i].get_ylim()
        if y_lim_accel_bottom > -axis_min_limit:
            axs[0][i].set_ylim(bottom=-axis_min_limit)
        if y_lim_accel_top < axis_min_limit:
            axs[0][i].set_ylim(top=axis_min_limit)

        axs[0][i].set_ylabel('Acceleration [m$\cdot$s$^{-2}$]', fontsize=16)
        axs[1][i].set_ylabel('Velocity [m$\cdot$s$^{-1}$]', fontsize=16)
        axs[2][i].set_ylabel('Displacement [m]', fontsize=16)
        axs[2][i].set_xlabel('Time [s]', fontsize=18)

plt.show()
