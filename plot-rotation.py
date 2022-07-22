import json
import matplotlib.pyplot as plt

plt.style.use('ggplot')
plt.ion()
fig, axs = plt.subplots(1, 3, figsize=(32, 12))

line_width = 1

with open('output_rotation.json', 'r') as outfile:
    data = json.load(outfile)

    names = ['X', 'Y', 'Z']

    for i in range(0, 3):
        axs[i].plot(data['timestamp'], list(map(lambda x: x[i], data['rotation'])),
                       label="Rotation " + names[i],
                       color=plt.cm.Set1(0 if i == 0 else (1 if i == 2 else 2)), linewidth=line_width)
        axs[i].set_ylabel('Rotation [°]', fontsize=16)
        axs[i].set_xlabel('Time [s]', fontsize=18)

    [axs[ii].legend() for ii in range(0, len(axs))]
    # axis_min_limit = .5
    # y_lim_accel_bottom, y_lim_accel_top = axs[0][i].get_ylim()
    # if y_lim_accel_bottom > -axis_min_limit:
    #     axs[0].set_ylim(bottom=-axis_min_limit)
    # if y_lim_accel_top < axis_min_limit:
    #     axs[0].set_ylim(top=axis_min_limit)

plt.show()
