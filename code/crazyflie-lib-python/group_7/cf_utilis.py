import os
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')

# live plotter for the position of the drone, and use `cm` as unit
def live_plotter(x_data, y_data, pos, pause_time, x = 500, y = 300, region_x = 120, zone_anno=True):

    xticks = np.arange(0, x, 50)
    yticks = np.arange(0, y, 50)

    if zone_anno:
        start = patches.Rectangle((0, 0), region_x, y, linewidth=1, edgecolor='blue', facecolor='blue', alpha=0.5)
        goal = patches.Rectangle((x-region_x, 0), region_x, y, linewidth=1, edgecolor='green', facecolor='green', alpha=0.5)

    if pos==[]:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(x/y*6, 6))
        ax = fig.add_subplot(111)
        # create a variable for the pos so we can later update it
        pos, = ax.plot(x_data, y_data, alpha=0.8, linewidth=3)
        # update plot label/title
        plt.xlabel('x position [cm]')
        plt.ylabel('y position [cm]')

        plt.xlim([0, x+1])
        plt.ylim([0, y+1])

        plt.xticks(np.arange(0, x+1, 20))
        plt.yticks(np.arange(0, y+1, 20))
        if zone_anno:
            ax.add_patch(start)
            ax.add_patch(goal)

        plt.title('Real time visualization')
        plt.show()

    # after the figure, axis, and pos are created, we only need to update the next position
    pos.set_data(x_data, y_data)

    # this pauses the data so the figure/axis can catch up
    plt.pause(pause_time)

    # return pos so we can update it again in the next iteration
    return pos

# overall plotting code after experiments
def plot_xy_traj(x_log_filename, y_log_filename, figname, example_folder='./', zone_anno=False, overall_x=480, overall_y=120, region_x=120):
    import seaborn as sns
    sns.set_theme()

    robot_x = pd.read_csv(x_log_filename, header=None, names=['X'])
    robot_y = pd.read_csv(y_log_filename, header=None, names=['Y'])

    # discard first row
    robot_x = robot_x[1:]
    robot_y = robot_y[1:]

    # axes instance
    ratio = overall_x/overall_y
    fig, ax = plt.subplots(figsize=(3*ratio, 4))

    # plot the trajectory
    ax.plot(robot_x['X'].values, robot_y['Y'].values,
            color="coral", linewidth=2)

    # plot the starting and landing region
    if zone_anno:
        start_region = patches.Rectangle(
            (0, 0), region_x, overall_y, linewidth=1, edgecolor='blue', facecolor='blue', alpha=0.2)
        land_region = patches.Rectangle((overall_x-region_x, 0), region_x, overall_y, linewidth=1,
                                        edgecolor='green', facecolor='green', alpha=0.2)
        ax.add_patch(start_region)
        ax.add_patch(land_region)

    # range
    ax.set_xlim([0, math.floor(overall_x/100)*100])

    # title and label
    ax.set_title("Crazyfile trajectory", fontsize=15)
    _ = ax.set_xlabel("Estimated X (cm)")
    _ = ax.set_ylabel("Estimated Y (cm)")
    ax.axis('equal')

    # save
    fig.tight_layout()
    plt.savefig(example_folder+figname + ".png", dpi=300)  # png, pdf

## make a dir just as in Linux
def mkdir(path):
    path=path.strip()
    path=path.rstrip("\\")
    isExists=os.path.exists(path)
    if not isExists:
        os.makedirs(path)
        print(path+' created successfully')
    else:
        print(path+' already exists')
