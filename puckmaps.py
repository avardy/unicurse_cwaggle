#!/usr/bin/env python

"""
Plots the positions of pucks on top of the obstacles image for specific trials.
"""

import cv2
import pandas as pd
import matplotlib.pyplot as plt

# Modify font size for all matplotlib plots.
plt.rcParams.update({'font.size': 9.1})

#
# Customize the following parameters...
#

LIVE = False
BASE_NAME = "data/"

BACKGROUND_IMAGE = '../images/4k_one_wall_goal_L_div_4/labyrinth_1000.png'

CONDITION_NAMES = [ 'numRobots_100' ]
CONDITION_LONG_NAMES = { "normal":"Unlimited Range"
                        , "puckSensorFOV_360":"Puck Sensor FOV: 360"
                        , "puckSensorFOV_180":"Puck Sensor FOV: 180"
                        , "puckSensorFOV_90":"Puck Sensor FOV: 90"
                        , "puckSensorFOV_45":"Puck Sensor FOV: 45"
                        }

PUCK_TYPE_COLORS = ['red']#, 'green']

BEST_TRIAL = True
WORST_TRIAL = False
SPECIFIC_TRIAL = False
SPECIFIC_TRIAL_NUMBER = 0

N_ROBOTS = 100
N_PUCKS = 100
N_PUCK_TYPES = 1
SAVE_FIG = True
START_TRIAL = 0
LAST_TRIAL = 9

def main():
    fig, axes = plt.subplots(1, len(CONDITION_NAMES), sharex='col', squeeze=False)

    subplot_column = 0
    for condition_name in CONDITION_NAMES:
        puckmap_for_condition(axes[0, subplot_column], condition_name)
        subplot_column += 1

    # Add headers for subplot columns (if more than one used).
    if len(CONDITION_NAMES) > 1:
        for col in range(len(CONDITION_NAMES)):
            axes[0, col].set_title(get_condition_long_name(CONDITION_NAMES[col]))

    plt.show()
    if SAVE_FIG:
        filename = "{}.pdf".format(BASE_NAME)
        fig.savefig(filename, bbox_inches='tight')

def get_condition_long_name(condition_name):
    if condition_name in CONDITION_LONG_NAMES.keys():
       return CONDITION_LONG_NAMES[condition_name]
    else:
        return condition_name

def puckmap_for_condition(axes, condition_name):

    chosen_puck_df_list = None
    chosen_robot_df = None
    best_score = float('inf')
    worst_score = 0
    for trial in range(START_TRIAL, LAST_TRIAL + 1):
        
        robot_df = dataframe_per_trial('robotPose', condition_name, trial)
        puck_df_list = []
        for puckType in range(N_PUCK_TYPES):
            puck_df_list.append(dataframe_per_trial('puckType_{}'.format(puckType), condition_name, trial))

        if SPECIFIC_TRIAL and trial == SPECIFIC_TRIAL_NUMBER:
            puckmap(axes, puck_df_list, robot_df, condition_name)

        stats_df = dataframe_per_trial('stats', condition_name, trial)

        # Get the score for the last row
        score = stats_df.at[ len(stats_df)-1, 1 ]
        if BEST_TRIAL and score < best_score:
            best_score = score
            chosen_puck_df_list = puck_df_list
            chosen_robot_df = robot_df
            print("Best trial so far: {}".format(trial))

        if WORST_TRIAL and score > worst_score:
            worst_score = score
            chosen_puck_df_list = puck_df_list
            chosen_robot_df = robot_df
            print("Worst trial so far: {}".format(trial))

    if BEST_TRIAL or WORST_TRIAL:
        puckmap(axes, chosen_puck_df_list, chosen_robot_df, condition_name)

def puckmap(axes, puck_df_list, robot_df, condition_name):
    #filename = 'images/{}/obstacles.png'.format(condition_name)
    #filename = 'images/sim_stadium_no_wall/labyrinth.png'.format(condition_name)

    #obstacles = cv2.imread(filename)
    obstacles = cv2.imread(BACKGROUND_IMAGE)
    obstacles = cv2.cvtColor(obstacles, cv2.COLOR_BGR2GRAY)
    axes.imshow(obstacles, cmap='gray', interpolation='none')
    #axes.imshow(obstacles)#, cmap='gray', interpolation='none')
    axes.axis('off')

    # Draw the pucks
    for puck_type in range(N_PUCK_TYPES):
        puck_df = puck_df_list[puck_type]
        row = len(puck_df) - 1 # We're always interested in the final row.
        for i in range(N_PUCKS):
            col = 1 + 2*i
            x = puck_df.at[row, col]
            y = puck_df.at[row, col + 1]
            circle = plt.Circle((x, y), 5, color='white')
            inner_circle = plt.Circle((x, y), 4, color=PUCK_TYPE_COLORS[puck_type])
            axes.add_patch(circle)
            axes.add_patch(inner_circle)

    # Draw the robots
    row = len(robot_df) - 1 # We're always interested in the final row.
    for i in range(N_ROBOTS):
        col = 1 + 3*i
        x = robot_df.at[row, col]
        y = robot_df.at[row, col + 1]
        circle = plt.Circle((x, y), 18, color='blue')
        axes.add_patch(circle)

    #for x, y in df:
    #df.plot(ax=axes, x='time', y=column_of_interest, label=label, linewidth=0.5)

def dataframe_per_trial(datatype, condition_name, trial):

    filename = BASE_NAME + "/"
    if len(condition_name) > 0:
        filename += condition_name + "/"
    filename += "{}_{}.dat".format(datatype, trial)

    print("Loading: {}".format(filename))
    dataframe = pd.read_csv(filename, " ")

    # The columns are arbitrarily labelled by the first row.  We overwrite to
    # just use integer column names.
    dataframe.columns = [i for i in range(len(dataframe.columns))]

    return dataframe

main()
