#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt

# Modify font size for all matplotlib plots.
plt.rcParams.update({'font.size': 9.1})

#
# Customize the following parameters...
#

LIVE = False
BASE_NAME = "data/"
TITLE = 'Number of Robots'


ARENA_NAMES = [ '' ]
ARENA_LONG_NAMES = { '':''
                   , "normal":"Fancy Normal"
                   }

EXPERIMENT_NAMES = [#'bed_0'
                    #'bed_100'
                    #, 'bed_200'
                   #]
                    'numRobots_5'
                    , 'numRobots_10'
                    , 'numRobots_25'
                    , 'numRobots_50'
                    , 'numRobots_75'
                    , 'numRobots_100'
                    , 'numRobots_125'
                    ]

EXPERIMENT_LONG_NAMES = { '':''
                        , 'numRobots_5':'5'
                        , 'numRobots_10':'10'
                        , 'numRobots_25':'25'
                        , 'numRobots_50':'50'
                        , 'numRobots_75':'75'
                        , 'numRobots_100':'100'
                        , 'numRobots_125':'125'
                        , "sensors_5_plus_1":"Arc: 5 + 1"
                        , "sensorConfig_arc_3":"Arc: 3 + 1"
                        , "sensorConfig_farline_3":"Far Line: 3 + 1"
                        , "sensorConfig_nearline_3":"Near Line: 3 + 1"
                        , "nearline_5":"Near Line: 5 + 1"
                        } 

COLUMNS_OF_INTEREST = ['eval']
COLUMN_NAMES = ['time', 'eval', 'propSlowed', 'cumEval', 'qTableCoverage', 'avgState']
COLUMN_LONG_NAMES = { "time":"Time"
                    , "eval":"Avg. Travel Time"
                    , "propSlowed":"Prop. Slowed"
                    , "cumEval":"Cum. Eval."
                    , "qTableCoverage":"Q Table Coverage"
                    , "avgState":"State"
                    }

if LIVE:
    X_LABEL = "Time (secs)"
else:
    X_LABEL = "Time (steps)"
SAVE_FIG = False
START_TRIAL = 0
LAST_TRIAL = 29
PLOT_TRIALS = False
PLOT_MEAN = True
COLLAPSE_EXPERIMENTS = True

def main():
    if COLLAPSE_EXPERIMENTS:
        fig, axes = plt.subplots(len(COLUMNS_OF_INTEREST), len(ARENA_NAMES), sharex='col', sharey='row', squeeze=False)
        #fig, axes = plt.subplots(len(COLUMNS_OF_INTEREST), len(ARENA_NAMES), sharex='col', squeeze=False)
    else:
        fig, axes = plt.subplots(len(EXPERIMENT_NAMES * len(COLUMNS_OF_INTEREST)), len(ARENA_NAMES), sharex='col', sharey='row', squeeze=False)

    subplot_column = 0
    for arena_name in ARENA_NAMES:
        plots_for_arena(axes, arena_name, subplot_column)
        subplot_column += 1

    # Add headers for subplot columns (if more than one used).
    if len(ARENA_NAMES) > 1:
        for col in range(len(ARENA_NAMES)):
            axes[0, col].set_title(get_arena_long_name(ARENA_NAMES[col]))

    if len(TITLE) > 0:
        fig.suptitle(TITLE)

    # Make more space between subplot rows
    #plt.subplots_adjust(hspace=0.45)

    plt.show()
    if SAVE_FIG:
        filename = "{}.pdf".format(BASE_NAME)
        fig.savefig(filename, bbox_inches='tight')

def plots_for_arena(axes, arena_name, subplot_column):
    axes_index = 0

    if COLLAPSE_EXPERIMENTS:
        for j in range(len(COLUMNS_OF_INTEREST)):
            for i in range(len(EXPERIMENT_NAMES)):
                plot(axes[axes_index, subplot_column], arena_name, EXPERIMENT_NAMES[i], COLUMNS_OF_INTEREST[j])
            axes_index += 1
    else:
        for i in range(len(EXPERIMENT_NAMES)):
            for j in range(len(COLUMNS_OF_INTEREST)):
                plot(axes[axes_index, subplot_column], arena_name, EXPERIMENT_NAMES[i], COLUMNS_OF_INTEREST[j])
                axes_index += 1

    axes[-1, subplot_column].set_xlabel(X_LABEL)

def get_arena_long_name(arena_name):
    if arena_name in ARENA_LONG_NAMES.keys():
       return ARENA_LONG_NAMES[arena_name]
    else:
        return arena_name
    
def get_experiment_long_name(experiment):
    if experiment in EXPERIMENT_LONG_NAMES.keys():
        return EXPERIMENT_LONG_NAMES[experiment]
    else:
        return experiment

def get_column_long_name(column_name):
    if column_name in COLUMN_LONG_NAMES.keys():
        return COLUMN_LONG_NAMES[column_name]
    else:
        return column_name
    
def plot(axes, arena_name, experiment, column_of_interest):
        
    axes.axhline(y=0, color='k', linewidth=0.5)

    #if column_of_interest == 'cumPropSlowed':
    #    axes.set_aspect(1.0)
    #elif LIVE:
    #    axes.set_aspect(4000.0)
    #else:
    #    axes.set_aspect(250000.0)

    all_dfs = []
    for trial in range(START_TRIAL, LAST_TRIAL + 1):
        df = dataframe_per_trial(axes, arena_name, experiment, column_of_interest, trial)
        all_dfs.append(df)

    df_concat = pd.concat(all_dfs)
    by_row_index = df_concat.groupby(df_concat.index)
    if PLOT_MEAN:
        by_row_index.mean().plot(ax=axes, x='time', y=column_of_interest, label=get_experiment_long_name(experiment), linewidth=1)#, color='black')

    axes.set_ylabel(get_column_long_name(column_of_interest))

    handles, labels = axes.get_legend_handles_labels()
    axes.legend(handles, labels)

    if not COLLAPSE_EXPERIMENTS:
        axes.get_legend().remove()

    # Show the legend in lower right and reverse its order
    #axes.legend(handles[::-1], labels[::-1], loc=1)

def dataframe_per_trial(axes, arena_name, experiment, column_of_interest, trial):

    #filename = "{}/{}/{}/{}/stats_{}.dat".format(BASE_NAME, arena_name, experiment, ROBOT, trial)
    filename = BASE_NAME + "/"
    if len(arena_name) > 0:
        filename += arena_name + "/"
    if len(experiment) > 0:
        filename += experiment + "/"
    #if LIVE:
    #    filename += "r4/"
    filename += "stats_{}.dat".format(trial)

    print("Loading: {}".format(filename))
    dataframe = pd.read_csv(filename, " ")

    dataframe.columns = COLUMN_NAMES

    if PLOT_TRIALS:
        #label = "Exp: {}, Trial: {}".format(experiment, trial)
        label = "{}".format(trial)
        dataframe.plot(ax=axes, x='time', y=column_of_interest, label=label, linewidth=0.5)
        axes.title.set_text(get_experiment_long_name(experiment))

    #print(dataframe.info())

    return dataframe

main()
