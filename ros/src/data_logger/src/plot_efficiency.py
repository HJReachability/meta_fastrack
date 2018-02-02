#!/usr/bin/env python2.7

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

if __name__ == '__main__':

    path = os.getcwd() + "/../data/coffee"

    adapt_safe = np.array([None]*16)
    adapt_eff = np.array([None]*16)
    irr_safe = np.array([None]*16)
    irr_eff = np.array([None]*16)
    rat_safe = np.array([None]*16)
    rat_eff = np.array([None]*16)
    ai = 0; ri = 0; ii = 0

    for filename in os.listdir(path):
        if "metrics" in filename and "p0" not in filename:
            with open(path+"/"+filename) as f:
                line = f.readlines()[0]
                data = line.split(" ")
                if "adaptive" in filename:
                    adapt_safe[ai] = float(data[1])
                    adapt_eff[ai] = float(data[2])
                    ai += 1
                elif "rational" in filename and "irrational" not in filename:
                    rat_safe[ri] = float(data[1])
                    rat_eff[ri] = float(data[2])
                    ri += 1
                elif "irrational" in filename:
                    irr_safe[ii] = float(data[1])
                    irr_eff[ii] = float(data[2])
                    ii += 1

    # fake up some data

    diff_irr_eff = adapt_eff - irr_eff
    diff_rat_eff = adapt_eff - rat_eff

    diff_irr_safe = adapt_safe - irr_safe
    diff_rat_safe = adapt_safe - rat_safe

    # basic plot
    """
    plt.figure()
    plt.title("Efficiency: Adaptive - Irrational")
    plt.boxplot(diff_irr_eff)
    plt.figure()
    plt.title("Efficiency: Adaptive - Rational")
    plt.boxplot(diff_rat_eff)
    """

    """
    fig = plt.figure()
    ax = fig.add_subplot(111)
    data = [adapt_safe, rat_safe, irr_safe]

    plt.title("Safety")
    plt.boxplot(data, patch_artist=True, notch=True, bootstrap=10000)
    plt.axhline(y=0, color="grey", linewidth=1)
    plt.grid(color="grey", axis="y", linewidth=1,  linestyle="-")

    # fill with colors
    colors = ['pink', 'lightblue', 'lightgreen']
    for bplot in (bplot1, bplot2):
        for patch, color in zip(bplot['boxes'], colors):
            patch.set_facecolor(color)

    ax.spines['bottom'].set_color('grey')
    ax.spines['top'].set_color('grey')
    ax.spines['right'].set_color('grey')
    ax.spines['left'].set_color('grey')

    plt.xlim([0,4])
    plt.ylim([-0.5,2.5])
    """
    blackC = "black"    #(214/255., 39/255., 40/255.)
    greyC = (125/255.0, 125/255.0, 125/255.0, 1.0) #"grey"      #(44/255., 160/255., 44/255.)
    blueC = "#4BABC5"   #(31/255., 119/255., 180/255.)
    orangeC = (247/255.0, 149/255.0, 69/255.0, 1.0) #"#F79545" #(255/255., 127/255., 14/255.)
    darkOrange = "#ea6900"
    darkGrey = (62/255.0, 137/255.0, 183/255.0, 1.0) #"#487a99"


    font = {'family' : 'palatino', 'weight' : 'bold', 'size' : 22, 'style' : 'normal'}
    text = {'usetex' : 'true'}
    matplotlib.rc('text', usetex=True)
    matplotlib.rc('font', **font)
    matplotlib.rc('axes', linewidth=2.5, titlepad=12, labelpad=12)
    matplotlib.rc('hatch', linewidth=4)
    #    matplotlib.rc('xtick', color=greyC)
#    matplotlib.rc('ytick', color=greyC)

#    data = [adapt_safe, rat_safe, irr_safe]
#    labels = [r'$\beta$ inference', r'$\beta$ high confidence', r'$\beta$ low confidence']

    data = [diff_rat_eff, diff_irr_eff]
    labels= [r'$T_{\textnormal{infer}} - T_{\textnormal{lo}}$', r'$T_{\textnormal{infer}} - T_{\textnormal{hi}}$']

    fig, axes = plt.subplots(nrows=1, ncols=1) #figsize=(9, 4))
    # adding horizontal grid lines
    axes.yaxis.grid(True,  color="grey", alpha=0.5, linewidth=2.5, linestyle="-")
    axes.tick_params(length=6, width=2.5, color=greyC)

    medianprops = dict(linestyle='-', linewidth=2.5, color='firebrick')
    boxprops = dict(linestyle='--', linewidth=2.5, color='black')
    flierprops = dict(marker='o', markerfacecolor=greyC, markersize=8, linestyle='none')
    whiskerprops = dict(linestyle='-', linewidth=2.5, color='black')
    capprops = dict(linestyle='-', linewidth=2.5, color='black')

    # rectangular box plot
    bplot1 = axes.boxplot(data,
                          bootstrap=10000,
                          notch=True,
                          vert=True,  # vertical box alignment
                          patch_artist=True,  # fill with color
                          labels=labels,
                          medianprops=medianprops,
                          boxprops=boxprops,
#                          flierprops=flierprops,
                          whiskerprops=whiskerprops,
                          capprops=capprops,
    )

    plt.setp(bplot1['fliers'][0], markerfacecolor=darkGrey, linewidth=2.5, markersize=8, linestyle='none')
    plt.setp(bplot1['fliers'][1], markerfacecolor=greyC, linewidth=2.5, markersize=8, linestyle='none')
    bplot1['boxes'][0].set(hatch='/', color=orangeC)
    bplot1['boxes'][1].set(hatch='/', color=orangeC)

#    plt.setp(bplot1['fliers'][0], markerfacecolor=orangeC, linewidth=2.5, markersize=8, linestyle='none')
#    plt.setp(bplot1['fliers'][1], markerfacecolor=darkGrey, linewidth=2.5, markersize=8, linestyle='none')
#    plt.setp(bplot1['fliers'][2], markerfacecolor=greyC, linewidth=2.5, markersize=8, linestyle='none')

    # will be used to label x-ticks
#    axes.set_title(r'\textbf{Safety Comparison for Coffee-Avoiding Human}')
#    axes.set_ylabel(r'$\min_{t}~\|x_H^t - x_R^t\|_2$~(m)')

    axes.set_title(r'\textbf{Efficiency Comparison for Coffee-Avoiding Human}')
    axes.set_ylabel(r"Robot's time to reach goal~(s)")

    # fill with colors
#    colors = [orangeC, darkGrey, greyC]
    colors = [darkGrey, greyC]

    colors2 = [darkOrange, "black", darkGrey]
    for patch, color, outline in zip(bplot1['boxes'], colors, colors2):
        patch.set_facecolor(color)
        patch.set_linewidth(2)

    axes.spines['bottom'].set_color('grey')
    axes.spines['top'].set_color('grey')
    axes.spines['right'].set_color('grey')
    axes.spines['left'].set_color('grey')
    #ax.set_xlabel('Three separate samples')
    #ax.set_ylabel('Observed values')

    plt.show()
