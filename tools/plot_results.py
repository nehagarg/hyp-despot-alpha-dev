import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches
#import matplotlib as mpl

def plot_bar_graph_with_std_error(means, stds, colors):
    N = len(means[0])               # number of data entries
    ind = np.arange(N)              # the x locations for the groups
    width = 0.35                    # bar width
    
    fig, ax = plt.subplots()
    rects = []
    for i in (0,len(means)):
        rects1 = ax.bar(ind, means[i],                  # data
                width,                          # bar width
                color=colors[i],        # bar colour
                yerr=stds[i],                  # data for error bars
                error_kw={'ecolor':'Black',    # error-bars colour
                          'linewidth':2})       # error-bar width
        rects.append(rects1)
    
    axes = plt.gca()
    axes.set_ylim([0, 100])             # y-axis bounds

    #ax.set_ylabel('#Cases with history length > 30')
    ax.set_title('Test Cases')
    ax.set_xticks(ind + width)
    ax.set_xticklabels(('PT10', 'PT1', 'L3', 'L2PT10', 'L2PT1'))

    #ax.legend((rects1[0], rects2[0]), ('Training Cases', 'Test Cases'))

    for rect in rects:
        autolabel(rect)


    plt.show()   

def autolabel(rects1, ax):
    for rect in rect1:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                '%d' % int(height),
                ha='center',            # vertical alignment
                va='bottom'             # horizontal alignment
            )
def plot_multi_agent_rock_sample(multi_agent = True):
    plt.style.use('default')
    means = []
    stds = []
    #colors = ['red', 'green', 'blue', 'yellow']
    colors = ['C0', 'C1', 'C5', 'C4']
    patterns = ['\\\\', '.', '//', 'O']
    legend_patches = []
    labels = ['DESPOT-DIS', 'DESPOT-ALPHA', 'HyP-DESPOT-DIS', 'HyP-DESPOT-ALPHA']
    a_val = 0.6
    for i in range(0,4):
        legend_patch = mpatches.Patch( facecolor=colors[i],alpha=a_val,hatch=patterns[i],label=labels[i])
        legend_patches.append(legend_patch)
    xtick_labels = ()
    y_bound = 60
    if multi_agent:
        xtick_labels = ('MARS(15,10)', 'MARS(15,15)', 'MARS(20,20)')
        y_bound = 60
        DESPOT_means = [41.52, 31.87, 14.96]
        means.append(DESPOT_means)
        DESPOTALPHA_means = [24.02, 16.98, 13.44]
        means.append(DESPOTALPHA_means)
        HyPDESPOT_means = [43.07, 51.09, 55.36]
        means.append(HyPDESPOT_means)        
        HyPDESPOTALPHA_means = [42.43, 52.37, 49.55]
        means.append(HyPDESPOTALPHA_means)        

        DESPOT_stds = [0.25, 0.41, 0.35]
        stds.append(DESPOT_stds)
        DESPOTALPHA_stds = [0.31, 0.35, 0.33]
        stds.append(DESPOTALPHA_stds)
        HyPDESPOT_stds = [0.34, 0.43, 0.50]
        stds.append(HyPDESPOT_stds)        
        HyPDESPOTALPHA_stds = [0.32, 0.31, 0.73]
        stds.append(HyPDESPOTALPHA_stds)
    else:
        xtick_labels = ('RS(15,10)', 'RS(15,15)', 'RS(20,20)')
        y_bound = 45
        DESPOT_means = [32.35, 39.39, 37.40]
        means.append(DESPOT_means)
        DESPOTALPHA_means = [32.72, 40.75, 36.43]
        means.append(DESPOTALPHA_means)
        #HyPDESPOT_means = [43.07, 51.09, 55.36]
        #means.append(HyPDESPOT_means)        
        #HyPDESPOTALPHA_means = [42.43, 52.37, 49.55]
        #means.append(HyPDESPOTALPHA_means)        

        DESPOT_stds = [0.25, 0.41, 0.21]
        stds.append(DESPOT_stds)
        DESPOTALPHA_stds = [0.25, 0.41, 0.24]
        stds.append(DESPOTALPHA_stds)
        #HyPDESPOT_stds = [0.34, 0.43, 0.50]
        #stds.append(HyPDESPOT_stds)        
        #HyPDESPOTALPHA_stds = [0.32, 0.31, 0.73]
        #stds.append(HyPDESPOTALPHA_stds)

    #print len(means)
    #print means[3]
    N = len(means[0])               # number of data entries
    ind = np.arange(N)              # the x locations for the groups
    width = 0.15                    # bar width
    
    fig, ax = plt.subplots()
    rects = []
    for i in range(0,len(means)):
        print i
        print "Heelo"
        rects1 = ax.bar(ind+(i*width), means[i],                  # data
                width,                          # bar width
                color=colors[i],               # bar colour
                hatch=patterns[i],              #bar pattern        
                yerr=stds[i],                  # data for error bars
                label=labels[i],               #label
                error_kw={'ecolor':'Black',    # error-bars colour
                          'linewidth':2})       # error-bar width
        rects.append(rects1)
    
    axes = plt.gca()
    axes.set_ylim([0, y_bound])             # y-axis bounds

    #ax.set_ylabel('#Cases with history length > 30')
    #ax.set_title('Test Cases')
    mult = 1
    if multi_agent:
        mult = 0
    
    ax.set_xticks(ind + (mult*width))
    ax.set_xticklabels(xtick_labels)
    #if multi_agent:
        #ax.legend((rects[0][0], rects[1][0], ), labels[0:2], loc = 2)
    ax.legend(loc = 2)
    
    #for rect1 in rects:
    #   autolabel(rect1, ax)
        


    plt.show()   
    
    
plot_multi_agent_rock_sample(False)
