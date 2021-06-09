from datetime import time
from numpy.lib.function_base import insert
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.stats as stats
# from bioinfokit.analys import stat
import statsmodels
import numpy as np
import rospkg

r = rospkg.RosPack()
data_path = r.get_path('promp_ros')

df = pd.read_excel(data_path+'/data/time.xlsx', usecols=[0,1,2,3,4])
# print(df)
hr_kf = df[df['Human-Robot']=='With KF']
hr_no_kf = df[df['Human-Robot']=='Without KF']
pa_kf = df[df['Plug Alignment']=='With KF']
pa_nokf = df[df['Plug Alignment']=='Without KF']
insertion_p = df[df['Insertion']=='Passive']
insertion_a = df[df['Insertion']=='Active']

## pick blue and orange from default seaborn palette
blue = sns.color_palette()[0]
orange = sns.color_palette()[1]

def plot_time():
    box_w = 0.5
    figure = plt.figure(1, figsize=(8, 5))
    ## color
    col = 'k'
    sns.set_style("whitegrid")
    
    ## Human robot (kf)
    ax1 = figure.add_subplot(131)
    sns.boxplot(x='Human-Robot', y='Time (s)', data=df,  width=box_w, ax=ax1)
    d1 = hr_kf['Time (s)']
    d2 = hr_no_kf['Time (s)']
    ## Anova test
    fv, pv = stats.f_oneway(d1, d2)
    print(pv)
    y = 1.01*max(max(d1), max(d2))
    ## specify decimals in scientific notation
    text = f"p={pv:.2e}(**)"
    ## space between line and box
    h = 0.1
    plt.plot([0, 0, 1, 1],[y, y+h, y+h, y], lw=1.5, c=col)
    plt.text(0.5, y+h, text, ha="center", va="bottom", color=col)
    
    ## Plug alignment (kf)
    ax2 = figure.add_subplot(132)
    sns.boxplot(x='Plug Alignment', y='Time (s)', data=df,  width=box_w, ax=ax2)
    d1 = pa_kf['Time (s)']
    d2 = pa_nokf['Time (s)']
    fv, pv = stats.f_oneway(d1, d2)
    print(pv)
    y = 1.01*max(max(d1), max(d2))
    text = f"p={pv:.2e}(**)"
    ## space between line and box
    h = 0.1
    plt.plot([0, 0, 1, 1],[y, y+h, y+h, y], lw=1.5, c=col)
    plt.text(0.5, y+h, text, ha="center", va="bottom", color=col)
    
    ## Insertion (p/a)
    ax3 = figure.add_subplot(133)
    sns.boxplot(x='Insertion', y='Time (s)', order=[ 'Active', 'Passive'], data=df, width=box_w, ax=ax3)
    d1 = insertion_p['Time (s)']
    d2 = insertion_a['Time (s)']
    fv, pv = stats.f_oneway(d1, d2)
    print(pv)
    y = 1.01*max(max(d1), max(d2))
    text = f"p={pv:.2e}(**)"
    ## space between line and box
    h = 0.1
    plt.plot([0, 0, 1, 1],[y, y+h, y+h, y], lw=1.5, c=col)
    plt.text(0.5, y+h, text, ha="center", va="bottom", color=col)
    
    
    plt.tight_layout()
    plt.show()
    

def plot_force():
    box_w = 0.5
    df_force = pd.read_csv(data_path+"/data/force.csv")
    figure = plt.figure(figsize=(5,4))
    sns.set_style("whitegrid")
    ax = figure.add_subplot(111)
    d1 = df_force[(df_force['Sex']=='Male') & (df_force['Direction']=='x')]['Directional Force (N)']
    d2 = df_force[(df_force['Sex']=='Female') & (df_force['Direction']=='x')]['Directional Force (N)']
    _, p1 = stats.f_oneway(d1, d2)
    ym1 = 1.03*max(max(d1), max(d2))
    d1 = df_force[(df_force['Sex']=='Male') & (df_force['Direction']=='y')]['Directional Force (N)']
    d2 = df_force[(df_force['Sex']=='Female') & (df_force['Direction']=='y')]['Directional Force (N)']
    _, p2 = stats.f_oneway(d1, d2)
    ym2 = 1.03*max(max(d1), max(d2))
    d1 = df_force[(df_force['Sex']=='Male') & (df_force['Direction']=='z')]['Directional Force (N)']
    d2 = df_force[(df_force['Sex']=='Female') & (df_force['Direction']=='z')]['Directional Force (N)']
    _, p3 = stats.f_oneway(d1, d2)
    ym3 = 1.03*max(max(d1), max(d2))
    sns.boxplot(x='Direction', y='Directional Force (N)', hue='Sex', data=df_force, width=box_w, ax=ax)

    p = [p1, p2, p3]
    y = [ym1, ym2, ym3]
    x_center = [0-box_w/4, 0+box_w/4, 1-box_w/4, 1+box_w/4, 2-box_w/4, 2+box_w/4]
    col = 'k'
    for i in range(3):
        if p[i] < 0.05:
            text = f"p={p[i]:.2e}(*)"
        else:
            text = f"p={p[i]:.2e}"
        x1 = x_center[2*i]
        x2 = x_center[2*i+1]
        h = 0.1
        plt.plot([x1,x1,x2,x2],[y[i], y[i]+h, y[i]+h, y[i]], lw=1.5, c=col)
        plt.text(i, y[i]+h, text, ha="center", va="bottom", color=col)


    plt.tight_layout()
    plt.show()

def plot_comparative_time():
    # f, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios':[4,2]}, figsize=(10,5))
    font = {'family' : 'normal',
            'weight': 'bold',
            'size'   : 8}
    matplotlib.rc('font', **font)
    df = pd.read_csv(data_path+"/data/comparison_target.csv")
    fig, ax = plt.subplots(figsize=(6, 3))
    # plt horizontal grid only
    clrs = ['gray', blue, blue, blue, orange]
    ax.yaxis.grid(True)
    # plot grid behind the graph
    ax.set_axisbelow(True)
    sns.barplot(x='Method', y='Time (s)', palette=clrs, 
                order=['direct tracking', 'baseline (p=0.5)','baseline (p=0.8)','baseline (p=1)', 'proposed'], 
                data=df, ax=ax, edgecolor='k')
    ax.bar_label(ax.containers[-1], padding=3)
    ax.xaxis.label.set_size(14)
    ax.yaxis.label.set_size(14)

    plt.tight_layout()
    plt.show()


def plot_precision():
    box_w = 0.5
    font = {'family' : 'normal',
            'weight': 'bold',
            'size'   : 8}
    matplotlib.rc('font', **font)
    # change font size 
    # sns.set_theme (font_scale=2.0, style="whitegrid")
    df = pd.read_csv(data_path+"/data/comparison_target.csv")
    fig, ax = plt.subplots(figsize=(5,3))
    ax.yaxis.grid(True)
    # plot grid behind the graph
    ax.set_axisbelow(True)
    # sns.set_style("whitegrid")
    clrs = [blue, blue, blue, orange]
    sns.barplot(x='Method', y='Position Error (cm)', palette=clrs, 
                order=['baseline (p=0.5)','baseline (p=0.8)','baseline (p=1)', 'proposed'], 
                data=df,  edgecolor='k', ax=ax)
    ax.bar_label(ax.containers[-1], padding=3)
    ax.xaxis.label.set_size(14)
    ax.yaxis.label.set_size(14)

    plt.tight_layout()
    plt.show()

def plot_pattern():
    font = {'family' : 'normal',
        'weight': 'bold',
        'size'   : 18}

    matplotlib.rc('font', **font)
    fig = plt.figure(figsize=(10,8))
    gs = fig.add_gridspec(2,3)
    ax1 = fig.add_subplot(gs[0,:])
    ax2 = fig.add_subplot(gs[1,0])
    ax3 = fig.add_subplot(gs[1,1])
    ax4 = fig.add_subplot(gs[1,2])

    df = pd.read_csv(data_path+"/data/comparison_target.csv")
    df_selected = df[(df['Method']=='proposed')|(df['Method']=='baseline')]
    clrs = [blue, orange]
    sns.barplot(x='Pattern', y='Position Error (cm)', hue='Method', palette=clrs, order=['straight', 'curved', 'accelerated'], 
                data=df_selected, ax=ax1, edgecolor='k')
    # sns.set_theme(style="whitegrid", font_scale=1.5)
    # plt horizontal grid only
    ax1.yaxis.grid(True)
    # plot grid behind the graph
    ax1.set_axisbelow(True)
    straight = np.loadtxt(data_path+"/data/straight.csv", delimiter=",")
    ax1.set_xlabel("")
    # ax1.tick_params(labelsize='large')
    ax2.plot(straight[0:len(straight):10,0], straight[0:len(straight):10,1], marker=6)
    ax2.set_xticks([-0.1, 0, 0.1])
    # ax2.tick_params(labelsize='large')
    ax2.set_xlabel("straight")
    curved = np.loadtxt(data_path+"/data/curved.csv", delimiter=",")
    ax3.plot(curved[0:len(curved):10,0], curved[0:len(curved):10,1], marker=6)
    ax3.set_xticks([-0.1, 0, 0.1])
    ax3.set_xlabel("curved")
    # ax3.tick_params(labelsize='large')
    accelerated = np.loadtxt(data_path+"/data/accelerated.csv", delimiter=",")
    ax4.plot(accelerated[0:len(accelerated):10,0], accelerated[0:len(accelerated):10,1], marker=6)
    ax4.set_xticks([-0.1, 0, 0.1])
    ax4.set_xlabel("accelerated")
    # ax4.tick_params(labelsize='large')

    # ax.bar_label(ax.containers[-1], padding=3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # plot_time()
    # plot_force()
    plot_comparative_time()
    # plot_pattern()
    # plot_precision()
    # plot_pattern()


