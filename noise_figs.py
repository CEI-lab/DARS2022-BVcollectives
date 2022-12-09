#! /usr/env/python

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sb
sb.set_theme(style='ticks')

tags = ['love', 'aggression', 'fear','curiosity']
types = ['dir_dir']
colors = {'love':'#FFF140', 'aggression':'#FF6565', 'fear':'#3FDCFF', 'curiosity':'#B831F3'}
markers = {'omni_omni':'o', 'dir_omni': 'o', 'dir_dir':'o'}


labels = {'omni_omni': "omni stim and sense",
          'dir_omni': "dir stim, omni sense",
          'dir_dir': "directional stim and sense"}

data_dir = './data/'
fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
ax1.set_ylim([0,85])
ax1.set_xlim([-0.5,9.5])
ax2.set_ylim([-1,16.5])
ax2.set_xlim([-0.5,9.5])

ax1.set_xlabel("Noise Std ($\sigma$), percent of max")
ax1.set_ylabel("Mean NND (px)")
ax2.set_xlabel("Noise Std ($\sigma$), percent of max")
ax2.set_ylabel("Mean Kinetic Energy ($px^2$)")

for tag in tags: # one of four behaviors love / etc
    for type in types: # sensor/stimulus type

        fname = 'noise_' + tag + '_' + type + '_20.csv'

        noise = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
        df = pd.read_csv(data_dir+fname)

        dist = df['mean nnd']
        dist_std = df['std nnd']
        ke = df['mean ke']
        ke_std = df['st ke']

        # plot means
        ax1.errorbar(noise, dist, yerr=dist_std, fmt='-', capthick=1,
        capsize=5, elinewidth=1, markeredgewidth=2, color=colors[tag],
        marker = markers[type], ecolor=colors[tag], label=tag)

        # plot std
        ax2.errorbar(noise, ke, yerr=ke_std, fmt='-', capthick=1,
        capsize=5, elinewidth=1, markeredgewidth=2, color=colors[tag],
        marker = markers[type], ecolor=colors[tag], label=tag)


ax1.set_aspect(1/23)
ax2.set_aspect(1/5)
#ax2.legend(bbox_to_anchor=(1.1, 1.1))
ax1.legend(loc='lower left', prop={'size':10})


plt.savefig(type+"_noise_plot.png", bbox_inches='tight')

plt.show()
