#! /usr/env/python

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sb
sb.set_theme(style='ticks')

tag = 'curiosity'
colors = {'omni':'#000000', 'diromni':'#004488', 'dir':'#BB5566'}

data_dir = './data/'

files = ['noise_'+tag+'_omni_omni_20.csv',
         'noise_'+tag+'_dir_omni_20.csv',
         'noise_'+tag+'_dir_dir_20.csv']
noise = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
df_oo = pd.read_csv(data_dir+files[0])
df_do  = pd.read_csv(data_dir+files[1])
df_dd  = pd.read_csv(data_dir+files[2])

dist_oo = df_oo['mean nnd']
dist_std_oo = df_oo['std nnd']
ke_oo = df_oo['mean ke']
ke_std_oo = df_oo['st ke']

dist_do = df_do['mean nnd']
dist_std_do = df_do['std nnd']
ke_do = df_do['mean ke']
ke_std_do = df_do['st ke']

dist_dd = df_dd['mean nnd']
dist_std_dd = df_dd['std nnd']
ke_dd = df_dd['mean ke']
ke_std_dd = df_dd['st ke']

fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
#plt.figure(num=1, figsize=(2,8), dpi=300)

# plot means
ax1.set_xlabel("Noise Std ($\sigma$), percent of max")
ax1.set_ylabel("Mean NND (px)")
ax1.set_ylim([0,85])
ax1.set_xlim([-0.5,9.5])
ax1.errorbar(noise, dist_oo, yerr=dist_std_oo, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['omni'],
ecolor=colors['omni'],label="omni stim and sense")

ax1.errorbar(noise, dist_do, yerr=dist_std_do, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['diromni'],
ecolor=colors['diromni'], label="dir stim, omni sense")

ax1.errorbar(noise, dist_dd, yerr=dist_std_dd, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['dir'],
ecolor=colors['dir'], label="directional stim and sense")

ax1.set_aspect(1/23)


# plot std
ax2.set_ylim([-1,16.5])
ax2.set_xlim([-0.5,9.5])
ax2.set_xlabel("Noise Std ($\sigma$), percent of max")
ax2.set_ylabel("Mean Kinetic Energy ($px^2$)")
ax2.errorbar(noise, ke_oo, yerr=ke_std_oo, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['omni'],
ecolor=colors['omni'],label="omni stim and sense")

ax2.errorbar(noise, ke_do, yerr=ke_std_do, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['diromni'],
ecolor=colors['diromni'], label="dir stim, omni sense")

ax2.errorbar(noise, ke_dd, yerr=ke_std_dd, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['dir'],
ecolor=colors['dir'], label="directional stim and sense")

ax2.set_aspect(1/5)
#ax2.legend(bbox_to_anchor=(1.1, 1.1))
ax2.legend(loc='center left', prop={'size':10})


plt.savefig(tag+"_noise_plot.png", bbox_inches='tight')

plt.show()
