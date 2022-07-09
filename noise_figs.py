#! /usr/env/python

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sb
sb.set_theme(style='ticks')

tag = 'aggression'
colors = {'omni':'#000000', 'diromni':'#004488', 'dir':'#BB5566'}

data_dir = '/home/alli/projects/BVCollectives/data/'

files = ['noise_'+tag+'_omni_lim100_data.csv',
         'noise_'+tag+'_lantern_sim_data.csv',
         'noise_'+tag+'_directional_data.csv']
noise = [0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0.0]
df_omni_omni = pd.read_csv(data_dir+files[0])
df_dir_omni  = pd.read_csv(data_dir+files[1])
df_dir_dir   = pd.read_csv(data_dir+files[2])

print(df_omni_omni)
print(df_dir_omni)
print(df_dir_dir)
dist_omni_omni = df_omni_omni['mean dist']
dist_std_omni_omni = df_omni_omni['std dist']
ke_omni_omni = df_omni_omni['mean ke']
ke_std_omni_omni = df_omni_omni['st ke']

dist_dir_omni = df_dir_omni['mean dist']
dist_std_dir_omni = df_dir_omni['std dist']
ke_dir_omni = df_dir_omni['mean ke']
ke_std_dir_omni = df_dir_omni['st ke']

dist_dir = df_dir_dir['mean dist']
dist_std_dir = df_dir_dir['std dist']
ke_dir = df_dir_dir['mean ke']
ke_std_dir = df_dir_dir['st ke']
print(dist_dir)
print(dist_std_dir)

fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
#plt.figure(num=1, figsize=(2,8), dpi=300)

# plot means
ax1.set_xlabel("Noise scale ($\gamma$)")
ax1.set_ylabel("Mean Distance (px)")
ax1.set_ylim([0,300])
ax1.set_xlim([-0.01,0.1])
ax1.errorbar(noise, dist_dir, yerr=dist_std_dir, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['dir'],
ecolor=colors['dir'],label="directional stim and sense")

ax1.errorbar(noise, dist_dir_omni, yerr=dist_std_dir_omni, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['diromni'],
ecolor=colors['diromni'], label="directional sense, omni stim")

ax1.errorbar(noise, dist_omni_omni, yerr=dist_std_omni_omni, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['omni'],
ecolor=colors['omni'], label="omni sense and stim")

ax1.set_aspect(1/8000)


# plot std
ax2.set_ylim([-1,16.5])
ax2.set_xlim([-0.01,0.1])
ax2.set_xlabel("Noise scale ($\gamma$)")
ax2.set_ylabel("Mean Kinetic Energy ($px^2$)")
ax2.errorbar(noise, ke_dir, yerr=ke_std_dir, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['dir'],
ecolor=colors['dir'],label="directional stimulus and sensing")

ax2.errorbar(noise, ke_dir_omni, yerr=ke_std_dir_omni, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['diromni'],
ecolor=colors['diromni'], label="directional sensing, omni stimulus")

ax2.errorbar(noise, ke_omni_omni, yerr=ke_std_omni_omni, fmt='o-', capthick=1,
capsize=5, elinewidth=1, markeredgewidth=2, color=colors['omni'],
ecolor=colors['omni'], label="omni sensing and stimulus")

ax2.set_aspect(1/450)
#ax2.legend(bbox_to_anchor=(1.1, 1.1))
ax2.legend(loc='upper right', prop={'size':6})


plt.savefig(tag+"_noise_plot.png", bbox_inches='tight')

plt.show()
