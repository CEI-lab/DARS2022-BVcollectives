import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pylab



# import the data
sim_data = pd.read_csv("data/test.csv",dtype=object)

x_list = []
y_list = []
theta_list = []
light_list = []
for i in range(sim_data['x'].shape[0]):
    x_list.append(list(map(int,sim_data['x'][i][1:-1].replace(" \n", "").split())))
    y_list.append(list(map(int,sim_data['y'][i][1:-1].replace(" \n", "").split())))
    theta_list.append(list(map(float,sim_data['theta'][i][1:-1].replace(" \n", "").split())))
    light_list.append(list(map(int,sim_data['light'][i][1:-1].replace(" \n", "").split())))



NUM_COLORS = 20

colors = np.zeros((NUM_COLORS,4))
cm = pylab.get_cmap('gist_rainbow')
for i in range(NUM_COLORS):
    colors[i] = cm(1.*i/NUM_COLORS)  # color will now be an RGBA tuple

fig, ax = plt.subplots()

ax.set_aspect('equal')

for i in range(len(x_list)-100,len(x_list)-1):
    ax.scatter(x_list[i],y_list[i],alpha=((i+100-len(x_list))/400),c=colors,s=2)

# plot the most recent one big and opaque
ax.scatter(x_list[-1],y_list[-1],alpha=1,c=colors)

plt.tick_params(axis='both',which='both',labelbottom=False,labelleft=False,bottom=False,left=False)

plt.ylim(0,500)
plt.xlim(0,500)
# plt.ylim(260,360)
# plt.xlim(385,485)
# plt.title("Curiosity at Convergence")
plt.show()
