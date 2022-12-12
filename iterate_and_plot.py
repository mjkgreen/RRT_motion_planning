import dubins_path_problem
import time
import pandas as pd
import random
import numpy as np
import matplotlib.pyplot as plt
random.seed(42)

#default vars
delta_ori = np.pi / 6  # orientation bias
delta_R = 2  # search radius
seg_turn = 1 / 5  # turning sensitivity
sam_inter = 2  # sample intervals
sam_dens = 5  # sample density
w_d = 0.5  # weight of trajectory length 0.25 - 2.5
w_q = 10  # weight of trajectory curvature 1-20
w_e = 0.5  # weight of position deviation from bias path 0.2-2
w_theta = 0.5  # weight of orientation deviation from bias path 0.2-2
choose_map = 1  # 1 = Vertical Lines Maze,  2 =  Narrow Corridor Map
showplots = False

var = 'Orientation Deviation Weight' #set variable you want to change
for i in range(1, 3): #iterate vars (be sure to not just plug in "i" if you don't want to sample from 1-10
    print(i)
    for j in range(1, 3):# iterate maps # 1 = Vertical Lines Maze,  2 =  Narrow Corridor Map
        start_t = time.time()
        path_node_list, path_len_cost, path_curv_cost, path_time = dubins_path_problem.run(delta_ori, delta_R, seg_turn,
                                                                                           sam_inter, sam_dens, w_d,
                                                                                           w_q, w_e, w_theta,  j,
                                                                                           showplots)
        # vary sample intervals i , and map j
        end_t = time.time()
        t = end_t-start_t
        R_df = pd.DataFrame(data=[[path_len_cost, path_curv_cost, path_time, t, j, i/5]],
                            columns=['path length', 'curvature cost', 'pathing time', 'planning time', 'map', var],
                            index=[i])
        if i == 1 and j == 1:
            Results = R_df
        else:
            Results = pd.concat([Results, R_df])

Results.to_csv(var + '.csv')


Results.set_index(var, inplace=True)
fig = plt.figure()

ax1 = fig.add_subplot(221)
Results.groupby('map')['path length'].plot(ax=ax1)
plt.xlabel(var)
plt.ylabel('path length (m)')

ax2 = fig.add_subplot(222, sharex=ax1)
Results.groupby('map')['curvature cost'].plot(ax=ax2)
plt.legend(['Vertical Lines Maze', 'Narrow Corridor Map'])
plt.ylabel('curvature cost')

ax4 = fig.add_subplot(223, sharex=ax1)
Results.groupby('map')['pathing time'].plot(ax=ax4)
plt.ylabel('pathing time (s)')

ax5 = fig.add_subplot(224, sharex=ax1)
Results.groupby('map')['planning time'].plot(ax=ax5)
plt.ylabel('planning time (s)')

#fig.legend = (['Vertical Lines Maze', 'Narrow Corridor Map'])
plt.show()
