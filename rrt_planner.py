"""
Assignment #2 Template file
"""
import random
import math
import numpy as np
import setup_func as s_f
import posq as posq

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_planner.py. Your implementation
   can be tested by running RRT_DUBINS_PROBLEM.PY (check the main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specificed below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)
3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def rrt_planner(rrt_anyangle,delta_ori,delta_R,seg_turn,sam_inter,sam_dens,bias_path, w_d, w_q, w_e, w_theta, display_map=False):
    """
        Execute RRT planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    
    direction = 0
    resolution = 0.1
    base = 0.4
    init_t = 0.1
    max_speed = 1.0
    # w_d = 0.5 # weight of trajectory length
    # w_q = 10 # weight of trajectory curvature
    # w_e = 0.5 # weight of position deviation from bias path
    # w_theta = 0.5 # weight of orientation deviation from bias path
    delta_W = 2
    # delta_ori = np.pi / 6  # orientation
    # delta_R = 2  # search radius
    # seg_turn = 1 / 5  # turning sensitivity
    # sam_inter = 2  # sample intervals
    # sam_dens = 3  # sample intervals

    # LOOP for max iterations
    i = 0
    while i < rrt_anyangle.max_iter:

        # print(rrt_anyangle.node_list[-1].x)
        # print(rrt_anyangle.node_list[-1].y)
        # print(rrt_anyangle.node_list[-1].yaw)
        n_node = len(rrt_anyangle.node_list)
        i += 1
        # if n_node>100:
        #     x_new = rrt_anyangle.goal.x 
        #     y_new = rrt_anyangle.goal.y
        #     ori_new = rrt_anyangle.goal.yaw
        #     d_P_new = rrt_anyangle.goal.d_P
        #     new_pseg_ind = rrt_anyangle.goal.pseg_ind
        # else:
        x_new, y_new, ori_new, d_P_new, new_pseg_ind = s_f.search_new_node(n_node, sam_inter, sam_dens,rrt_anyangle.start.yaw, rrt_anyangle.goal.yaw, rrt_anyangle.bias_path, rrt_anyangle.bias_path_seg, delta_W, delta_ori, seg_turn)
   
        
        # Find an existing node nearest to the random vehicle state
        cost_list = []
        len_cost_list = []
        curv_cost_list = []
        traj_list = []
        inct_list = []
        ind_list = []
        
        for ind, tree_node in enumerate(rrt_anyangle.node_list):
            # ind = rrt_anyangle.node_list.index(tree_node)
            dx_node = x_new - tree_node.x
            dy_node = y_new - tree_node.y
            de_node = math.hypot(dx_node, dy_node)
            if de_node < 0.001:
                continue
            elif de_node < delta_R:
                source = np.array([tree_node.x, tree_node.y, tree_node.yaw])
                target = np.array([x_new, y_new, ori_new])
                traj, _, _, inct, traj_cost, len_cost, curv_cost = posq.integrate(source, target, direction,
                                                                   resolution, base,
                                                                   init_t, max_speed, w_d, w_q, nS=0)
                if traj[-1,2]-ori_new > 0.5:
                    continue
                Geo_cost = w_e*(d_P_new+tree_node.d_P)+w_theta*((1 - np.cos(traj[-1,2]-rrt_anyangle.bias_path_seg[0,new_pseg_ind])))+w_theta*((1 - np.cos(tree_node.yaw-rrt_anyangle.bias_path_seg[0,tree_node.pseg_ind])))
                cost = traj_cost + Geo_cost + tree_node.cost
                #cost = traj_cost + Geo_cost
                cost_list.append(cost)
                len_cost_list.append(len_cost)
                curv_cost_list.append(curv_cost)
                traj_list.append(traj)
                inct_list.append(inct)
                ind_list.append(ind)
            else:
                continue
            
        if not cost_list:
            continue
            
        index_n = cost_list.index(min(cost_list))
        nn_node_id = ind_list[index_n]
        new_node = rrt_anyangle.Node(x_new,y_new,ori_new,d_P_new,new_pseg_ind)
        
        traj_new = traj_list[index_n]
        t_togo_new = inct_list[index_n]
        cost_new = cost_list[index_n]
        len_cost_new = len_cost_list[index_n]
        curv_cost_new = curv_cost_list[index_n]
        
        px = list(traj_new[:,0])
        py = list(traj_new[:,1])
        pyaw = list(traj_new[:,2])
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]
        
        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost = rrt_anyangle.node_list[nn_node_id].cost + cost_new
        new_node.len_cost = rrt_anyangle.node_list[nn_node_id].len_cost + len_cost_new
        new_node.curv_cost = rrt_anyangle.node_list[nn_node_id].curv_cost + curv_cost_new
        new_node.t_togo = rrt_anyangle.node_list[nn_node_id].t_togo + t_togo_new
        new_node.parent = rrt_anyangle.node_list[nn_node_id]

        # Check if the path between nearest node and random state has obstacle collision
        # Add the node to nodes_list if it is valid
        if rrt_anyangle.check_collision(new_node):
            rrt_anyangle.node_list.append(new_node) # Storing all valid nodes

        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_anyangle.draw_graph(bias_path)

        # Check if new_node is close to goal
        if rrt_anyangle.goal.is_state_identical(rrt_anyangle.node_list[-1]):
            print("Iters:", i, ", number of nodes:", len(rrt_anyangle.node_list))
            break

    if i == rrt_anyangle.max_iter:
        print('reached max iterations')

    # Return path, which is a list of nodes leading to the goal...
    cur_node = rrt_anyangle.node_list[-1]
    rrt_path_list = [cur_node]
    while not(rrt_anyangle.start.is_state_identical(cur_node)):
        cur_node = cur_node.parent
        rrt_path_list.append(cur_node)
        
    rrt_path_list.reverse()
    path_len_cost = rrt_anyangle.node_list[-1].len_cost
    path_curv_cost = rrt_anyangle.node_list[-1].curv_cost
    path_time = rrt_anyangle.node_list[-1].t_togo
    return rrt_path_list, path_len_cost, path_curv_cost, path_time
