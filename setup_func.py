# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 16:26:43 2022

@author: xzhan
"""

# import math
# import matplotlib.pyplot as plt
import numpy as np

def path_seg_config(bias_path):
    bias_path_ori = []
    bias_path_len = []
    bias_path_prelen = [0]
    n_seg = bias_path.shape[1]-1   
    for j in range (n_seg):
        dx = bias_path[0,j+1] - bias_path[0,j]
        dy = bias_path[1,j+1] - bias_path[1,j]
        ori_seg = np.arctan2(dy, dx)
        len_seg = np.sqrt(dx**2 + dy**2)
        bias_path_ori.append(ori_seg)
        bias_path_len.append(len_seg)
        bias_path_prelen.append(bias_path_prelen[-1]+len_seg)
    del bias_path_prelen[-1]
    bias_path_seg = np.array([np.array(bias_path_ori), np.array(bias_path_len), np.array(bias_path_prelen)])
    return bias_path_seg

def search_new_node(n_node, sam_inter, sam_dens, start_yaw, goal_yaw, bias_path, bias_path_seg, delta_W, delta_ori, seg_turn):
    total_len = np.sum(bias_path_seg[1])
    l_start = n_node//sam_dens
    if l_start > total_len-sam_inter:
        l_start = total_len-sam_inter
        
    rand_len = sam_inter*np.random.rand() + l_start
    for j in range(bias_path_seg.shape[1]):
        if rand_len<= bias_path_seg[2,j] + bias_path_seg[1,j]:
            sample_seg_ind = j
            l = rand_len - bias_path_seg[2,j]
            break
    seg_len = bias_path_seg[1,sample_seg_ind]
    if bias_path_seg.shape[1] == 1:
        if l>(1-seg_turn)*seg_len:
            w = (seg_len-l)/(seg_len*seg_turn)
            ori_mean = bias_path_seg[0,sample_seg_ind]*w + goal_yaw*(1-w) 
        elif l<seg_turn*seg_len:
            w = l/(seg_len*seg_turn)
            ori_mean = bias_path_seg[0,sample_seg_ind]*w + start_yaw*(1-w)      
        else:
            ori_mean = bias_path_seg[0,sample_seg_ind]
    else:
        if sample_seg_ind == 0:
            if l<=seg_turn*seg_len:
                w = l/(seg_len*seg_turn)
                ori_mean = bias_path_seg[0,sample_seg_ind]*w + start_yaw*(1-w)      
            elif l<=(1-seg_turn)*seg_len:
                ori_mean = bias_path_seg[0,sample_seg_ind]
            else:
                w = 0.5 + (seg_len-l)/(seg_len*seg_turn)*0.5
                ori_mean = bias_path_seg[0,sample_seg_ind]*w + bias_path_seg[0,sample_seg_ind+1]*(1-w)             
        elif sample_seg_ind == bias_path_seg.shape[1]-1:
            if l>(1-seg_turn)*seg_len:
                w = (seg_len-l)/(seg_len*seg_turn)
                ori_mean = bias_path_seg[0,sample_seg_ind]*w + goal_yaw*(1-w) 
            elif l<seg_turn*seg_len:
                w = 0.5 + l/(seg_len*seg_turn)*0.5
                ori_mean = bias_path_seg[0,sample_seg_ind]*w + bias_path_seg[0,sample_seg_ind-1]*(1-w)    
            else:
                ori_mean = bias_path_seg[0,sample_seg_ind]
        else:
            if l<seg_turn*seg_len:
                w = 0.5 + l/(seg_len*seg_turn)*0.5
                ori_mean = bias_path_seg[0,sample_seg_ind]*w + bias_path_seg[0,sample_seg_ind-1]*(1-w)
            elif l>(1-seg_turn)*seg_len:
                w = 0.5 + (seg_len-l)/(seg_len*seg_turn)*0.5
                ori_mean = bias_path_seg[0,sample_seg_ind]*w + bias_path_seg[0,sample_seg_ind+1]*(1-w)
            else:
                ori_mean = bias_path_seg[0,sample_seg_ind]
        
    seg_ori = bias_path_seg[0,sample_seg_ind]
    x_set = bias_path[0,sample_seg_ind] + l * np.cos(seg_ori)
    y_set = bias_path[1,sample_seg_ind] + l * np.sin(seg_ori)
    rand_W = delta_W * np.random.rand() - delta_W/2
    d_P_new = abs(rand_W)
    x_new = x_set + rand_W * np.sin(ori_mean)
    y_new = y_set - rand_W * np.cos(ori_mean)
    ori_new = 2*delta_ori*np.random.rand() + ori_mean - delta_ori
    
    
    return x_new, y_new, ori_new, d_P_new, sample_seg_ind
        