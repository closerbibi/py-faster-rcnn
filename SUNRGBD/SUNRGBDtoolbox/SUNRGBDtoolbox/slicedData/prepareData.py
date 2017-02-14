# -*- coding: utf-8 -*-
"""
Created on Thu Jan  5 16:36:15 2017

@author: closerbibi
"""

import scipy.io as sio
import os
import numpy as np
import h5py
import pdb
import time
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math

def compute_mean():


def plot_3d(layer, xmin, xmax, ymin, ymax, zmin, zmax, layers):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    color = ['r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r']
    for num in xrange(len(layer)):
        xs = layer[num][0][range(0,len(layer[num][0]),10)]
        ys = layer[num][1][range(0,len(layer[num][1]),10)]
        zs = layer[num][2][range(0,len(layer[num][2]),10)]
        xb = np.concatenate((xmin[0],xmax[0]))
        yb = np.concatenate((zmin[0],xmax[0]))
        zb = np.concatenate((zmin[0],zmax[0]))
        ax.scatter(xs, ys, zs, c=color[num], marker='o')
        #ax.scatter(xb, yb, zb, c='y', marker='^')
        ax.scatter(xmin[0], ymin[0], zmin[0], c='y', marker='^')
        ax.scatter(xmax[0], ymax[0], zmax[0], c='k', marker='^')
        #plt.setp(ax.scatter(0, 0, 0), color='yellow')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

def slicedto2D(pc, normals, imagenum, xmin, xmax, ymin, ymax, zmin, zmax, pooling):
    # find upright vector
    upright = np.array([1e-06,1e-06, 1])
    points_height = pc[2,:]
    ceiling = np.max(points_height)
    floor = np.min(points_height)

    # seperate to multiple layers
    layers = 25
    l_step = (ceiling - floor)/layers
    layer = {}
    # find index in the range
    for layer_num in xrange(layers):
        cur_ceiling = floor + l_step*(layer_num+1)
        cur_floor = floor + l_step*layer_num

        # trim layer without target class (chair only first)
        if (cur_ceiling < zmin[0]).all() or (cur_floor > zmax[0]).all():
            continue
        idx = np.where((pc[2,:] > cur_floor) * (pc[2,:] < cur_ceiling))
        layer[layer_num] = pc[:,idx[0]]

    # show 3d data and bbox corner
    #plot_3d(layer, xmin, xmax, ymin, ymax, zmin, zmax, layers)
    
    # making grid
    


    pcx_shift = pc[0]-np.min(pc[0])
    pcy_shift = pc[1]-np.min(pc[1])
    largex = np.floor(pcx_shift*100)
    largey = np.floor(pcy_shift*100)
    largepc = np.vstack((largex,largey,pc[2]))
    idxx = range(int(np.min(largepc[0])),int(np.max(largepc[0])+1),1)
    idxy = range(int(np.min(largepc[1])),int(np.max(largepc[1])+1),1)
    data_grid = np.meshgrid(idxx,idxy)
    grid = np.zeros((len(layer),2,len(idxx),len(idxy)))
    for ilayer in xrange(len(layer)):
        layerpcx_shift = layer[ilayer][0]-np.min(pc[0])
        layerpcy_shift = layer[ilayer][1]-np.min(pc[1])
        layerlargex = np.floor(layerpcx_shift*100)
        layerlargey = np.floor(layerpcy_shift*100)
        large_layerpc = np.vstack((layerlargex,layerlargey,layer[ilayer][2]))
        start = time.time()
        for ix in idxx:
            for iy in idxy:
                # compute density
                location = (large_layerpc[0]==ix)*(large_layerpc[1]==iy)
                grid[ilayer][0][ix][iy] = np.sum((large_layerpc[0]==ix)*(large_layerpc[1]==iy))
                # find max
                if grid[ilayer][0][ix][iy] == 0:   # if empty, giving lowest value of current layer
                    grid[ilayer][1][ix][iy] = np.min(large_layerpc[2])
                else:
                    grid[ilayer][1][ix][iy] = np.max(large_layerpc[2][location])
        print 'time:'+ str(time.time()-start)
    pdb.set_trace()


    
    return grid


data = h5py.File('/media/closerbibi/internal3/3D/understanding/rankpooling/rgbd_data/nyu_v2_labeled/nyu_depth_v2_labeled.mat')

#calibration parameters from author
K = np.array([[5.7616540758591043e+02,0,3.2442516903961865e+02],[0,5.7375619782082447e+02,2.3584766381177013e+02],[0,0,1]]) 

count=1;

for imagenum in xrange(data['depths'].shape[0]):#size(depths,3):
    start_time = time.time()
    # bed=157, chair=5, table=19, sofa=83, toilet=124
    try:
        box_pc = sio.loadmat('../alignData/image%04d/annotation_pc.mat' % (imagenum)); # pc generate by ../seeAlignment_pc_3dBox.m
    except:
        continue
    #points3d = points3d'
    pc = box_pc['points3d']; clss=box_pc['clss']
    ymin = box_pc['ymin']; ymax=box_pc['ymax']; xmin=box_pc['xmin']; xmax=box_pc['xmax']; zmin=box_pc['zmin']; zmax=box_pc['zmax']
    normals = sio.loadmat('./normalAndpc/normalAndpc%06d.mat'%(imagenum))['normals']
    pooling = 0
    #slicedto2D
    
    [grid, mean] = slicedto2D(pc, normals, imagenum, xmin, xmax, ymin, ymax, zmin, zmax, pooling);
    '''
    %% computing mean
    mean1 = mean1 + mean_all(1);
    mean2 = mean2 + mean_all(2);
    mean3 = mean3 + mean_all(3);
    mean4 = mean4 + mean_all(4);
    %%  
    count=count+1;
»···toc;
    disp(imagenum)
    '''
    print 'image: %d' % (imagenum), 'time: %.2f s' % (time.time()-start_time)
