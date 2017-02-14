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
import pylab
import matplotlib.patches as patches

def plot_2d_pc_bbox(grid, xmin, xmax, ymin, ymax):
    fig = pylab.figure()
    ax = fig.add_subplot(111, aspect='equal')
    xlen=  xmax - xmin
    ylen=  ymax - ymin
    '''
    ax3.add_patch(
        patches.Rectangle(
            (xmin,ymin), xlen, ylen, fill=False, # remove background~
            edgecolor='green'
        )
    )
    '''
    for i in xrange(len(xmin)):
        ax.add_patch(patches.Rectangle( (xmin[i],ymin[i]), xlen[i], ylen[i], fill=False, edgecolor='green' ))
        #ax.add_patch(patches.Rectangle( (ymin[i],xmin[i]), ylen[i], xlen[i], fill=False, edgecolor='green' ))
    plt.imshow(grid[4,0,:])
    plt.draw()
    plt.title('layer:'+'' )
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    plt.show()
    plt.show(block=False)
    pdb.set_trace()


def compute_mean():
    print "Hello, I am empty"

def plot_3d(layer, xmin, xmax, ymin, ymax, zmin, zmax, layers):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    color = ['r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r']
    for num in xrange(len(layer)):
        xs = layer[num][0][range(0,len(layer[num][0]),10)]
        ys = layer[num][1][range(0,len(layer[num][1]),10)]
        zs = layer[num][2][range(0,len(layer[num][2]),10)]
        xb = np.concatenate((xmin,xmax))
        yb = np.concatenate((zmin,xmax))
        zb = np.concatenate((zmin,zmax))
        ax.scatter(xs, ys, zs, c=color[num], marker='o')
        #ax.scatter(xb, yb, zb, c='y', marker='^')
        ax.scatter(xmin, ymin, zmin, c='y', marker='^')
        ax.scatter(xmax, ymax, zmax, c='k', marker='^')
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
        if (cur_ceiling < zmin).all() or (cur_floor > zmax).all():
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
    grid = np.zeros((len(layer),2,len(idxy),len(idxx)))
    for ilayer in xrange(len(layer)): #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        layerpcx_shift = layer[ilayer][0]-np.min(pc[0])
        layerpcy_shift = layer[ilayer][1]-np.min(pc[1])
        layerlargex = np.floor(layerpcx_shift*100)
        layerlargey = np.floor(layerpcy_shift*100)
        large_layerpc = np.vstack((layerlargex,layerlargey,layer[ilayer][2]))
        start = time.time()
        for ix in idxx:
            for iy in idxy:
                rviy = max(idxy) - iy
                # compute density: channel 0
                location = (large_layerpc[0]==ix)*(large_layerpc[1]==iy)
                grid[ilayer][0][rviy][ix] = np.sum((large_layerpc[0]==ix)*(large_layerpc[1]==iy))
                # find max: channel 1
                if grid[ilayer][0][rviy][ix] == 0:   # if empty, giving lowest value of current layer
                    grid[ilayer][1][rviy][ix] = np.min(large_layerpc[2])
                else:
                    grid[ilayer][1][rviy][ix] = np.max(large_layerpc[2][location])
        print 'time'+ str(time.time()-start)
    pdb.set_trace()
    xmin = xmin - np.min(pc[0]); xmax = xmax - np.min(pc[0]); ymin = ymin - np.min(pc[1]); ymax = ymax - np.min(pc[1]);
    xmin = np.floor(xmin*100); xmax = np.floor(xmax*100); ymin = np.floor(ymin*100); ymax = np.floor(ymax*100); #zmin = np.floor(zmin*100); zmax = np.floor(zmax*100);
    ymin = max(idxy) - ymin; ymax = max(idxy) - ymax
    plot_2d_pc_bbox(grid, xmin, xmax, ymin, ymax)
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
    # change bbox to y,x,z
    ymin = box_pc['ymin'][0]; ymax=box_pc['ymax'][0]; xmin=box_pc['xmin'][0]; xmax=box_pc['xmax'][0]; zmin=box_pc['zmin'][0]; zmax=box_pc['zmax'][0];
    #xmin = box_pc['ymin'][0]; xmax=box_pc['ymax'][0]; ymin=box_pc['xmin'][0]; ymax=box_pc['xmax'][0]; zmin=box_pc['zmin'][0]; zmax=box_pc['zmax'][0];
    pc = np.swapaxes(pc, 0, 1)
    # change pc to y,x,z
    #pc = pc[[1,0,2],:]
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
