#!/usr/bin/env python

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt

import time
import sys
import numpy as np

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl


def get_fig_2d(list_hor, list_list_ver, title='title'):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    list_colors = ['r', 'g', 'b', 'c', 'm', 'y']

    for i, list_ver in enumerate(list_list_ver):
        color = list_colors[i % 6]
        ax.plot(list_hor, list_ver, c=color)

    ax.set_xlabel('list_hor')
    ax.set_ylabel('list_ver')
    ax.set_title(title)

    return fig


def get_fig_scatter3d(list_x, list_y, list_z, title='title', scale=[-1, 1]):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xs = np.array(list_x)
    ys = np.array(list_y)
    zs = np.array(list_z)

    ax.scatter(xs, ys, zs, c='r', marker='o')
    ax.auto_scale_xyz(scale, scale, scale)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)

    return fig


def cut_cols(list_target_cols, filename, delimiters=' '):
    dict_list_cols = {}
    for col_index in list_target_cols:
        dict_list_cols[col_index] = []
    #print dict_list_cols

    with open(filename) as f:
        for line in f:
            if line.startswith('#'):
                continue

            #time, tx, ty, tz, qx, qy, qz, qw = l.split()
            list_nums = line.split(delimiters)

            for target in dict_list_cols.keys():
                #print target
                list_col = dict_list_cols[target]
                num = float(list_nums[target])
                list_col.append(num)

    return dict_list_cols


def plot_poscam():
    #filename = '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt'
    #filename = '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/assoc_opt_traj_final.txt'

    # dict_list_cols = cut_cols([0, 1, 2], ',log-posCam')
    # list_x = dict_list_cols[0]
    # list_y = dict_list_cols[1]
    # list_z = dict_list_cols[2]

    # dict_list_cols = cut_cols([2, 3, 4], 'log-poscam--m200')
    # list_x = dict_list_cols[2]#[9:19]
    # list_y = dict_list_cols[3]#[9:19]
    # list_z = dict_list_cols[4]#[9:19]

    #dict_list_cols = cut_cols([2, 3, 4], 'log-poscam--ambassador')
    # dict_list_cols = cut_cols([2, 3, 4], 'log-poscam--orig-mod')
    dict_list_cols = cut_cols([2, 3, 4], 'log-poscam')
    list_x = dict_list_cols[2]
    list_y = dict_list_cols[3]
    list_z = dict_list_cols[4]



    #print dict_list_cols
    #exit()


    fig_cam_xyz = get_fig_2d(range(len(list_x)), [list_x, list_y, list_z], 'camXYZ')
    fig_cam_traj = get_fig_scatter3d(list_x, list_y, list_z, 'camTraj')

    #fig.show()
    #time.sleep(120)

    plt.show()  # show all figs created


def plot_kp():
    if 0:
        #filename = 'log-k_P'
        #filename = 'log-k_P-freiburg3'
        filename = 'log-kp-grass'
        #filename = 'log-kp-sq'


        col = cut_cols([3, 4, 5], filename)  # col is a dictionary
        fig = get_fig_scatter3d(col[3], col[4], col[5], 'k_P')
        #fig.show()
        #time.sleep(120)

        plt.show()  # show all figs created

    if 1:
        #filename = '2.610428_raw.csv'
        #filename = '2.610428_model.csv'
        filename = '2.744492_model.csv'
        col = cut_cols([2, 3, 4], filename, ';')
        fig = get_fig_scatter3d(col[2], col[3], col[4], filename)
        plt.show()  # show all figs created


def test_GLScatterPlotItem():
    app = QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.opts['distance'] = 20
    w.show()
    w.setWindowTitle('pyqtgraph example: GLScatterPlotItem mods')

    g = gl.GLGridItem()
    w.addItem(g)

    #-----------------

    if 0:
        sp_origin = gl.GLScatterPlotItem(
            pos=np.array([(0, 0, 0)]), size=0.5,
            color=(1.0, 0.0, 0.0, 0.5), pxMode=False)
        #sp_origin.translate(5, 5, 0)
        w.addItem(sp_origin)

    if 0:
        # http://www.pyqtgraph.org/documentation/3dgraphics/glscatterplotitem.html
        # orig: ../opengl/pyqtgraph-develop/examples/GLScatterPlotItem.py
        pos = np.zeros((3, 3))
        size = np.zeros((3))
        color = np.zeros((3, 4))
        pos[0] = (0,0,0)
        pos[1] = (1,1,1)
        pos[2] = (2,2,2)
        size[0] = 0.5
        size[1] = 0.2
        size[2] = 2./3.
        color[0] = (1.0, 0.0, 0.0, 0.5)
        color[1] = (0.0, 0.0, 1.0, 0.5)
        color[2] = (0.0, 1.0, 0.0, 0.5)
        print pos
        sp1 = gl.GLScatterPlotItem(pos=pos, size=size, color=color, pxMode=False)
        sp1.translate(5,5,0)
        w.addItem(sp1)

    if 0:
        filename = '2.403822_model_large.csv'
        bbox = (15, 55, 264, 370)
        # filename = '6.961998_model_large.csv'
        # bbox = (306, 103, 594, 435)

        print 'filename: '+filename
        col = cut_cols([0, 1, 2, 3, 4, 5, 6, 7], filename, ';')
        npa_ij = np.array([col[0], col[1]]).transpose()

        print 'finding indices in bbox...'
        li_indices_bb = []
        i_min = bbox[0]
        i_max = bbox[2]
        j_min = bbox[1]
        j_max = bbox[3]
        for index, ij in enumerate(npa_ij):
            if i_min <= ij[0] <= i_max:
                if j_min <= ij[1] <= j_max:
                    li_indices_bb.append(index)
        print '# of pixels in bbox: %d' % len(li_indices_bb)

        xyz = np.array([col[2], col[3], col[4]]).transpose()
        xyz_bb = xyz[li_indices_bb, :]
        # print xyz
        # print xyz_bb

        if 1:
            csv_out = 'renameme_bbox.csv'
            print 'dumping %s ...' % csv_out
            npa_csv = np.array([col[0], col[1],
                                col[2], col[3], col[4],
                                col[5], col[6], col[7]]).transpose()
            with open(csv_out, 'w') as f:
                for index in li_indices_bb:
                    vals = npa_csv[index, :]
                    f.write('%d;%d;%f;%f;%f;%d;%d;%d\n' % (vals[0], vals[1],
                                                           vals[2], vals[3], vals[4],
                                                           vals[5], vals[6], vals[7]))
            print 'done.'

        rgb = [(1.0, 0.0, 0.0, 0.5),
               (0.0, 1.0, 0.0, 0.5),
               (0.0, 0.0, 1.0, 0.5)]
        w.addItem(gl.GLScatterPlotItem(pos=xyz, size=2.0, color=rgb[0], pxMode=True))
        w.addItem(gl.GLScatterPlotItem(pos=xyz_bb, size=2.0, color=rgb[1], pxMode=True))

    if 1:
        filename = 'ellispsoid.csv'
        print 'filename: '+filename
        col = cut_cols([0, 1, 2], filename, ';')
        xyz = np.array([col[0], col[1], col[2]]).transpose()
        print '# of points:'
        print xyz.shape
        rgb = [(1.0, 0.0, 0.0, 0.5),
               (0.0, 1.0, 0.0, 0.5),
               (0.0, 0.0, 1.0, 0.5)]
        w.addItem(gl.GLScatterPlotItem(pos=xyz, size=2.0, color=rgb[0], pxMode=True))

    #-----------------
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
    #plot_poscam()
    #plot_kp()
    test_GLScatterPlotItem()
