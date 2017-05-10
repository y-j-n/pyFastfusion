#!/usr/bin/env python
#  -*- coding: utf-8 -*-
"""
Demonstrate use of GLLinePlotItem to draw cross-sections of a surface.

"""
## Add path to library (just for examples; you do not need this)
# import initExample

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import math
import os
import pcl  # using a local symlink to ../python-pcl-master/pcl
from util import util_calc_plane_rot_trans


app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.opts['distance'] = 4
w.show()
w.setWindowTitle('draw x + y + z - 1 = 0')

if 0:
    gx = gl.GLGridItem()
    gx.rotate(90, 0, 1, 0)
    # gx.translate(-10, 0, 0)
    w.addItem(gx)
    # ----
    gy = gl.GLGridItem()
    gy.rotate(90, 1, 0, 0)
    # gy.translate(0, -10, 0)
    # ----
    w.addItem(gy)
    gz = gl.GLGridItem()
    # gz.translate(0, 0, -10)
    w.addItem(gz)

# draw x + y + z - 1 = 0
# http://stackoverflow.com/questions/13199126/find-opengl-rotation-matrix-for-a-plane-given-the-normal-vector-after-the-rotat
# n = (0 0 1)
# n' = (1 1 1) / sqrt(3)
# rotAxis = cross(n, n') = (-1 1 0) / sqrt(3)
# rotAngle = acos(dot(n, n'))
gg = gl.GLGridItem()
gg.rotate(math.degrees(math.acos(3**(-0.5))), -1, 1, 0)
gg.translate(1./3, 1./3, 1./3)
w.addItem(gg)

# origin
sp_origin = pg.opengl.GLScatterPlotItem(
    pos=np.array([(0, 0, 0)]), size=0.1,
    color=(1.0, 0.0, 0.0, 0.5), pxMode=False)
w.addItem(sp_origin)

# three support points
sp_pts = pg.opengl.GLScatterPlotItem(
    pos=np.array([(1, 0, 0),(0, 1, 0),(0, 0, 1)]), size=0.1,
    color=(0.0, 1.0, 0.0, 0.5), pxMode=False)
w.addItem(sp_pts)

# ---------------------------
dirname = './tmp_planes/'
path_models_file = dirname + 'models.txt'
if not os.path.isfile(path_models_file):
    print 'No planar models found...'
else:
    npa_pcd_files = np.loadtxt(path_models_file, usecols=(0,), dtype='S')
    npa_pcd_points = np.loadtxt(path_models_file, usecols=(1,), dtype='i4')
    npa_abcd = np.loadtxt(path_models_file, usecols=(2, 3, 4, 5), dtype='f4')

    # plot a planar point cloud
    pc = pcl.load(dirname + npa_pcd_files[0])  # the most dominant plane......only
    npa_pc = np.asarray(pc)
    print npa_pc.shape
    w.addItem(pg.opengl.GLScatterPlotItem(
        pos=npa_pc, size=0.05,
        color=(1.0, 1.0, 0.0, 0.5), pxMode=False))

    # plot a grid that fits the point cloud
    gg = gl.GLGridItem()
    rot_axis, rot_angle, trans = util_calc_plane_rot_trans(npa_abcd[0])
    gg.rotate(rot_angle, rot_axis[0], rot_axis[1], rot_axis[2])
    gg.translate(trans[0], trans[1], trans[2])
    w.addItem(gg)




# ---------------------------

# def fn(x, y):
#     return np.cos((x**2 + y**2)**0.5)
#
# n = 51
# y = np.linspace(-10,10,n)
# x = np.linspace(-10,10,100)
# for i in range(n):
#     yi = np.array([y[i]]*100)
#     d = (x**2 + yi**2)**0.5
#     z = 10 * np.cos(d) / (d+1)
#     pts = np.vstack([x,yi,z]).transpose()
#     plt = gl.GLLinePlotItem(pos=pts, color=pg.glColor((i,n*1.3)), width=(i+1)/10., antialias=True)
#     w.addItem(plt)
    


## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
