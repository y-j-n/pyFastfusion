import os
import errno
import shutil
import sys
import math
import numpy as np
from numpy import linalg as LA

from PySide import QtCore, QtGui, QtOpenGL

import time
from contextlib import contextmanager


# http://stackoverflow.com/questions/2327719/timing-block-of-code-in-python-without-putting-it-in-a-function
@contextmanager
def util_measure_time(title):
    t1 = time.clock()
    yield
    t2 = time.clock()
    print '%s: %0.2f seconds elapsed' % (title, t2-t1)


# http://stackoverflow.com/questions/1823058/how-to-print-number-with-commas-as-thousands-separators
def util_format_k(num):
    return '{:,}'.format(num)


def util_format_2f(num):
    return '{0:.2f}'.format(num)


def util_format_3f(num):
    return '{0:.3f}'.format(num)


# http://stackoverflow.com/questions/13199126/find-opengl-rotation-matrix-for-a-plane-given-the-normal-vector-after-the-rotat
def util_calc_plane_rot_trans(li_abcd):
    # calculate rotation
    n = np.array([0, 0, 1])
    norm_abc = LA.norm(li_abcd[0:3])
    nn = np.array(li_abcd[0:3]) / norm_abc
    rot_axis = np.cross(n, nn)
    rot_angle = math.degrees(math.acos(np.dot(n, nn)))

    # calculate translation
    # dist_abs_from_origin = abs(li_abcd[3]) / norm_abc
    dist_abs_from_origin = -li_abcd[3] / norm_abc
    trans = dist_abs_from_origin * nn
    # print trans

    return rot_axis, rot_angle, trans


def util_create_empty_dir(dirname):
    shutil.rmtree(dirname, ignore_errors=True)
    try:
        os.makedirs(dirname)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise


def util_qimage_to_ndarray(qimage):
    image_in = qimage.convertToFormat(QtGui.QImage.Format.Format_RGB32)
    width = image_in.width()
    height = image_in.height()
    print width, height
    # http://stackoverflow.com/questions/19902183/qimage-to-numpy-array-using-pyside
    # ptr = image_in.bits()
    ptr = image_in.constBits()
    print image_in.byteCount()  # 1228800

    if sys.platform == 'darwin':
        arr = np.array(ptr).reshape(height, width, 4)  # Copies the data
        # print np.array(ptr).shape  # (1228800,) i.e. 640*480*4
    else:
        print type(ptr)  # 'buffer'
        # http://stackoverflow.com/questions/11760095/convert-binary-string-to-numpy-array
        # http://docs.scipy.org/doc/numpy/reference/arrays.dtypes.html
        arr = np.frombuffer(ptr, dtype='b')
        print arr  # fixme: color is broken........................
        print arr.size
        print arr.shape
        arr = arr.reshape(height, width, 4)

    # http://docs.scipy.org/doc/numpy/reference/generated/numpy.delete.html
    # remove alpha channel
    #print arr.shape  # (480, 640, 4)
    arr = np.delete(arr, np.s_[3], 2)
    #print arr.shape  # (480, 640, 3)
    return arr
