from cam_abstract import AbstractCam
from PySide import QtCore, QtGui, QtOpenGL
from OpenGL.GL import *
import numpy as np
from util import *


class ObjCam(AbstractCam):
    def __init__(self, glformat, parent=None, sharewidget=None):
        super(ObjCam, self).__init__(glformat, parent, sharewidget=sharewidget)

        self.widget_world = sharewidget
        self.idx_plane_selected_last = -1
        self.str_log_last = ''

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateGL)
        timer.start(100)  # 10 fps

    #override
    def paintGL(self):
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()  # must be this location!! -------------------
        glLoadIdentity()

        glClearColor(.8, .6, .8, 1.0)  # default background color
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        pos_v, rot_v, stamp = self.get_pose_virt()
        AbstractCam.set_mat_mv(pos_v, rot_v)

        self.draw_mesh_obj()

        str_log = self.get_log_text()
        if str_log != self.str_log_last:
            print str_log
            self.log_set_text(str_log)
            self.str_log_last = str_log

        glPopMatrix()  # must be this location!! -------------------

    #override
    def get_log_text(self):
        str_pose = super(ObjCam, self).get_log_text()
        str_len_vbo_index = 'len_vbo_indices: %s' % util_format_k(self.widget_world.len_vbo_indices_obj)
        str_npts_cloud = 'npts_cloud: %s' % util_format_k(self.widget_world.get_mesh_info('npts_cloud_obj'))
        str_d_ave = 'd_ave_cm: %s' % util_format_2f(self.widget_world.get_mesh_info('d_ave_obj'))
        str_d_std = 'd_std_cm: %s' % util_format_2f(self.widget_world.get_mesh_info('d_std_obj'))
        return '%s %s %s %s %s' % (str_pose, str_len_vbo_index, str_npts_cloud, str_d_ave, str_d_std)
