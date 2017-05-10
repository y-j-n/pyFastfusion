from glwidget_abstract import GLWidgetAbstract
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
# following pyqtgraph-develop/pyqtgraph/opengl/GLViewWidget.py
import OpenGL.GL.framebufferobjects as glfbo

from PySide import QtCore, QtGui, QtOpenGL
import numpy as np
import cv2
import cols_plot as cols


class AbstractCam(GLWidgetAbstract):
    def __init__(self, glformat, parent=None, sharewidget=None):
        super(AbstractCam, self).__init__(glformat, parent, sharewidget=sharewidget)

        self.widget_world = sharewidget

        self.trans_eye = [0.0, 0.0, 0.0]
        self.rot_eye = [0.0, 0.0, 0.0]
        self.log_set_text(self.get_log_text())

        self.tex = 0
        self.fbo = 0

        self.depth_rendered = None

    def __del__(self):
        self.makeCurrent()

        glDeleteBuffers(1, [self.tex])
        glDeleteBuffers(1, [self.fbo])

    #override
    def initializeGL(self):
        super(AbstractCam, self).initializeGL()
        print '[cam] initializeGL(): hello'

        self.mode_polygon = GL_FILL

        self.tex = glGenTextures(1)
        print '[cam] self.tex: %s' % self.tex

        # ref: songho/fboDepth/main.cpp
        glBindTexture(GL_TEXTURE_2D, self.tex)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
        #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        #glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE)
        # use None for 0 in PyOpenGL !!
        # glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
        #              self.viewport_width, self.viewport_height,
        #              0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                     # self.viewport_width, self.viewport_height,  # broken image
                     640, 640,  # fixme: how to get correct 640x480 depth texture?
                     0, GL_DEPTH_COMPONENT, GL_FLOAT, None)
        #glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, self.viewport_width, self.viewport_height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, None)
        glBindTexture(GL_TEXTURE_2D, 0)  # unbind

        self.fbo = glfbo.glGenFramebuffers(1)
        print '[cam] self.fbo: %s' % self.fbo
        glfbo.glBindFramebuffer(glfbo.GL_FRAMEBUFFER, self.fbo)
        # the last arg is mipmap level.  http://pyopengl.sourceforge.net/documentation/manual-3.0/glFramebufferTexture.html
        glfbo.glFramebufferTexture2D(glfbo.GL_FRAMEBUFFER, glfbo.GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, self.tex, 0)
        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)
        glfbo.glBindFramebuffer(glfbo.GL_FRAMEBUFFER, 0)  # unbind

        if 1:
            AbstractCam.set_mat_proj_xtion()
        else:
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(90.0, 1.0, 1.0, 100.0)

    #override
    def resizeGL(self, width, height):
        print '[cam] resizeGL(): called with %s %s' % (width, height)
        print '[cam] resizeGL(): setting viewport with: 0 0 640 480'
        glViewport(0, 0, 640, 480)

    #override
    def wheelEvent(self, event):
        delta_wheel = event.delta()
        self.translate(0, 0, 1.0 if delta_wheel > 0 else -1.0)

    #override
    def mouseMoveEvent(self, event):
        dx = event.x() - self.last_pos_mouse.x()
        dy = event.y() - self.last_pos_mouse.y()
        delta = 0.25
        delta_rot = 0.5

        if event.buttons() & QtCore.Qt.LeftButton:
            if abs(dx) > abs(dy):
                self.rotate(0, delta_rot if dx > 0 else -delta_rot, 0)
            else:
                self.rotate(-delta_rot if dy > 0 else delta_rot, 0, 0)
        # elif event.buttons() & QtCore.Qt.RightButton:
            # reserve for euclid measurement, use wheels instead for z-trans...
            # self.translate(0, 0, -delta if dy > 0 else delta)
        elif event.buttons() & QtCore.Qt.MidButton:
            if abs(dx) > abs(dy):
                self.translate(delta if dx > 0 else -delta, 0, 0)
            else:
                self.translate(0, -delta if dy > 0 else delta, 0)

        self.last_pos_mouse = event.pos()

    def get_log_text(self):
        return '{trans,rot}_eye: %g %g %g %g %g %g' % tuple(self.trans_eye + self.rot_eye)

    def translate(self, dx, dy, dz):
        self.trans_eye[0] += dx
        self.trans_eye[1] += dy
        self.trans_eye[2] += dz
        self.log_set_text(self.get_log_text())

    def rotate(self, dthx, dthy, dthz):
        self.rot_eye[0] += dthx
        self.rot_eye[1] += dthy
        self.rot_eye[2] += dthz
        self.log_set_text(self.get_log_text())

    def clear_pose_eye(self):
        self.trans_eye = [0.0, 0.0, 0.0]
        self.rot_eye = [0.0, 0.0, 0.0]
        self.log_set_text(self.get_log_text())

    def get_pose_eye(self):
        return self.trans_eye, self.rot_eye

    def get_pose_virt(self):
        pos, rot, stamp, pose_virt = self.widget_world.get_caminfo_selected()
        if pose_virt is None:
            self.clear_pose_eye()

        # take into account cam's virtual (translational) movement
        npa_rot = np.array([[rot[0], -rot[1], -rot[2]],
                            [rot[3], -rot[4], -rot[5]],
                            [rot[6], -rot[7], -rot[8]]])
        s = np.sin(np.array(self.rot_eye) / 180. * np.pi)
        c = np.cos(np.array(self.rot_eye) / 180. * np.pi)
        # http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rot_x = np.array([[1, 0, 0], [0, c[0], -s[0]], [0, s[0], c[0]]])
        rot_y = np.array([[c[1], 0, s[1]], [0, 1, 0], [-s[1], 0, c[1]]])
        # rot_z = np.array([[c[2], -s[2], 0], [s[2], c[2], 0], [0, 0, 1]])  # not used

        # fixme: after rotaion, 3d pos is strange........?????
        npa_rot_new = np.dot(rot_y, np.dot(rot_x, npa_rot))  # order of rot ok??
        pos_virt = pos + np.dot(npa_rot_new, -np.array(self.trans_eye))
        rot_virt = [
            npa_rot_new[0][0], -npa_rot_new[0][1], -npa_rot_new[0][2],
            npa_rot_new[1][0], -npa_rot_new[1][1], -npa_rot_new[1][2],
            npa_rot_new[2][0], -npa_rot_new[2][1], -npa_rot_new[2][2]]
        return pos_virt, rot_virt, stamp

    def draw_mesh_obj(self):
        stat = self.widget_world.get_mesh_info('stat')
        # print 'draw_mesh_obj(): stat (world): ' + stat
        if self.widget_world.is_first_mesh_uploaded and stat == 'uploaded':
            self.draw_indexed_vbo(self.widget_world.vbo_indices_obj,
                                  self.widget_world.vbo_data,
                                  self.mode_polygon,
                                  self.widget_world.len_vbo_indices_obj)

    def draw_mesh_world(self):
        stat = self.widget_world.get_mesh_info('stat')
        # print 'draw_mesh_world(): stat (world): ' + stat
        # if self.widget_world.is_first_mesh_uploaded and stat != 'ready':
        if self.widget_world.is_first_mesh_uploaded and stat == 'uploaded':
            self.draw_indexed_vbo(self.widget_world.vbo_indices,
                                  self.widget_world.vbo_data,
                                  self.mode_polygon,
                                  self.widget_world.len_vbo_indices)

    def grab_depth_rendered(self):
        glfbo.glBindFramebuffer(glfbo.GL_FRAMEBUFFER, self.fbo)
        # clear buffer
        glClearColor(1, 1, 1, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.draw_mesh_world()
        # if gear3 > 0:
        #     self.draw_sth(gear3, (-2.0, -2.0, 10.0), mode=GL_FILL)
        # if cam_fake > 0:
        #     self.draw_sth(cam_fake, cam_fake_pos)

        glfbo.glBindFramebuffer(glfbo.GL_FRAMEBUFFER, 0)  # unbind

        glBindTexture(GL_TEXTURE_2D, self.tex)
        level_mipmap = 0
        # http://bazaar.launchpad.net/~mcfletch/openglcontext/trunk/view/head:/tests/getteximage.py
        img = glGetTexImage(GL_TEXTURE_2D, level_mipmap,
                            GL_DEPTH_COMPONENT, GL_FLOAT)
        #width = glGetTexLevelParameteriv(GL_TEXTURE_2D, level_mipmap, GL_TEXTURE_WIDTH)
        #height = glGetTexLevelParameteriv(GL_TEXTURE_2D, level_mipmap, GL_TEXTURE_HEIGHT)
        glBindTexture(GL_TEXTURE_2D, 0)  # unbind

        # print width, height
        # print type(img_sq_640)  # should be <type 'numpy.ndarray'>
        # print img_sq_640
        cv2.flip(img, 0, img)
        # http://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python
        img_crop = img[160:640, 0:640]  # 640x640 -> 640x480
        #print img_crop.size  # 307200
        return img_crop

    @staticmethod
    def get_intrinsics_xtion():
        return 525.0, 525.0, 319.5, 239.5  # fx, fy, cx, cy

    @staticmethod
    def set_mat_proj_xtion():
        # assuming viewport is set as: glViewport(0, 0, 640, 480)

        glMatrixMode(GL_PROJECTION)

        # Asus Xtion Pro
        fx, fy, cx, cy = AbstractCam.get_intrinsics_xtion()
        # near = 0.1  # depth texture not visible...
        near = 2.0  # fixme: adjust this...
        far = 100.0

        # http://stackoverflow.com/questions/22064084/how-to-create-perspective-projection-matrix-given-focal-points-and-camera-princ
        # https://github.com/visionegg/visionegg/blob/master/VisionEgg/Core.py
        mat_proj = np.array([[fx/cx, 0.,    0.,                     0.],
                             [0.,    fy/cy, 0.,                     0.],
                             [0.,    0.,    -(far+near)/(far-near), -2*far*near/(far-near)],
                             [0.,    0.,    -1.0,                   0.]])
        mat_proj = np.transpose(mat_proj)  # OpenGL's format
        glLoadMatrixf(mat_proj)

    @staticmethod
    def set_mat_mv(pos, rot):
        if 1:
            # ######## this works!!
            # matches actuall images taken: associate_gt_skip.txt
            rotmat_4x4 = np.array([
                [rot[0], -rot[1], -rot[2], 0],
                [rot[3], -rot[4], -rot[5], 0],
                [rot[6], -rot[7], -rot[8], 0],
                [0.,     0.,     0.,     1.]])
            glMultMatrixf(rotmat_4x4)
            glTranslatef(-pos[0], -pos[1], -pos[2])
            # print pos
            # self.dump_mat_mv()

        if 0:
            # ######## BROKEN old version; crap
            # not matching images taken: associate_gt_skip.txt
            # https://www.opengl.org/discussion_boards/showthread.php/135456-Problem-with-gluLookAt
            # gluLookAt() conflicting with PushMatrix() above??
            rotmat_4x4 = np.array([
                [rot[0], rot[1], rot[2], 0],
                [rot[3], rot[4], rot[5], 0],
                [rot[6], rot[7], rot[8], 0],
                [0.,     0.,     0.,     1.]])
            rotmat_4x4 = np.transpose(rotmat_4x4)  # OpenGL's format
            glMultMatrixf(rotmat_4x4)
            # first, place the cam pointing +z-direction with -1 y-up-vector
            gluLookAt(pos[0], pos[1], pos[2],  # eyepoint
                      pos[0], pos[1], pos[2]+10,  # center-of-view
                      0, -1, 0)  # up-vector
            # print pos
            # self.dump_mat_mv()

        # if z_eye is not None:
        #     #http://stackoverflow.com/questions/3380100/how-do-i-use-glulookat-properly
        #     #(The intuition behind the "up" vector in gluLookAt is simple:
        #     # Look at anything. Now tilt your head 90 degrees. Where you
        #     # are hasn't changed, the direction you're looking at hasn't
        #     # changed, but the image in your retina clearly has. What's the
        #     # difference? Where the top of your head is pointing to. That's
        #     # the up vector.)
        #     gluLookAt(0, 0, z_eye,  # eyepoint
        #               0, 0, 60,  # center-of-view
        #               0, -1, 0)  # up-vector

    @staticmethod
    def apply_mask(img_gray, mask_gray):
        thresh = 1
        max_intensity = 1
        mask_bw = cv2.threshold(mask_gray, thresh, max_intensity, cv2.THRESH_BINARY)[1]
        # cv2.imshow('mask_bw', mask_bw)
        # print mask_bw.shape, mask_bw
        return cv2.bitwise_and(img_gray, img_gray, mask=mask_bw)

    @staticmethod
    def read_shape_file(shape_file):
        # todo: use np.loadtxt() instead!!!  http://docs.scipy.org/doc/numpy/reference/generated/numpy.loadtxt.html
        col = cols.cut_cols([0, 1, 2], shape_file, ';')
        xyz = np.array([col[0], col[1], col[2]]).transpose()
        return xyz

    @staticmethod
    def get_depth_mask(depth, dir_mask, stamp):
        path_mask = dir_mask + '/%s.png' % stamp
        # http://stackoverflow.com/questions/7624765/converting-an-opencv-image-to-black-and-white
        mask_gray = cv2.imread(path_mask, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        if mask_gray is None:
            print 'get_depth_mask(): failed to get mask_gray.  using entire depth...'
            return depth
        else:
            cv2.imshow('mask_gray', mask_gray)
            return AbstractCam.apply_mask(depth, mask_gray)

    @staticmethod
    def get_depth_rendered_mask(depth_rendered, dir_mask, stamp):
        depth_masked = AbstractCam.get_depth_mask(
            depth_rendered, dir_mask, stamp)
        # The range of depth_rendered is [0,1]
        # invalidate zeros as 2
        depth_masked[depth_masked < 0.0001] = 2
        cv2.imshow('depth_masked', depth_masked)
        return depth_masked
