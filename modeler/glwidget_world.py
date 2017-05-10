from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
# following pyqtgraph-develop/pyqtgraph/opengl/GLViewWidget.py
import OpenGL.GL.framebufferobjects as glfbo

from ctypes import *
import threading
import custom_threading
import urllib
import glob

from glwidget_abstract import GLWidgetAbstract
from util import *
from bin_if import *


class GLWidgetWorld(GLWidgetAbstract):
    xRotationChanged = QtCore.Signal(int)
    yRotationChanged = QtCore.Signal(int)
    zRotationChanged = QtCore.Signal(int)

    def __init__(self, glformat, parent=None, sharewidget=None):
        super(GLWidgetWorld, self).__init__(
            glformat, parent, sharewidget=sharewidget)

        self.datasetdir_edit = QtGui.QLineEdit()
        self.datasetdir_edit.setReadOnly(True)

        # http://effbot.org/zone/thread-synchronization.htm
        # http://stackoverflow.com/questions/5185568/python-conditional-with-lock-design
        self.lock = threading.RLock()

        # VBOs (0=invalid)
        self.vbo_indices = 0
        self.vbo_indices_obj = 0
        self.vbo_data = 0
        self.len_vbo_indices = -1
        self.len_vbo_indices_obj = -1
        self.__mesh_data_current = {
            'npa_vbo_index': np.array([0, 0, 0]),
            'npa_vbo_index_obj': np.array([0, 0, 0]),
            'npts_cloud_obj': -1,
            'd_ave_obj': -1,
            'd_std_obj': -1,
            'npa_vbo_vertex': np.array([0, 0, 0, 0, 0, 0]),
            'stat': 'invalidated',
        }
        self.is_first_mesh_uploaded = False

        self.load_world_posted = None
        self.save_world_posted = None
        self.clear_world_posted = None
        self.build_world_posted = None
        self.upload_vbo_index_obj_posted = None

        # Display lists. (0=invalid)
        self.axes = 0
        self.cam = 0
        self.gear1 = 0
        self.gear2 = 0
        self.gear3 = 0

        self.cam_fake = 0
        self.cam_fake_pos = (0, 0, -114.0)  # make sure: near < Z < far in set_mat_proj_xtion()
        # self.cam_fake_pos = (0, 0, 10.1819)  # make sure: near < Z < far in set_mat_proj_xtion()
        ## carmine 1.09: 50cm matches 1.0 in ff's unit
        # self.cam_fake_pos = (0, 0, 10.0)  # make sure: near < Z < far in set_mat_proj_xtion()

        self.scale = 1.0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.gear1Rot = 0

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.advanceGears)
        timer.start(20)  # 50 fps

        self.stream_poscam = self.fifo_poscam()
        self.list_traj_cam_svo = []

        self.datasetdir = ''
        self.list_traj_cam_fusion = []
        self.list_rot_cam_fusion = []
        self.list_stamp_cam_fusion = []
        self.cam_selected = None
        self.tpl_pose_cam_virt = None
        self.mask_raw_enabled = False
        self.show_traj = True

        self.ff = FastFusion()
        self.ff_thread = None

        if 0:
            th = threading.Thread(target=self.svo_thread, args=())
            th.daemon = True
            th.start()

        if 0:
            th = threading.Thread(target=self.fusion_thread, args=(self.ff,))
            th.daemon = True
            th.start()

    def __del__(self):
        #http://qt-project.org/doc/qt-4.8/qglwidget.html#makeCurrent
        self.makeCurrent()

        # http://pyopengl.sourceforge.net/documentation/manual-3.0/glDeleteBuffers.html
        # https://bitbucket.org/rndblnch/opengl-programmable/src/tip/08-pbo.py
        glDeleteBuffers(1, [self.vbo_indices])
        glDeleteBuffers(1, [self.vbo_indices_obj])
        glDeleteBuffers(1, [self.vbo_data])

        # https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/glGenLists.3.html
        glDeleteLists(self.axes, 1)
        glDeleteLists(self.cam, 1)
        glDeleteLists(self.gear1, 1)
        glDeleteLists(self.gear2, 1)
        glDeleteLists(self.gear3, 1)
        glDeleteLists(self.cam_fake, 1)

    def get_datasetdir_edit(self):
        return self.datasetdir_edit

    def get_datasetdir_assoc(self):
        path_assoc = self.datasetdir + '/associate.txt'
        has_assoc = True if glob.glob(path_assoc) else False
        c_frames = -1
        if has_assoc:
            # http://stackoverflow.com/questions/845058/how-to-get-line-count-cheaply-in-python
            with open(path_assoc) as f:
                c_frames = sum(1 for line in f)
        # print 'has_assoc: %r' % has_assoc
        return has_assoc, path_assoc, c_frames

    def datasetdir_set(self, dirpath):
        self.datasetdir = dirpath

        # for model data
        path_vertex = dirpath + '/npa_vbo_vertex.npy'
        has_model = glob.glob(path_vertex)
        if has_model:
            print 'npa_vbo_vertex.npy: found'
            # self.datasetdir_edit.setText() is called inside load_world()
            self.post_load_world(dirpath)
        else:
            print 'npa_vbo_vertex.npy: not found'
            self.datasetdir_edit.setText('[model: not found] %s' % dirpath)
            self.post_clear_world()

        # for rgbd data
        return self.get_datasetdir_assoc()

    def toggle_raw_mask(self):
        self.mask_raw_enabled = not self.mask_raw_enabled
        print '[world] toggle_raw_mask(): self.mask_raw_enabled: %s' % self.mask_raw_enabled

    def set_pose_cam_virt(self, pos, rot):
        self.tpl_pose_cam_virt = (pos, rot)

    def get_raw_images(self, stamp):
        if self.datasetdir is None:
            print 'get_raw_images(): datasetdir not available. bye'
            return None, None

        path_associate = self.datasetdir + '/associate.txt'
        if not os.path.isfile(path_associate):
            print 'get_raw_images(): %s not found!!' % path_associate
            return None, None
        else:
            print 'found: ' + path_associate

        npa_stamp_depth_rgb = np.loadtxt(path_associate, usecols=(0, 9, 11), dtype='S')
        # print npa_stamp_depth_rgb, npa_stamp_depth_rgb.shape
        idx = np.where(npa_stamp_depth_rgb == stamp)[0][0]
        print stamp, idx
        print npa_stamp_depth_rgb[idx]
        try:
            path_depth = self.datasetdir + '/' + npa_stamp_depth_rgb[idx][1]
            path_rgb = self.datasetdir + '/' + npa_stamp_depth_rgb[idx][2]
        except IndexError:
            print 'get_raw_images(): stamp not found in %s!!' % path_associate
            return None, None

        if not os.path.isfile(path_depth) or not os.path.isfile(path_rgb):
            print 'get_raw_images(): img(s) not found %s %s!!' % (path_depth, path_rgb)
            return None, None
        else:
            print 'found: ' + path_depth
            print 'found: ' + path_rgb

        depth = cv2.imread(path_depth, cv2.IMREAD_UNCHANGED)
        rgb = cv2.imread(path_rgb, cv2.IMREAD_COLOR)  # no alpha
        return depth, rgb

    def get_caminfo_selected(self):
        index = -1 if self.cam_selected is None else self.cam_selected

        list_pos = self.list_traj_cam_fusion
        pos = list_pos[index] if list_pos else np.array([0, 0, 0])
        list_rot = self.list_rot_cam_fusion
        rot = list_rot[index] if list_rot else np.array([1, 0, 0, 0, 1, 0, 0, 0, 1])
        list_stamp = self.list_stamp_cam_fusion
        stamp = list_stamp[index] if list_stamp else ''
        return pos, rot, stamp, self.tpl_pose_cam_virt

    def get_mesh_info(self, k):
        return self.__mesh_data_current[k]

    def set_mesh_info(self, k, v):
        self.__mesh_data_current[k] = v

    def setXRotation(self, angle):
        self.normalizeAngle(angle)

        if angle != self.xRot:
            self.xRot = angle
            self.xRotationChanged.emit(angle)
            self.updateGL()

    def setYRotation(self, angle):
        self.normalizeAngle(angle)

        if angle != self.yRot:
            self.yRot = angle
            self.yRotationChanged.emit(angle)
            self.updateGL()

    def setZRotation(self, angle):
        self.normalizeAngle(angle)

        if angle != self.zRot:
            self.zRot = angle
            self.zRotationChanged.emit(angle)
            self.updateGL()

    #override
    def initializeGL(self):
        super(GLWidgetWorld, self).initializeGL()
        self.mode_polygon = GL_FILL

        print 'initializeGL(): hello'
        light_pos = (5.0, 5.0, 10.0, 1.0)
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)

        #ply_data = self.get_ply_data_from_file('untitled.ply')
        #ply_data = self.get_ply_data_from_file('current_upto_176.ply')
        #ply_data = self.get_ply_data_from_file('mesh-first-desk.ply', 10.0)

        reflectance_axes = (0.2, 1.2, 1.0, 1.0)
        reflectance_cam = (1.2, 0.2, 1.0, 1.0)
        reflectance1 = (0.8, 0.1, 0.0, 1.0)
        reflectance2 = (0.0, 0.8, 0.2, 1.0)
        reflectance3 = (0.2, 0.2, 1.0, 1.0)
        self.axes = self.make_axes(reflectance_axes)
        self.cam = self.make_cam(reflectance_cam)
        self.gear1 = self.makeGear(reflectance1, 1.0, 4.0, 1.0, 0.7, 20)
        self.gear2 = self.makeGear(reflectance2, 0.5, 2.0, 2.0, 0.7, 10)
        self.gear3 = self.makeGear(reflectance3, 1.3, 2.0, 0.5, 0.7, 10)

        # when turn this off, smooth shading for polygons enforced.  why???
        self.cam_fake = self.make_cam(reflectance_cam)

        print '[world] display list instances: %s %s %s %s %s' % (
            self.axes, self.cam, self.gear1, self.gear2, self.gear3
        )

        # fixme: how to refrect non attribute object colors in shader???
        # on linux, it's working fine...

        if 0:
            #fixme: osx gives only 1.20 legathy profile...
            print glGetString(GL_VERSION)
            print glGetString(GL_SHADING_LANGUAGE_VERSION)
            # pro:
            # 2.1 INTEL-8.28.32
            # 1.20
            #time.sleep(999)

        self.vbo_indices = glGenBuffers(1)
        self.vbo_indices_obj = glGenBuffers(1)
        self.vbo_data = glGenBuffers(1)
        print '[world] self.vbo_indices: %s' % self.vbo_indices
        print '[world] self.vbo_indices_obj: %s' % self.vbo_indices_obj
        print '[world] self.vbo_data: %s' % self.vbo_data

        # [ok] draw elements: c.f. opengl-programmable/07-attrib_my.py
        # test uploading and drawing two triangles
        if 1:
            print '[world] uploading test triangles (blue/sky)'
            # ----
            # indices = [0, 1, 2, 3, 4, 5]
            # vertices_raw = [(0.0, 1.0, 0.0),  (0.0, 0.0, 0.0), (1.0, 1.0, 0.0),
            #                 (0.0, -1.0, 1.0), (0.0, 0.0, 1.0), (-1.0, -1.0, 1.0)]
            # # blue and sky-blue
            # colors_raw = [(0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0),
            #               (0.0, 1.0, 1.0), (0.0, 1.0, 1.0), (0.0, 1.0, 1.0)]
            # -----------------------------------
            # 0-5: facing +Z, 6-11: facing -Z
            indices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
            indices_obj = indices[0:9]
            vertices_raw = [
                (0.0, 1.0, 10.0),  (0.0, 0.0, 10.0), (1.0, 1.0, 10.0),
                (0.0, -1.0, 11.0), (0.0, 0.0, 11.0), (-1.0, -1.0, 11.0),
                (0.0, 1.0, 10.0),  (1.0, 1.0, 10.0), (0.0, 0.0, 10.0),
                (0.0, -1.0, 11.0),  (-1.0, -1.0, 11.0), (0.0, 0.0, 11.0),
            ]
            # blue and sky-blue
            colors_raw = [
                (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0),
                (0.0, 1.0, 1.0), (0.0, 1.0, 1.0), (0.0, 1.0, 1.0),
                (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0),
                (0.0, 1.0, 1.0), (0.0, 1.0, 1.0), (0.0, 1.0, 1.0),
            ]
            # ----

            data = GLWidgetAbstract.flatten3(*zip(vertices_raw, colors_raw))
            #print data
            # x, y, z, r, g, b
            #[0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
            # 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            # 1.0, 1.0, 0.0, 0.0, 0.0, 1.0,
            # 0.0, -1.0, 0.0, 0.0, 1.0, 1.0,
            # 0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
            # -1.0, -1.0, 0.0, 0.0, 1.0, 1.0]

            self.set_mesh_info('npa_vbo_index', np.array(indices))
            self.set_mesh_info('npa_vbo_index_obj', np.array(indices_obj))
            self.set_mesh_info('npa_vbo_vertex', np.array(data))

            self.upload_indexed_vbo_indices(self.vbo_indices, np.array(indices))
            self.len_vbo_indices = len(np.array(indices))

            self.upload_indexed_vbo_indices(self.vbo_indices_obj, np.array(indices_obj))
            self.len_vbo_indices_obj = len(np.array(indices_obj))

            self.upload_indexed_vbo_data(self.vbo_data, np.array(data))

            self.set_mesh_info('stat', 'uploaded')
            self.is_first_mesh_uploaded = True

        # STREAM, STATIC, DYNAMIC: The usage flag is a hint, not a enforcement.
        # -- datenwolf http://stackoverflow.com/questions/8281653/how-to-choose-between-gl-stream-draw-or-gl-dynamic-draw
        # [ok] test following svo_slam_no_ros/opengl/songho/vbo/src/main.cpp
        # vertices_new = [0.0, 1.0, 0.0,  0.0, 0.0, 0.0,  1.0, 1.0, 0.0,
        #                 0.0, -2.0, 0.0,  0.0, 0.0, 0.0,  -2.0, -2.0, 0.0]
        # glBindBuffer(GL_ARRAY_BUFFER, self.vbo_data)
        # glBufferSubData(GL_ARRAY_BUFFER, 0, len(vertices_new)*4,
        #                 (c_float*len(vertices_new))(*vertices_new))
        # glBindBuffer(GL_ARRAY_BUFFER, 0)

    def __kf_clear(self):
        self.cam_selected = None

    def __kf_start(self):
        if self.cam_selected is not None:
            self.cam_selected = 0

    def __kf_end(self):
        if self.cam_selected is not None:
            self.cam_selected = len(self.list_traj_cam_fusion)-1

    def __kf_next(self):
        if self.cam_selected is not None:
            if self.cam_selected < len(self.list_traj_cam_fusion)-1:
                self.cam_selected += 1

    def __kf_prev(self):
        if self.cam_selected is not None:
            if self.cam_selected > 0:
                self.cam_selected -= 1

    di_func = {
        'start': __kf_start,
        'end': __kf_end,
        'next': __kf_next,
        'prev': __kf_prev}

    def kf_cmd(self, cmd):
        if cmd in self.di_func:
            self.di_func.get(cmd)(self)
            self.tpl_pose_cam_virt = None
            if self.cam_selected is not None:
                self.log_set_text('cam_selected: %s stamp: %s' % (self.cam_selected, self.list_stamp_cam_fusion[self.cam_selected]))
                # print '[world] cam_selected: %s pos: %s' % (self.cam_selected, self.list_traj_cam_fusion[self.cam_selected])
                # print '[world] cam_selected: %s rot: %s' % (self.cam_selected, self.list_rot_cam_fusion[self.cam_selected])

    def toggle_show_traj(self):
        self.show_traj = not self.show_traj

    #This is called on updateGL() or Qt's need of redraw
    #http://stackoverflow.com/questions/17710514/when-is-qglwidgets-paintgl-called
    #http://qt-project.org/doc/qt-4.8/qglwidget.html
    #override
    def paintGL(self):
        glClearColor(.8, .8, .8, 1.0)  # default background color
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glPushMatrix()
        #glTranslated(0, 0, 1)
        glScale(self.scale, self.scale, self.scale)
        #print 'rot: %s %s %s' % (self.xRot, self.yRot, self.zRot)
        glRotated(self.xRot, 1.0, 0.0, 0.0)
        glRotated(self.yRot, 0.0, 1.0, 0.0)
        glRotated(self.zRot, 0.0, 0.0, 1.0)
        #glTranslated(0, 0, -1)

        self.draw_sth(self.axes, (0.0, 0.0, 0.0))
        #self.draw_sth(self.cam, (1.0, 1.0, 0.0))
        #self.draw_sth(self.cam, (-1.0, -1.0, 1.0))
        #self.draw_sth_multi(self.cam, [(-1,-1,1), (-1,-1,2)])
        if self.cam_fake > 0:
            self.draw_sth(self.cam_fake, self.cam_fake_pos)

            for pos in self.na_pos_cam_bundle:
                self.draw_sth(self.cam_fake, pos)

        if self.show_traj:
            self.draw_sth_multi(self.cam, self.list_traj_cam_svo, mode=GL_LINE)

            # render cam trajectory
            self.draw_sth_multi(self.cam, self.list_traj_cam_fusion,
                                list_rot=self.list_rot_cam_fusion,
                                mode=GL_LINE)
            #print 'cam_selected: %s' % self.cam_selected
            if self.list_traj_cam_fusion:
                index = -1 if self.cam_selected is None else self.cam_selected
                self.draw_sth(self.cam, self.list_traj_cam_fusion[index],
                              rot=self.list_rot_cam_fusion[index],
                              mode=GL_FILL)

            # Show shifted cam
            if self.tpl_pose_cam_virt is not None:
                # print self.tpl_pose_cam_virt
                self.draw_sth(self.cam,
                              self.tpl_pose_cam_virt[0],
                              self.tpl_pose_cam_virt[1], mode=GL_FILL)

        if self.load_world_posted is not None:
            self.load_world(self.load_world_posted['datasetdir'])
            self.load_world_posted = None

        if self.save_world_posted is not None:
            self.save_world(self.save_world_posted['savedir'])
            self.save_world_posted = None

        if self.clear_world_posted is not None:
            self.clear_world()
            self.clear_world_posted = None

        if self.build_world_posted is not None:
            self.build_world()
            self.build_world_posted = None

        if self.upload_vbo_index_obj_posted is not None:
            self.upload_vbo_index_obj(
                self.upload_vbo_index_obj_posted['index'],
                self.upload_vbo_index_obj_posted['npts_cloud'],
                self.upload_vbo_index_obj_posted['d_ave'],
                self.upload_vbo_index_obj_posted['d_std'])
            self.upload_vbo_index_obj_posted = None

        stat = self.get_mesh_info('stat')
        if stat == 'ready_obj':
            with self.lock:
                npa_vbo_index_obj = self.get_mesh_info('npa_vbo_index_obj')
                self.upload_indexed_vbo_indices(self.vbo_indices_obj, npa_vbo_index_obj)
                self.len_vbo_indices_obj = len(npa_vbo_index_obj)
                self.set_mesh_info('stat', 'uploaded')
                print '[world] vbo_index_obj uploaded'

        if stat == 'ready':
            with self.lock:
                npa_vbo_index = self.get_mesh_info('npa_vbo_index')
                npa_vbo_index_obj = self.get_mesh_info('npa_vbo_index_obj')

                # trap!! if change this order, program crashes!!!
                # e.g. upload data and then indices --> crash!!  why??
                self.upload_indexed_vbo_indices(
                    self.vbo_indices, npa_vbo_index)
                self.len_vbo_indices = len(npa_vbo_index)

                self.upload_indexed_vbo_indices(
                    self.vbo_indices_obj, npa_vbo_index_obj)
                self.len_vbo_indices_obj = len(npa_vbo_index_obj)

                self.upload_indexed_vbo_data(
                    self.vbo_data, self.get_mesh_info('npa_vbo_vertex'))

                self.set_mesh_info('stat', 'uploaded')
                if not self.is_first_mesh_uploaded:
                    self.is_first_mesh_uploaded = True
                print '[world] vbo uploaded'

        if self.is_first_mesh_uploaded:
            self.draw_indexed_vbo(self.vbo_indices,
                                  self.vbo_data,
                                  self.mode_polygon,
                                  self.len_vbo_indices)

        #self.drawGear(self.gear1, -3.0, -2.0, 0.0, self.gear1Rot / 16.0)
        #self.drawGear(self.gear2, +3.1, -2.0, 0.0, -2.0 * (self.gear1Rot / 16.0) - 9.0)

        if 0:
            glRotated(+90.0, 1.0, 0.0, 0.0)
            self.drawGear(self.gear3, -3.1, +1.8, -2.2, +2.0 * (self.gear1Rot / 16.0) - 2.0)
            #self.draw_sth(self.gear3, (-3.1, +1.8, -2.2), mode=GL_FILL)

        glPopMatrix()

    #http://qt-project.org/doc/qt-4.8/qglwidget.html
    #override
    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        #print width, height, side
        print '[world] resizeGL(): width: %s height: %s side: %s' % (width, height, side)

        #http://www.songho.ca/opengl/gl_transform.html
        if 1:
            # force VGA (make it a square to eliminate streching
            glViewport(0, 0, self.viewport_width, self.viewport_height)
            print '[world] resizeGL(): glViewport with: 0 0 %s %s' % (self.viewport_width, self.viewport_height)

            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()

            # songho: http://www.songho.ca/opengl/gl_transform.html
            # equivalents:
            gluPerspective(90.0, 1.0, 1.0, 100.0)
            #glFrustum(-1.0, +1.0, -1.0, 1.0, 1.0, 100.0)

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            glTranslated(0.0, 0.0, -6.0)

            # equivalent of pointing to z-direction with
            # up-vector=(0,-1,0)
            self.xRot = 0
            self.yRot = 180
            self.zRot = 180
        else:
            # orig:
            x = (width - side) / 2
            y = (height - side) / 2
            w = side
            h = side
            glViewport(x, y, w, h)
            print '[world] resizeGL(): glViewport with: %s %s %s %s' % (x, y, w, h)

            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            glFrustum(-1.0, +1.0, -1.0, 1.0, 5.0, 60.0)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            glTranslated(0.0, 0.0, -40.0)

    def set_scale(self, scale):
        if scale != self.scale:
            self.scale = scale
            self.scaleChanged.emit(scale)
            self.updateGL()

    # override
    def keyPressEvent(self, event):
        super(GLWidgetWorld, self).keyPressEvent(event)
        # deprecated: use worldKfNextAct/worldKfPrevAct instead
        # text = str(event.text())
        # if text == 'j':
        #     self.kf_next()
        # elif text == 'k':
        #     self.kf_prev()

    #override
    def wheelEvent(self, event):
        delta_wheel = event.delta()
        delta_scale = 0.1 if delta_wheel > 0 else -0.1
        scale_new = self.scale + delta_scale
        if scale_new < 0:
            scale_new = 0.1
        self.set_scale(scale_new)

    #override
    def mouseMoveEvent(self, event):
        dx = event.x() - self.last_pos_mouse.x()
        dy = event.y() - self.last_pos_mouse.y()
        sensitivity = 0.5

        if event.buttons() & QtCore.Qt.LeftButton:
            self.setXRotation(self.xRot + sensitivity * dy)
            self.setYRotation(self.yRot + sensitivity * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.setXRotation(self.xRot + sensitivity * dy)
            self.setZRotation(self.zRot + sensitivity * dx)

        self.last_pos_mouse = event.pos()

    def advanceGears(self):
        self.gear1Rot += 2 * 16
        self.updateGL()

    def xRotation(self):
        return self.xRot

    def yRotation(self):
        return self.yRot

    def zRotation(self):
        return self.zRot

    @staticmethod
    def get_ply_data():
        list_vertex = [
            (1.000000, 1.000000, -1.000000, 0.000000, 0.000000, -1.000000),
            (1.000000, -1.000000, -1.000000, 0.000000, 0.000000, -1.000000),
            (-1.000000, -1.000000, -1.000000, 0.000000, 0.000000, -1.000000),
            (-1.000000, 1.000000, -1.000000, 0.000000, 0.000000, -1.000000),
            (1.000000, 0.999999, 1.000000, 0.000000, -0.000000, 1.000000),
            (-1.000000, 1.000000, 1.000000, 0.000000, -0.000000, 1.000000),
            (-1.000000, -1.000000, 1.000000, 0.000000, -0.000000, 1.000000),
            (0.999999, -1.000001, 1.000000, 0.000000, -0.000000, 1.000000),
            (1.000000, 1.000000, -1.000000, 1.000000, -0.000000, 0.000000),
            (1.000000, 0.999999, 1.000000, 1.000000, -0.000000, 0.000000),
            (0.999999, -1.000001, 1.000000, 1.000000, -0.000000, 0.000000),
            (1.000000, -1.000000, -1.000000, 1.000000, -0.000000, 0.000000),
            (1.000000, -1.000000, -1.000000, -0.000000, -1.000000, -0.000000),
            (0.999999, -1.000001, 1.000000, -0.000000, -1.000000, -0.000000),
            (-1.000000, -1.000000, 1.000000, -0.000000, -1.000000, -0.000000),
            (-1.000000, -1.000000, -1.000000, -0.000000, -1.000000, -0.000000),
            (-1.000000, -1.000000, -1.000000, -1.000000, 0.000000, -0.000000),
            (-1.000000, -1.000000, 1.000000, -1.000000, 0.000000, -0.000000),
            (-1.000000, 1.000000, 1.000000, -1.000000, 0.000000, -0.000000),
            (-1.000000, 1.000000, -1.000000, -1.000000, 0.000000, -0.000000),
            (1.000000, 0.999999, 1.000000, 0.000000, 1.000000, 0.000000),
            (1.000000, 1.000000, -1.000000, 0.000000, 1.000000, 0.000000),
            (-1.000000, 1.000000, -1.000000, 0.000000, 1.000000, 0.000000),
            (-1.000000, 1.000000, 1.000000, 0.000000, 1.000000, 0.000000)
        ]
        list_face = [
            (4, 0, 1, 2, 3),
            (4, 4, 5, 6, 7),
            (4, 8, 9, 10, 11),
            (4, 12, 13, 14, 15),
            (4, 16, 17, 18, 19),
            (4, 20, 21, 22, 23)
        ]
        return {'vertex': list_vertex, 'face': list_face}

    @staticmethod
    def get_ply_data_from_file(filename, scale=1.0):
        list_vertex = []
        list_face = []
        with open(filename, 'r') as f:
            num_vertex = -1
            num_face = -1
            for line in f:
                if line.startswith('element vertex '):
                    tokens = line.split()
                    num_vertex = int(tokens[2])
                elif line.startswith('element face '):
                    tokens = line.split()
                    num_face = int(tokens[2])
                elif line.startswith('end_header'):
                    break
            print 'PLY header parsed -- num_vertex: %d num_face: %d' % (num_vertex, num_face)

            #!!!! TURN OFF ALL print's IN PRODUCTION, or slow !!!!
            for index, line in enumerate(f):
                #print 'index: %d vertex line: %s' % (index, line)
                tokens = line.split()
                list_vertex.append((float(tokens[0])*scale, float(tokens[1])*scale, float(tokens[2])*scale))
                if index == num_vertex-1:
                    break

            for index, line in enumerate(f):
                #print 'index: %d face line: %s' % (index, line)  # e.g. index: 0 face line: 4 0 1 2 3
                list_id_vertex = line.split()
                #print list_id_vertex  # e.g. ['4', '0', '1', '2', '3']
                ll = tuple([int(x) for x in list_id_vertex])
                #print ll  # e.g. (4, 0, 1, 2, 3)
                list_face.append(ll)
                if index == num_face-1:
                    break
        #print list_face
        return {'vertex': list_vertex, 'face': list_face}

    def makeGear(self, reflectance, innerRadius, outerRadius, thickness, toothSize, toothCount):
        list_gear = glGenLists(1)
        glNewList(list_gear, GL_COMPILE)
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance)

        r0 = innerRadius
        r1 = outerRadius - toothSize / 2.0
        r2 = outerRadius + toothSize / 2.0
        delta = (2.0 * math.pi / toothCount) / 4.0
        z = thickness / 2.0

        glShadeModel(GL_FLAT)

        for i in range(2):
            if i == 0:
                sign = +1.0
            else:
                sign = -1.0

            glNormal3d(0.0, 0.0, sign)

            glBegin(GL_QUAD_STRIP)

            for j in range(toothCount+1):
                angle = 2.0 * math.pi * j / toothCount
                glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), sign * z)
                glVertex3d(r1 * math.cos(angle), r1 * math.sin(angle), sign * z)
                glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), sign * z)
                glVertex3d(r1 * math.cos(angle + 3 * delta), r1 * math.sin(angle + 3 * delta), sign * z)

            glEnd()

            glBegin(GL_QUADS)

            for j in range(toothCount):
                angle = 2.0 * math.pi * j / toothCount
                glVertex3d(r1 * math.cos(angle), r1 * math.sin(angle), sign * z)
                glVertex3d(r2 * math.cos(angle + delta), r2 * math.sin(angle + delta), sign * z)
                glVertex3d(r2 * math.cos(angle + 2 * delta), r2 * math.sin(angle + 2 * delta), sign * z)
                glVertex3d(r1 * math.cos(angle + 3 * delta), r1 * math.sin(angle + 3 * delta), sign * z)

            glEnd()

        glBegin(GL_QUAD_STRIP)

        for i in range(toothCount):
            for j in range(2):
                angle = 2.0 * math.pi * (i + (j / 2.0)) / toothCount
                s1 = r1
                s2 = r2

                if j == 1:
                    s1, s2 = s2, s1

                glNormal3d(math.cos(angle), math.sin(angle), 0.0)
                glVertex3d(s1 * math.cos(angle), s1 * math.sin(angle), +z)
                glVertex3d(s1 * math.cos(angle), s1 * math.sin(angle), -z)

                glNormal3d(s2 * math.sin(angle + delta) - s1 * math.sin(angle), s1 * math.cos(angle) - s2 * math.cos(angle + delta), 0.0)
                glVertex3d(s2 * math.cos(angle + delta), s2 * math.sin(angle + delta), +z)
                glVertex3d(s2 * math.cos(angle + delta), s2 * math.sin(angle + delta), -z)

        glVertex3d(r1, 0.0, +z)
        glVertex3d(r1, 0.0, -z)
        glEnd()

        glShadeModel(GL_SMOOTH)

        glBegin(GL_QUAD_STRIP)

        for i in range(toothCount+1):
            angle = i * 2.0 * math.pi / toothCount
            glNormal3d(-math.cos(angle), -math.sin(angle), 0.0)
            glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), +z)
            glVertex3d(r0 * math.cos(angle), r0 * math.sin(angle), -z)

        glEnd()

        glEndList()

        return list_gear

    @staticmethod
    def drawGear(gear, dx, dy, dz, angle):
        glPushMatrix()
        glTranslated(dx, dy, dz)
        glRotated(angle, 0.0, 0.0, 1.0)
        glCallList(gear)
        glPopMatrix()

    @staticmethod
    def normalizeAngle(angle):
        while angle < 0:
            angle += 360 * 16

        while angle > 360 * 16:
            angle -= 360 * 16

    def save_world(self, savedir):
        print 'saving vbo_index.npy'
        np.save(savedir+'/npa_vbo_index', self.get_mesh_info('npa_vbo_index'))

        print 'saving vbo_vertex.npy'
        np.save(savedir+'/npa_vbo_vertex', self.get_mesh_info('npa_vbo_vertex'))

        print 'saving traj/rot/stamp'
        np.save(savedir+'/list_traj_cam_fusion', self.list_traj_cam_fusion)
        np.save(savedir+'/list_rot_cam_fusion', self.list_rot_cam_fusion)
        np.save(savedir+'/list_stamp_cam_fusion', self.list_stamp_cam_fusion)
        print 'done.'

    def post_load_world(self, datasetdir):
        self.load_world_posted = {'datasetdir': datasetdir}

    def post_save_world(self, savedir):
        self.save_world_posted = {'savedir': savedir}

    def post_clear_world(self):
        self.clear_world_posted = {}

    def post_build_world(self):
        self.build_world_posted = {}

    def post_upload_vbo_index_obj(
            self, npa_vbo_index_obj,
            sum_npts_cloud_obj, d_ave, d_std):
        self.upload_vbo_index_obj_posted = {
            'index': npa_vbo_index_obj,
            'npts_cloud': sum_npts_cloud_obj,
            'd_ave': d_ave,
            'd_std': d_std}

    def upload_vbo_index_obj(self, npa_vbo_index_obj_in, npts_cloud_obj,
                             d_ave, d_std):
        self.set_mesh_info('stat', 'invalidated')
        self.set_mesh_info('npa_vbo_index_obj', npa_vbo_index_obj_in)
        self.set_mesh_info('npts_cloud_obj', npts_cloud_obj)
        self.set_mesh_info('d_ave_obj', d_ave)
        self.set_mesh_info('d_std_obj', d_std)
        print 'npa_vbo_index_obj_in: '
        print npa_vbo_index_obj_in
        if 0:  # test
            npa_vbo_index = self.get_mesh_info('npa_vbo_index')
            print npa_vbo_index
            tri1 = npa_vbo_index[6:9]
            tri2 = npa_vbo_index[9:12]
            print tri1, tri2
            # http://docs.scipy.org/doc/numpy/reference/generated/numpy.concatenate.html
            npa_vbo_index_part = np.concatenate((tri1, tri2))
            print npa_vbo_index_part
            print 'len of npa_vbo_index_part: %d' % len(npa_vbo_index_part)
            self.set_mesh_info('npa_vbo_index_obj', npa_vbo_index_part)
        self.set_mesh_info('stat', 'ready_obj')

    def load_world(self, datasetdir):
        self.clear_world()

        self.set_mesh_info('stat', 'invalidated')  # --------

        print 'loading vbo_index.npy'
        npa_vbo_index = np.load(datasetdir+'/npa_vbo_index.npy')
        str_c_triangles = util_format_k(npa_vbo_index.shape[0]/3)
        print '# of triangles: %s' % str_c_triangles
        self.set_mesh_info('npa_vbo_index', npa_vbo_index)

        self.set_mesh_info('npa_vbo_index_obj', np.array([0, 0, 0]))

        print 'loading vbo_vertex.npy'
        npa_vbo_vertex = np.load(datasetdir+'/npa_vbo_vertex.npy')
        print '# of vertices: %s' % util_format_k(npa_vbo_vertex.shape[0]/6)
        self.set_mesh_info('npa_vbo_vertex', npa_vbo_vertex)

        self.set_mesh_info('stat', 'ready')  # --------

        print 'loading traj/rot/stamp'
        self.list_traj_cam_fusion = [pos for pos in np.load(datasetdir+'/list_traj_cam_fusion.npy')]
        self.list_rot_cam_fusion = [rot for rot in np.load(datasetdir+'/list_rot_cam_fusion.npy')]
        self.list_stamp_cam_fusion = np.load(datasetdir+'/list_stamp_cam_fusion.npy').tolist()
        if not self.list_traj_cam_fusion:
            # The trajectory is empty
            self.cam_selected = None
        else:
            self.cam_selected = 0  # the first pose in traj
            self.kf_cmd('start')

        self.datasetdir_edit.setText('[model: %s triangles] %s' % (str_c_triangles, datasetdir))
        print 'done.'

        if 0:
            # ----- this block was refactored into SnapRaw
            path_dataset = '../data-fastfusion-tum/rgbd_dataset-cubi10-slowly-OK/'
            path_associate = path_dataset + 'associate.txt'
            npa_stamp_depth_rgb = np.loadtxt(path_associate, usecols=(0, 9, 11), dtype='S')

            stamp = '3.749972'
            idx = self.list_stamp_cam_fusion.index(stamp)
            path_depth = path_dataset + npa_stamp_depth_rgb[idx][1]
            path_rgb = path_dataset + npa_stamp_depth_rgb[idx][2]
            depth = cv2.imread(path_depth, cv2.IMREAD_UNCHANGED)
            rgb = cv2.imread(path_rgb, cv2.IMREAD_COLOR)  # no alpha

            print (
                self.list_rot_cam_fusion[idx],
                self.list_traj_cam_fusion[idx],
                np.array([525., 525., 319.5, 239.5]),
                path_depth, path_rgb, stamp)
            sensor_data_raw = self.ff.get_sensor_data(
                self.list_rot_cam_fusion[idx],
                self.list_traj_cam_fusion[idx],
                np.array([525., 525., 319.5, 239.5]),
                depth, rgb, stamp, dumpfile=True)
            print sensor_data_raw

    def clear_world(self):
        print 'clearing vbo_index/vbo_vertex'
        self.set_mesh_info('stat', 'invalidated')
        self.set_mesh_info('npa_vbo_index', np.array([0, 0, 0]))
        self.set_mesh_info('npa_vbo_index_obj', np.array([0, 0, 0]))
        self.set_mesh_info('npa_vbo_vertex', np.array([0, 0, 0, 0, 0, 0]))
        self.set_mesh_info('stat', 'ready')

        print 'clearing traj/rot/stamp'
        del self.list_traj_cam_fusion[:]
        del self.list_rot_cam_fusion[:]
        del self.list_stamp_cam_fusion[:]
        self.cam_selected = None
        self.log_set_text('')
        print 'done.'

    def build_world(self):
        has_assoc, path_assoc = self.get_datasetdir_assoc()[0:2]
        if not has_assoc:
            print 'build_world(): error: no assoc found, bye'
            self.widget_ctrl.set_build_iface(True)
            return

        self.clear_world()

        self.ff_thread = custom_threading.CustomThread(
            target=self.ff_task, args=(self.ff, path_assoc,),
            on_finished=self.on_fusion_finished)
        self.ff_thread.start()

        # self.ff_thread = threading.Thread(
        #     target=self.ff_task, args=(self.ff, path_assoc,))
        # self.ff_thread.daemon = True
        # self.ff_thread.start()

    def on_fusion_finished(self):
        print 'on_fusion_finished(): hello'
        self.widget_ctrl.set_build_iface(True)

    def ff_task(self, ff, path_associate):
        print 'call fastfusion here...'
        # return

        ff.configure({
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset/associate.txt',
            # ----------------------
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/associate_gt_skip.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/associate_gt.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset_freiburg3_long_office_household/associate_dvo.txt',
            # 'scale_ply': 5.0,
            # ----------------------
            # d{1,2,3}
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-d2/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e1/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e2/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e3/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e4/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e5/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e6/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e6p/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-e7p/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-ruler-2xA4vert/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-ruler-50cm/associate.txt',
            'filename_associate': path_associate,
            # ----------------------
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-f0/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-f1/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-f2/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-f3/associate.txt',
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-f4/associate.txt',
            'scale_ply': 10.0,
            # ----------------------
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset--homkrun/associate.txt',
            # 'scale_ply': 5.0,
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset--monitor200/associate.txt',
            # 'scale_ply': 5.0,
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-c2-600/associate.txt',
            # 'scale_ply': 5.0,
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-c3-600/associate.txt',
            # 'scale_ply': 5.0,
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset-c4-600/associate.txt',
            # 'scale_ply': 5.0,
            # 'filename_associate': '../data-fastfusion-tum/rgbd_dataset--ambassador/associate.txt',
            # 'scale_ply': 1.0,
        })
        ff.init_posinfo_disk()

        if 0:
            ff.test_main()
            return

        count = 0
        mod_cam = 4
        mod_ply = 16
        c_frames = ff.count_frames()
        while count < c_frames:
        # while count < 4+1:
        # while count < 1:
        # while count < 40+1:  # kf_10
        # while count < 80+1:
        # while count < 480+1:
            if count % mod_cam == 0:
                cam_trans = ff.get_trans_current()
                cam_rot = ff.get_rot_current()
                cam_stamp = ff.get_str_timestamp_current()
                #print cam_trans
                #print cam_rot
                #print ff.get_intrinsic_current()  # fx, fy, cx, cy
                print '[fusion] kf: %s stamp: %s' % (count, cam_stamp)
                self.list_traj_cam_fusion.append(cam_trans)
                self.list_rot_cam_fusion.append(cam_rot)
                self.list_stamp_cam_fusion.append(cam_stamp)

            ff.fusion_frame()

            print 'py +++ count: %d/%d' % (count, c_frames)
            if count % mod_ply == 0:
                self.set_mesh_info('stat', 'invalidated')
                self.set_mesh_info('npa_vbo_index', ff.get_vbo_index())
                self.set_mesh_info('npa_vbo_index_obj', np.array([0, 0, 0]))
                self.set_mesh_info('npa_vbo_vertex', ff.get_vbo_vertex())
                self.set_mesh_info('stat', 'ready')

            if count % mod_cam == 0:
                ff.append_posinfo_disk()
                if 0:  # dump raw csv's
                    stamp = ff.get_str_timestamp_current()
                    depth = cv2.imread(ff.get_path_depth_current(), cv2.IMREAD_UNCHANGED)

                    depth_masked = depth
                    if self.mask_raw_enabled:
                        depth_masked = GLWidgetAbstract.get_depth_mask(
                            depth, self.mask_dir, stamp)
                    cv2.imshow('depth_masked_raw', depth_masked)  # 0 = invalid

                    rgb = cv2.imread(ff.get_path_rgb_current(), cv2.IMREAD_COLOR)  # no alpha
                    # print (
                    #     ff.get_rot_current(),
                    #     ff.get_trans_current(scale=False),
                    #     ff.get_intrinsic_current(),
                    #     ff.get_path_depth_current(),
                    #     ff.get_path_rgb_current(),
                    #     stamp)
                    sensor_data_raw = ff.get_sensor_data(
                        ff.get_rot_current(),
                        ff.get_trans_current(scale=False),
                        ff.get_intrinsic_current(),
                        depth_masked, rgb, stamp, dumpfile=True)
                    print '# of valid sensor records (raw): %s' % (len(sensor_data_raw)/8)
                    print sensor_data_raw

            ff.increment_frame()
            count += 1
            #return  #debug!!!!!

        # ff.write_ply_data_current(count-1)
        ff.delete()

    def svo_thread(self):
        # spawn a poscam thread
        th = threading.Thread(target=self.poscam_handler, args=(self.list_traj_cam_svo, self.stream_poscam))
        th.daemon = True
        th.start()

        # spawn a mjpeg thread
        th = threading.Thread(target=self.mjpeg_handler, args=(False, 0.6,))
        th.daemon = True
        th.start()

        Svo().run()

    @staticmethod
    def poscam_handler(list_traj_cam, stream_poscam):
        while 1:
            print 'poscam_handler: reading stream_poscam... (may block)'
            data = next(stream_poscam)
            #print data
            kf = data[3]
            id = data[4]
            if id % 10 == 0:
                xyz_scaled = tuple(f*10 for f in data[:3])
                #print xyz_scaled
                list_traj_cam.append(xyz_scaled)

    #http://stackoverflow.com/questions/21702477/how-to-parse-mjpeg-http-stream-from-ip-camera
    #https://github.com/ethanrublee/streamer
    @staticmethod
    def mjpeg_handler(is_dumping=False, sec_wait_urlopen=0.6):
        url = 'http://localhost:9090/stream_0'
        #url = 'http://192.168.7.2:9090/stream_0'

        print '>>> mjpeg_client: waiting %s sec for %s' % (sec_wait_urlopen, url)
        time.sleep(sec_wait_urlopen)  # wait for the stream opened

        # streamer-master/build/bin/httpserv 0.0.0.0 9090 8 .
        stream = urllib.urlopen(url)
        bytes = ''

        if is_dumping:
            util_create_empty_dir('mjpeg_save_renameme')

        index = 0
        while True:
            bytes += stream.read(1024)
            a = bytes.find('\xff\xd8')
            b = bytes.find('\xff\xd9')
            #### fixme:
            #### commented cv2 related since ff using cv-with-stdc++
            # if a != -1 and b != -1:
            #     jpg = bytes[a:b+2]
            #     bytes = bytes[b+2:]
            #     img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
            #     title = '%s %s %s' % (url, type(img), img.shape)
            #     cv2.imshow(title, img)
            #
            #     if is_dumping:
            #         # this block make the thread slow!!
            #         filename = 'feats_%s.png' % str(index).zfill(8)
            #         cv2.imwrite(dirname+'/'+filename, img)
            #         index += 1

    @staticmethod
    def fifo_poscam():
        filename = 'fifo-poscam'
        with open(filename, 'r') as f:
            # NG: don't use in, or suspicious buffering happens...
            #for line in f.readline():
            #for line in f:
            while 1:
                line = f.readline()  # may block
                #print line
                if not line.startswith('@@@ posCam'):
                    #print 'not process: '+line
                    continue

                list_tokens = line.split()
                x = float(list_tokens[2])
                y = float(list_tokens[3])
                z = float(list_tokens[4])
                kf = int(list_tokens[5])
                id = int(list_tokens[6])
                #print '%s line --> %f %f %f %d %d' % (filename, x, y, z, kf, id)
                yield (x, y, z, kf, id)

