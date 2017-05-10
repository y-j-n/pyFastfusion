from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
# following pyqtgraph-develop/pyqtgraph/opengl/GLViewWidget.py
import OpenGL.GL.framebufferobjects as glfbo

from PySide import QtCore, QtGui, QtOpenGL
import itertools
from ctypes import *

from shader_if import *
from util import *


class GLWidgetAbstract(QtOpenGL.QGLWidget):
    scaleChanged = QtCore.Signal(int)

    def __init__(self, glformat, parent=None, sharewidget=None):
        super(GLWidgetAbstract, self).__init__(glformat, parent, shareWidget=sharewidget)
        self.widget_ctrl = None

        # http://nullege.com/codes/show/src@v@i@visvis-1.9@backends@qtcommon.py/106/PyQt4.QtOpenGL.QGLWidget.setFocusPolicy
        # http://www.qtcentre.org/threads/22154-QGLWidget-keyPressEvent-does-not-work
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()

        self.log_widget = QtGui.QLineEdit()
        self.log_widget.setReadOnly(True)

        self.mode_polygon = GL_LINE

        self.viewport_width = 640
        self.viewport_height = 480
        ##self.setMinimumSize(640, 480)

        # self.mask_dir = '../data-fastfusion-tum/rgbd_dataset-d--output/visvoc--nov12/visvoc@p10_p10dsift3ikm100_m_3_100_svm_gridsearch/results'
        self.mask_dir = '../data-fastfusion-tum/rgbd_dataset-e6p/visvoc@p10_p10dsift3ikm100_m_3_100_svm_gridsearch/results'
        # self.mask_dir = 'worlds/world-e6p-full400/masks'

        # test showing freiburg3-8x-sift-done--/bundle/bundle.out
        self.na_pos_cam_bundle = np.array([
            (-1.0769498435e-01, 6.7903144755e-02, -2.5671110566e-01),
            (-9.2239343244e-02, 6.9616271743e-02, -2.6310879939e-01),
            (-7.6916980769e-02, 6.9852581173e-02, -2.6778042734e-01),
            (-4.2908073946e-02, 6.9340870788e-02, -2.8470687183e-01),
            (1.1067816758e-02, 9.2373005106e-02, -3.1673983950e-01),
            (7.1502784029e-02, 1.0129081058e-01, -3.3117094686e-01),
            (9.5139397520e-02, 7.7316883258e-02, -3.1544899109e-01),
            (1.1410728541e-01, 8.1530440231e-02, -3.1140744210e-01),
            (1.9403549513e-01, 7.6644713579e-02, -3.2831166741e-01),
            (3.0071999060e-01, 9.2322297390e-02, -3.3128853413e-01),
            (3.9307520530e-01, 9.5140267978e-02, -3.2997734924e-01),
            (4.9812563675e-01, 8.3766508856e-02, -3.4761894688e-01),
            (6.3555826962e-01, 1.1228790565e-01, -4.5124533828e-01),
            (7.6532383571e-01, 1.5397904515e-01, -4.7810340861e-01),
            (8.9022640857e-01, 1.8390312444e-01, -4.0769654027e-01),
            (1.0011655351e+00, 1.6040032433e-01, -3.1879648504e-01),
            (1.0851921668e+00, 1.5488524301e-01, -2.5787636236e-01),
            (1.1524663046e+00, 1.0489630717e-01, -2.2385239012e-01),
            (1.2198125123e+00, 6.5081860361e-02, -1.7386418971e-01),
        ])
        # self.na_pos_cam_bundle -= (-1.0769498435e-01, 6.7903144755e-02, -2.5671110566e-01)
        self.na_pos_cam_bundle *= 10.0
        self.na_pos_cam_bundle = []  # !!!!!!!!!!!! turn it off

        self.last_pos_mouse = None

    def register_ctrl(self, ctrl):
        self.widget_ctrl = ctrl

    #http://qt-project.org/doc/qt-4.8/qglwidget.html
    #Sets up the OpenGL rendering context, defines display lists, etc.
    #Gets called once before the first time resizeGL() or paintGL() is called.
    #override
    def initializeGL(self):
        # init shader here is ok
        print '[abst] init shader'
        shader_program = compileProgram([VERTEX_SHADER, ], [FRAGMENT_SHADER,])
        if shader_program:
            glUseProgramObjectARB(shader_program)

        glEnableVertexAttribArray(0)  # in_position
        glEnableVertexAttribArray(1)  # in_color

        glEnable(GL_NORMALIZE)
        glPolygonMode(GL_FRONT_AND_BACK, self.mode_polygon)  # default polygon mode

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)

        # ref: songho/fboDepth/main.cpp
        # http://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20040914
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4)  # 4-byte pixel alignment
        glEnable(GL_TEXTURE_2D)

        glClearStencil(0)  # clear stencil buffer
        glClearDepth(1.0)  # 0 is near, 1 is far
        glDepthFunc(GL_LEQUAL)

    #override
    def mousePressEvent(self, event):
        self.last_pos_mouse = event.pos()

    #override
    def keyPressEvent(self, event):
        text = str(event.text())
        #print 'key press: '+text
        # deprecated this is done through qt menu
        # if text == 'p':
        #     self.toggle_polygon_mode()

    def get_log_widget(self):
        return self.log_widget

    def log_set_text(self, text):
        self.log_widget.setText(text)

    def toggle_polygon_mode(self):
        if self.mode_polygon == GL_FILL:
            self.mode_polygon = GL_LINE
        elif self.mode_polygon == GL_LINE:
            self.mode_polygon = GL_FILL
        else:
            print 'current polygon mode is invalid!!'

    def grab_rgb_rendered(self):
        return util_qimage_to_ndarray(self.grabFrameBuffer())

    # http://stackoverflow.com/questions/406121/flattening-a-shallow-list-in-python
    @staticmethod
    def flatten3(*lll):
        return [e for ll in lll for l in ll for e in l]

    @staticmethod
    def upload_indexed_vbo_indices(vbo_indices, indices):
        # Not using glBufferSubData() since we are updating entire
        # vertices data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_indices)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     (c_int*len(indices))(*indices), GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

    @staticmethod
    def upload_indexed_vbo_data(vbo_data, data):
        glBindBuffer(GL_ARRAY_BUFFER, vbo_data)
        glBufferData(GL_ARRAY_BUFFER,
                     (c_float*len(data))(*data), GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    @staticmethod
    def upload_indexed_vbo(vbo_indices, vbo_data, indices, data):
        # trap!! if swap this order, program crashes!!!
        GLWidgetAbstract.upload_indexed_vbo_indices(vbo_indices, indices)
        GLWidgetAbstract.upload_indexed_vbo_data(vbo_data, data)

    @staticmethod
    def draw_indexed_vbo(vbo_index, vbo_data, mode, c_records):
        if c_records < 0:
            print 'draw_indexed_vbo(): invalid c_records'
            return

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_index)
        glBindBuffer(GL_ARRAY_BUFFER, vbo_data)

        float_size = sizeof(c_float)
        record_len = 6 * float_size  # 3 for in_location, 3 for in_coloar

        glVertexAttribPointer(0, 3, GL_FLOAT, False,
                              record_len, c_void_p(0*float_size))
        glVertexAttribPointer(1, 3, GL_FLOAT, False,
                              record_len, c_void_p(3*float_size))

        glPushAttrib(GL_POLYGON_BIT)
        glPolygonMode(GL_FRONT_AND_BACK, mode)
        glDrawElements(GL_TRIANGLES, c_records, GL_UNSIGNED_INT, c_void_p(0))
        glPopAttrib()

        # switch back to normal pointer operation
        #http://www.songho.ca/opengl/gl_vbo.html
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    @staticmethod
    def calc_dists_signed(vbo_index, vbo_vertex, li_abcd):
        d = li_abcd[3]
        abc = li_abcd[0:3]
        norm_abc = LA.norm(abc)
        # print li_abcd, d, abc, norm_abc

        # print npa_vbo_vertex.shape
        # # http://docs.scipy.org/doc/numpy/reference/generated/numpy.reshape.html
        vert_col6 = np.reshape(vbo_vertex, (-1, 6))
        # print vert_col6[0:10]

        # debug: explicit but slow
        # for idx in vbo_index[0:6]:
        #     xyz = vert_col6[idx][0:3]
        #     # xyz = [0, 0, 0]
        #     print xyz
        #     dist_signed = (np.dot(abc, xyz) + d) / norm_abc
        #     print dist_signed
        # -------
        # out_slow = np.array([(np.dot(abc, vert_col6[idx][0:3]) + d) / norm_abc for idx in vbo_index])
        # print out_slow.shape

        out_fast = (np.dot(abc, vert_col6[vbo_index, 0:3].T) + d) / norm_abc
        # print out_fast.shape
        return out_fast

    @staticmethod
    def get_vbo_index_plane(npa_vbo_index, npa_vbo_vertex, li_abcd, dist_thresh):
        with util_measure_time('calc_dists_signed....'):
            npa_d = np.fabs(GLWidgetAbstract.calc_dists_signed(
                npa_vbo_index, npa_vbo_vertex, li_abcd))

        # with util_measure_time('build vbo_index_plane....slow'):
        #     # http://stackoverflow.com/questions/2990121/how-do-i-loop-through-a-python-list-by-twos
        #     li_vbo_index_plane = []
        #     for i in xrange(0, npa_d.size, 3):
        #         if npa_d[i] > dist_thresh:
        #             continue
        #         if npa_d[i+1] > dist_thresh:
        #             continue
        #         if npa_d[i+2] > dist_thresh:
        #             continue
        #         # this triangle is near enough to the plane, so keep it
        #         li_vbo_index_plane.append(npa_vbo_index[i])
        #         li_vbo_index_plane.append(npa_vbo_index[i+1])
        #         li_vbo_index_plane.append(npa_vbo_index[i+2])
        #     out_slow = np.array(li_vbo_index_plane)
        #     print out_slow.shape

        with util_measure_time('build vbo_index_plane....fast'):
            # approximating......
            # pick up triangles whose averaged distance of its three vertices are less than dist_thresh
            npa_d_col3 = np.reshape(npa_d, (-1, 3))
            npa_d_ave = np.average(npa_d_col3, axis=1)
            npa_vbo_index_col3 = np.reshape(npa_vbo_index, (-1, 3))

            # http://stackoverflow.com/questions/3030480/numpy-array-how-to-select-indices-satisfying-multiple-conditions
            out_fast = npa_vbo_index_col3[npa_d_ave < dist_thresh].reshape(-1)
            # print out_fast.shape

            d_ave = np.average(npa_d_ave[npa_d_ave < dist_thresh])
            d_std = np.std(npa_d_ave[npa_d_ave < dist_thresh])
            d_ave_unit_cm = d_ave / 20 * 100
            d_std_unit_cm = d_std / 20 * 100
            print 'dist_thresh: %g' % dist_thresh
            print 'inliers dist ave: %g' % d_ave
            print 'inliers dist std: %g' % d_std
            print 'inliers dist ave_cm: %g' % d_ave_unit_cm
            print 'inliers dist std_cm: %g' % d_std_unit_cm

        # return out_fast, d_ave, d_std
        return out_fast, d_ave_unit_cm, d_std_unit_cm

    @staticmethod
    def make_axes(reflectance):
        list_gl = glGenLists(1)
        glNewList(list_gl, GL_COMPILE)
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance)
        glShadeModel(GL_FLAT)
        glNormal3d(0.0, 0.0, 1.0)

        glBegin(GL_LINES)
        glVertex3d(0.0, 0.0, 0.0)
        glVertex3d(4.0, 0.0, 0.0)
        glVertex3d(0.0, 0.0, 0.0)
        glVertex3d(0.0, 8.0, 0.0)
        glVertex3d(0.0, 0.0, 0.0)
        glVertex3d(0.0, 0.0, 12.0)
        glEnd()

        glEndList()
        return list_gl

    @staticmethod
    def make_cam(reflectance):
        list_gl = glGenLists(1)
        glNewList(list_gl, GL_COMPILE)
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance)
        glShadeModel(GL_FLAT)
        glNormal3d(0.0, 0.0, 1.0)

        # kludge: show front and back...
        glBegin(GL_QUADS)
        glVertex3d(-0.5, -0.5, 0.0)
        glVertex3d(0.5, -0.5, 0.0)
        glVertex3d(0.5, 0.5, 0.0)
        glVertex3d(-0.5, 0.5, 0.0)
        #----
        glVertex3d(-0.5, 0.5, 0.0)
        glVertex3d(0.5, 0.5, 0.0)
        glVertex3d(0.5, -0.5, 0.0)
        glVertex3d(-0.5, -0.5, 0.0)
        glEnd()

        glBegin(GL_LINES)
        glVertex3d(0.0, 0.0, 0.0)
        glVertex3d(0.0, 0.0, 1.0)
        glEnd()

        glEndList()
        return list_gl

    @staticmethod
    def draw_sth_multi(sth, list_pos, list_rot=[], mode=GL_FILL):
        for pos, rot in itertools.izip_longest(list_pos, list_rot):
            # http://stackoverflow.com/questions/1663807/how-can-i-iterate-through-two-lists-in-parallel-in-python
            GLWidgetAbstract.draw_sth(sth, pos, rot=rot, mode=mode)

    @staticmethod
    def draw_sth(sth, pos, rot=None, mode=GL_FILL):
        glPushMatrix()
        glTranslated(pos[0], pos[1], pos[2])

        if rot is not None:
            #GLWidgetAbstract.test_rot_equivalence()
            rotmat_4x4 = np.array([
                [rot[0], rot[1], rot[2], 0.],
                [rot[3], rot[4], rot[5], 0.],
                [rot[6], rot[7], rot[8], 0.],
                [0.,     0.,     0.,     1.]])
            rotmat_4x4 = np.transpose(rotmat_4x4)  # OpenGL's format
            glMultMatrixf(rotmat_4x4)

        glPushAttrib(GL_POLYGON_BIT)
        glPolygonMode(GL_FRONT_AND_BACK, mode)
        glCallList(sth)
        glPopAttrib()

        glPopMatrix()

    @staticmethod
    def test_rot_equivalence():
        # equivalence check... ok
        deg = 30.
        if 1:
            # http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
            sin = np.sin(deg / 180. * np.pi)
            cos = np.cos(deg / 180. * np.pi)
            rot_z = np.array([[cos,  -sin,  0.,   0.],
                              [sin,   cos,  0.,   0.],
                              [0.,     0.,  1.,   0.],
                              [0.,     0.,  0.,   1.]])
            rotmat = rot_z
            rotmat = np.transpose(rotmat)  # OpenGL's format
            glMultMatrixf(rotmat)
        else:
            glRotated(deg, 0.0, 0.0, 1.0)
        GLWidgetAbstract.dump_mat_mv()

    @staticmethod
    def dump_mat_mv():
        print 'modelview mat:'
        GLWidgetAbstract.__dump_mat(GL_MODELVIEW_MATRIX)
        print '--------'

    @staticmethod
    def dump_mat_proj():
        print 'proj mat:'
        GLWidgetAbstract.__dump_mat(GL_PROJECTION_MATRIX)
        print '--------'

    @staticmethod
    def __dump_mat(type):
        a = (GLfloat * 16)()
        mat = glGetFloatv(type, a)
        list_a = list(a)
        print list_a[0:4]
        print list_a[4:8]
        print list_a[8:12]
        print list_a[12:16]

