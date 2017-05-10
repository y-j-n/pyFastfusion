#!/usr/bin/env python

# c.f. c++ version that uses a native thread
# https://www.thasler.org/blog/?p=10

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *
    # following pyqtgraph-develop/pyqtgraph/opengl/GLViewWidget.py
    import OpenGL.GL.framebufferobjects as glfbo
except ImportError:
    glfbo = None
    app = QtGui.QApplication(sys.argv)
    QtGui.QMessageBox.critical(None, 'modeler', 'PyOpenGL must be installed to run this program.')
    sys.exit(1)

from glwidget_world import GLWidgetWorld
from world_ctrl import WorldCtrl
from cam_ctrl import CamCtrl
from cam_snap import SnapCam
from cam_obj import ObjCam
from util import *
import cv2


class AbstractWindow(QtGui.QMainWindow):
    def __init__(self):
        super(AbstractWindow, self).__init__()
        self.on_close_event = None

    def call_on_close_event(self):
        if self.on_close_event is not None:
            self.on_close_event()
        else:
            print 'on_close_event not set'

    def set_on_close_event(self, f):
        self.on_close_event = f

    # http://stackoverflow.com/questions/9249500/pyside-pyqt-detect-if-user-trying-to-close-window
    #override
    def closeEvent(self, event):
        self.call_on_close_event()
        print 'closeEvent done.'

    # abstract
    def close_all(self):
        print 'closing a window object...'


class ObjWindow(AbstractWindow):
    def __init__(self, worldwidget):
        super(ObjWindow, self).__init__()

        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)

        self.objCam = ObjCam(QtOpenGL.QGLFormat(), sharewidget=worldwidget)

        self.glWidgetArea = QtGui.QScrollArea()
        self.glWidgetArea.setWidget(self.objCam)
        self.glWidgetArea.setWidgetResizable(True)
        self.glWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.glWidgetArea.setMinimumSize(640+2, 480+2)  # ?? without +2, resizeGL() not report 640 480 why??

        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        grid.addWidget(self.glWidgetArea, 1, 0)
        grid.addWidget(self.objCam.get_log_widget(), 2, 0)
        centralWidget.setLayout(grid)

        self.setWindowTitle('Objects')

    # override
    def close_all(self):
        super(ObjWindow, self).close_all()
        print '[obj_win] before close()'
        self.close()
        print '[obj_win] after close()'


class MainWindow(AbstractWindow):
    def __init__(self, worldwidget):
        super(MainWindow, self).__init__()

        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)

        # fixme: not working under osx???
        # seems an old bug?
        # http://www.morethantechnical.com/2013/11/09/vertex-array-objects-with-shaders-on-opengl-2-1-glsl-1-2-wcode/
        # https://qt.gitorious.org/qt/qt5-maemo5-qtbase/commit/77fc6d30f1bd4bdd8894dd98e12373211241a091
        #
        # http://schi.iteye.com/blog/1969710
        # http://stackoverflow.com/questions/19277730/qt-opengl-os-x-glsl-shader-version-only-120-on-mountain-lion
        glformat = QtOpenGL.QGLFormat()
        # stick to glsl=120
        #glformat.setVersion(3, 3)
        #glformat.setProfile(QtOpenGL.QGLFormat.CoreProfile)
        #print glformat.profile()

        self.glWidgetWorld = worldwidget
        self.snapCam = SnapCam(glformat, sharewidget=self.glWidgetWorld)

        self.glWidgetArea = QtGui.QScrollArea()
        self.glWidgetArea.setWidget(self.glWidgetWorld)
        self.glWidgetArea.setWidgetResizable(True)
        self.glWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.glWidgetArea.setMinimumSize(640+2, 480+2)  # ?? without +2, resizeGL() not report 640 480 why??

        self.pixmapLabelArea = QtGui.QScrollArea()
        self.pixmapLabelArea.setWidget(self.snapCam)
        self.pixmapLabelArea.setWidgetResizable(True)
        self.pixmapLabelArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.pixmapLabelArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.pixmapLabelArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.pixmapLabelArea.setMinimumSize(640+2, 480+2)  # ?? without +2, resizeGL() not report 640 480 why??

        # self.pixmapLabelArea = QtGui.QScrollArea()
        # self.pixmapLabel = QtGui.QLabel()
        # self.pixmapLabelArea.setWidget(self.pixmapLabel)
        # self.pixmapLabelArea.setSizePolicy(QtGui.QSizePolicy.Ignored,
        #         QtGui.QSizePolicy.Ignored)
        # self.pixmapLabelArea.setMinimumSize(50, 50)

        # xSlider = self.createSlider(self.glWidgetWorld.xRotationChanged,
        #         self.glWidgetWorld.setXRotation)
        # ySlider = self.createSlider(self.glWidgetWorld.yRotationChanged,
        #         self.glWidgetWorld.setYRotation)
        # zSlider = self.createSlider(self.glWidgetWorld.zRotationChanged,
        #         self.glWidgetWorld.setZRotation)

        self.file_menu = None
        self.world_menu = None
        self.camera_menu = None
        self.help_menu = None
        self.create_menus()

        # centralLayout = QtGui.QGridLayout()
        # centralLayout.addWidget(self.glWidgetArea, 0, 0)
        # centralLayout.addWidget(self.pixmapLabelArea, 0, 1)
        # # centralLayout.addWidget(xSlider, 1, 0, 1, 2)
        # # centralLayout.addWidget(ySlider, 2, 0, 1, 2)
        # # centralLayout.addWidget(zSlider, 3, 0, 1, 2)
        # centralWidget.setLayout(centralLayout)

        # hbox = QtGui.QHBoxLayout()
        # hbox.addWidget(self.glWidgetArea)
        # hbox.addWidget(self.pixmapLabelArea)
        # centralWidget.setLayout(hbox)
        # ------- http://zetcode.com/gui/pyqt4/layoutmanagement/
        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        # ---- ----
        grid.addWidget(self.glWidgetArea, 1, 0)
        grid.addWidget(self.pixmapLabelArea, 1, 1)
        # ---- ----
        grid.addWidget(self.glWidgetWorld.get_log_widget(), 2, 0)
        grid.addWidget(self.snapCam.get_log_widget(), 2, 1)
        grid.addWidget(self.snapCam.get_log_widget_measure(), 3, 1)
        # ----
        world_ctrl = WorldCtrl(self.glWidgetWorld)
        self.glWidgetWorld.register_ctrl(world_ctrl)
        grid.addLayout(world_ctrl, 0, 0)
        self.world_ctrl = world_ctrl

        cam_ctrl = CamCtrl(self.snapCam)
        self.snapCam.register_ctrl(cam_ctrl)
        grid.addLayout(cam_ctrl, 0, 1)
        # ----
        centralWidget.setLayout(grid)

        # xSlider.setValue(15 * 16)
        # ySlider.setValue(345 * 16)
        # zSlider.setValue(0 * 16)

        self.setWindowTitle('Modeler')
        #self.resize(1024, 768)
        #self.resize(1024, 480)

    def renderIntoPixmap(self):
        # size = self.getSize()
        # if size.isValid():
        #     pixmap = self.glWidgetWorld.renderPixmap(size.width(), size.height())
        #     self.setPixmap(pixmap)

        qpixmap = self.glWidgetWorld.renderPixmap(640, 480)
        qimage = qpixmap.toImage()
        ndarray = util_qimage_to_ndarray(qimage)
        cv2.imshow('render', ndarray)

    def grabFrameBuffer(self):
        # image = self.glWidgetWorld.grabFrameBuffer()
        # print type(image)  # <type 'PySide.QtGui.QImage'>
        # self.setPixmap(QtGui.QPixmap.fromImage(image))

        #print self.snapCam.width(), self.snapCam.height()
        qimage = self.snapCam.grabFrameBuffer()
        print qimage.size()
        ndarray = util_qimage_to_ndarray(qimage)
        cv2.imshow('grab', ndarray)

    def clearPixmap(self):
        self.setPixmap(QtGui.QPixmap())

    def about(self):
        QtGui.QMessageBox.about(self, 'About Grabber', 'The <b>Grabber</b> example demonstrates two approaches for rendering OpenGL into a Qt pixmap.')

    # override
    def close_all(self):
        super(MainWindow, self).close_all()
        self.snapCam.killall_pcl_viewer()
        self.snapCam.snap_win.close()
        self.snapCam.planar_win.close()
        print '[main_win] before close()'
        self.close()
        print '[main_win] after close()'

    def set_menu(self, menu, tpl_args):
        title = tpl_args[0]
        shortcut = tpl_args[1]
        triggered = tpl_args[2]
        kwargs = tpl_args[3] if len(tpl_args) > 3 else {}
        if title == '__sep__':
            menu.addSeparator()
        else:
            menu.addAction(QtGui.QAction(
                title, self, shortcut=shortcut, triggered=triggered,
                checkable=kwargs.get('checkable', False)))

    def create_menus(self):
        self.file_menu = self.menuBar().addMenu('&File')
        for tpl_args in [
            ('&Render into Pixmap...', 'Ctrl+R', self.renderIntoPixmap),
            ('&Grab Frame Buffer', 'Ctrl+G', self.grabFrameBuffer),
            ('&Clear Pixmap', 'Ctrl+L', self.clearPixmap),
            ('__sep__', None, None),
            ('E&xit', 'Ctrl+Q', self.call_on_close_event),
            ('Exit', 'Esc', self.call_on_close_event),
        ]:
            self.set_menu(self.file_menu, tpl_args)

        # -------------
        # http://stackoverflow.com/questions/890128/why-python-lambdas-are-useful
        self.world_menu = self.menuBar().addMenu('&World')
        for tpl_args in [
            ('Wireframe', 'Shift+w', lambda: self.glWidgetWorld.toggle_polygon_mode(),
             {'checkable': True}),
            ('__sep__', None, None),
            ('KfStart', 'g', lambda: self.glWidgetWorld.kf_cmd('start')),
            ('KfPrev', 'k', lambda: self.glWidgetWorld.kf_cmd('prev')),
            ('KfNext', 'j', lambda: self.glWidgetWorld.kf_cmd('next')),
            ('KfEnd', 'Shift+g', lambda: self.glWidgetWorld.kf_cmd('end')),
            # ('ToggleMaskRaw', '', lambda: self.glWidgetWorld.toggle_raw_mask(),
            #  {'checkable': True}),
            ('__sep__', None, None),
            ('ruler50cm', 'r', lambda: self.world_ctrl.set_datasetdir('../worlds/world-ruler-50cm')),
            ('desk', 'd', lambda: self.world_ctrl.set_datasetdir('../worlds/world-freiburg-gt-count80')),
            ('plant', 'p', lambda: self.world_ctrl.set_datasetdir('../worlds/world-e6p-full400')),
        ]:
            self.set_menu(self.world_menu, tpl_args)

        # -------------
        self.camera_menu = self.menuBar().addMenu('&Camera')
        for tpl_args in [
            ('Wireframe', 'w', lambda: self.snapCam.toggle_polygon_mode(),
             {'checkable': True}),
            ('__sep__', None, None),
            ('Pitch+', 'Shift+Up', lambda: self.snapCam.rotate(-1.0, 0, 0)),
            ('Pitch-', 'Shift+Down', lambda: self.snapCam.rotate(1.0, 0, 0)),
            ('Yaw+', 'Shift+Right', lambda: self.snapCam.rotate(0, 1.0, 0)),
            ('Yaw-', 'Shift+Left', lambda: self.snapCam.rotate(0, -1.0, 0)),
            ('TransX+', 'Right', lambda: self.snapCam.translate(-1.0, 0, 0)),
            ('TransX-', 'Left', lambda: self.snapCam.translate(1.0, 0, 0)),
            ('TransY+', 'Up', lambda: self.snapCam.translate(0, -1.0, 0)),
            ('TransY-', 'Down', lambda: self.snapCam.translate(0, 1.0, 0)),
            ('TransZ+', '=', lambda: self.snapCam.translate(0, 0, 1.0)),
            ('TransZ-', '-', lambda: self.snapCam.translate(0, 0, -1.0)),
            ('__sep__', None, None),
            ('SnapAdd', 'Shift+s', lambda: self.snapCam.post_snap_task()),
            ('SnapClear', 'Shift+c', lambda: self.snapCam.snap_clear()),
            ('__sep__', None, None),
            ('ToggleMaskSnap', 'Shift+m', lambda: self.snapCam.toggle_snap_mask(),
             {'checkable': True}),
            ('ToggleMaskOverlay', 'Shift+o', lambda: self.snapCam.toggle_overlay_mask(),
             {'checkable': True}),
        ]:
            self.set_menu(self.camera_menu, tpl_args)

        # -------------
        # self.help_menu = self.menuBar().addMenu('&Help')
        # for tpl_args in [
        #     # ('&About', '', self.about),  # fixme: this kills menu entry why?
        #     # ('About &Qt', '', QtGui.qApp.aboutQt),  # fixme: this kills menu entry why?
        # ]:
        #     self.set_menu(self.help_menu, tpl_args)

    def createSlider(self, changedSignal, setterSlot):
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, 360 * 16)
        slider.setSingleStep(16)
        slider.setPageStep(15 * 16)
        slider.setTickInterval(15 * 16)
        slider.setTickPosition(QtGui.QSlider.TicksRight)

        slider.valueChanged.connect(setterSlot)
        changedSignal.connect(slider.setValue)

        return slider

    def setPixmap(self, pixmap):
        print 'setPixmap(): hello'
        self.pixmapLabel.setPixmap(pixmap)
        size = pixmap.size()

        if size - QtCore.QSize(1, 0) == self.pixmapLabelArea.maximumViewportSize():
            size -= QtCore.QSize(1, 0)

        self.pixmapLabel.resize(size)

    def getSize(self):
        text, ok = QtGui.QInputDialog.getText(self, 'Grabber',
                'Enter pixmap size:', QtGui.QLineEdit.Normal,
                '%d x %d' % (self.glWidgetWorld.width(), self.glWidgetWorld.height()))

        if not ok:
            return QtCore.QSize()

        regExp = QtCore.QRegExp('([0-9]+) *x *([0-9]+)')

        if regExp.exactMatch(text):
            width = int(regExp.cap(1))
            height = int(regExp.cap(2))
            if width > 0 and width < 2048 and height > 0 and height < 2048:
                return QtCore.QSize(width, height)

        return self.glWidgetWorld.size()


g_li_windows = []


def g_exit_app():
    for win in g_li_windows:
        win.close_all()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)

    glwidget_world = GLWidgetWorld(QtOpenGL.QGLFormat())

    main_win = MainWindow(glwidget_world)
    main_win.set_on_close_event(g_exit_app)
    g_li_windows.append(main_win)

    obj_win = ObjWindow(glwidget_world)
    obj_win.set_on_close_event(g_exit_app)
    g_li_windows.append(obj_win)

    main_win.show()
    obj_win.show()

    sys.exit(app.exec_())
