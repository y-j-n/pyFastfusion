import numpy as np
import pyqtgraph as pg
print pg.__file__
print pg.__version__


# http://stackoverflow.com/questions/19428251/pyqtgraph-when-i-click-on-a-plotitem-how-do-i-know-which-item-has-been-clicked
class GlvwAbstract(pg.opengl.GLViewWidget):
    # pltClicked = Signal()

    def __init__(self, parent=None, title='TITLE', origin=True, grid=True, dist=40):
        super(GlvwAbstract, self).__init__(parent)

        self.setWindowTitle(title)
        self.opts['distance'] = dist

        if origin:
            self.addItem(pg.opengl.GLScatterPlotItem(
                pos=np.array([(0, 0, 0)]), size=0.5,
                color=(1.0, 0.0, 0.0, 0.5), pxMode=False))

        if grid:
            # fixme: color not working.........!!!!!!!????????
            # gi = pg.opengl.GLGridItem(color=(1.0, 0.0, 0.0, 0.5))
            # ----
            gi = pg.opengl.GLGridItem()
            self.addItem(gi)

        self.li_items = []

    def clear_items(self):
        for item in self.li_items:
            self.removeItem(item)
        del self.li_items[:]

    def get_items(self):
        return self.li_items

    def append_item(self, item):
        self.addItem(item)
        self.li_items.append(item)

    def remove_item(self, item):
        self.removeItem(item)
        self.li_items.remove(item)

    def have_item(self, item):
        return item in self.li_items


# >>> from PySide import QtGui, QtCore
# >>> import sys
# >>> app = QtGui.QApplication(sys.argv)
# >>> from pyqtgraph.opengl import *
# >>> glvw = GLViewWidget()
# >>> dir(glvw)
# ['DrawChildren', 'DrawWindowBackground', 'IgnoreMask', 'PaintDeviceMetric', 'PdmDepth', 'PdmDpiX', 'PdmDpiY', 'PdmHeight', 'PdmHeightMM', 'PdmNumColors', 'PdmPhysicalDpiX', 'PdmPhysicalDpiY', 'PdmWidth', 'PdmWidthMM', 'RenderFlag', 'RenderFlags', '__METAOBJECT__', '__class__', '__delattr__', '__dict__', '__doc__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'acceptDrops', 'accessibleDescription', 'accessibleName', 'actionEvent', 'actions', 'activateWindow', 'addAction', 'addActions', 'addItem', 'adjustSize', 'autoBufferSwap', 'autoFillBackground', 'backgroundRole', 'baseSize', 'bindTexture', 'blockSignals', 'cameraPosition', 'changeEvent', 'checkOpenGLVersion', 'childAt', 'childEvent', 'children', 'childrenRect', 'childrenRegion', 'clearFocus', 'clearMask', 'close', 'closeEvent', 'colorCount', 'colormap', 'connect', 'connectNotify', 'contentsMargins', 'contentsRect', 'context', 'contextMenuEvent', 'contextMenuPolicy', 'convertToGLFormat', 'createWinId', 'cursor', 'customContextMenuRequested', 'customEvent', 'deleteLater', 'deleteTexture', 'depth', 'destroy', 'destroyed', 'devType', 'disconnect', 'disconnectNotify', 'doneCurrent', 'doubleBuffer', 'dragEnterEvent', 'dragLeaveEvent', 'dragMoveEvent', 'drawItemTree', 'drawTexture', 'dropEvent', 'dumpObjectInfo', 'dumpObjectTree', 'dynamicPropertyNames', 'effectiveWinId', 'emit', 'ensurePolished', 'enterEvent', 'evalKeyState', 'event', 'eventFilter', 'findChild', 'findChildren', 'focusInEvent', 'focusNextChild', 'focusNextPrevChild', 'focusOutEvent', 'focusPolicy', 'focusPreviousChild', 'focusProxy', 'focusWidget', 'font', 'fontInfo', 'fontMetrics', 'foregroundRole', 'format', 'frameGeometry', 'frameSize', 'geometry', 'getContentsMargins', 'getViewport', 'glDraw', 'glInit', 'grabFrameBuffer', 'grabGesture', 'grabKeyboard', 'grabMouse', 'grabShortcut', 'graphicsEffect', 'graphicsProxyWidget', 'hasFocus', 'hasMouseTracking', 'height', 'heightForWidth', 'heightMM', 'hide', 'hideEvent', 'inherits', 'initializeGL', 'initializeOverlayGL', 'inputContext', 'inputMethodEvent', 'inputMethodHints', 'inputMethodQuery', 'insertAction', 'insertActions', 'installEventFilter', 'isActiveWindow', 'isAncestorOf', 'isEnabled', 'isEnabledTo', 'isFullScreen', 'isHidden', 'isLeftToRight', 'isMaximized', 'isMinimized', 'isModal', 'isRightToLeft', 'isSharing', 'isValid', 'isVisible', 'isVisibleTo', 'isWidgetType', 'isWindow', 'isWindowModified', 'items', 'itemsAt', 'keyPressEvent', 'keyReleaseEvent', 'keyTimer', 'keyboardGrabber', 'keysPressed', 'killTimer', 'languageChange', 'layout', 'layoutDirection', 'leaveEvent', 'locale', 'logicalDpiX', 'logicalDpiY', 'lower', 'macCGHandle', 'macQDHandle', 'makeCurrent', 'makeOverlayCurrent', 'mapFrom', 'mapFromGlobal', 'mapFromParent', 'mapTo', 'mapToGlobal', 'mapToParent', 'mask', 'maximumHeight', 'maximumSize', 'maximumWidth', 'metaObject', 'metric', 'minimumHeight', 'minimumSize', 'minimumSizeHint', 'minimumWidth', 'mouseDoubleClickEvent', 'mouseGrabber', 'mouseMoveEvent', 'mousePressEvent', 'mouseReleaseEvent', 'move', 'moveEvent', 'moveToThread', 'nativeParentWidget', 'nextInFocusChain', 'noRepeatKeys', 'normalGeometry', 'numColors', 'objectName', 'opts', 'orbit', 'overlayContext', 'overrideWindowFlags', 'overrideWindowState', 'paintEngine', 'paintEvent', 'paintGL', 'paintOverlayGL', 'painters', 'paintingActive', 'palette', 'pan', 'parent', 'parentWidget', 'physicalDpiX', 'physicalDpiY', 'pixelSize', 'pos', 'previousInFocusChain', 'projectionMatrix', 'property', 'qglClearColor', 'qglColor', 'raise_', 'readQImage', 'receivers', 'rect', 'registerUserData', 'releaseKeyboard', 'releaseMouse', 'releaseShortcut', 'removeAction', 'removeEventFilter', 'removeItem', 'render', 'renderPixmap', 'renderText', 'renderToArray', 'repaint', 'resetInputContext', 'resize', 'resizeEvent', 'resizeGL', 'resizeOverlayGL', 'restoreGeometry', 'saveGeometry', 'scroll', 'sender', 'senderSignalIndex', 'setAcceptDrops', 'setAccessibleDescription', 'setAccessibleName', 'setAttribute', 'setAutoBufferSwap', 'setAutoFillBackground', 'setBackgroundColor', 'setBackgroundRole', 'setBaseSize', 'setCameraPosition', 'setColormap', 'setContentsMargins', 'setContextMenuPolicy', 'setCursor', 'setDisabled', 'setEnabled', 'setFixedHeight', 'setFixedSize', 'setFixedWidth', 'setFocus', 'setFocusPolicy', 'setFocusProxy', 'setFont', 'setForegroundRole', 'setGeometry', 'setGraphicsEffect', 'setHidden', 'setInputContext', 'setInputMethodHints', 'setLayout', 'setLayoutDirection', 'setLocale', 'setMask', 'setMaximumHeight', 'setMaximumSize', 'setMaximumWidth', 'setMinimumHeight', 'setMinimumSize', 'setMinimumWidth', 'setModelview', 'setMouseTracking', 'setObjectName', 'setPalette', 'setParent', 'setProjection', 'setProperty', 'setShortcutAutoRepeat', 'setShortcutEnabled', 'setSizeIncrement', 'setSizePolicy', 'setStatusTip', 'setStyle', 'setStyleSheet', 'setTabOrder', 'setToolTip', 'setUpdatesEnabled', 'setVisible', 'setWhatsThis', 'setWindowFilePath', 'setWindowFlags', 'setWindowIcon', 'setWindowIconText', 'setWindowModality', 'setWindowModified', 'setWindowOpacity', 'setWindowRole', 'setWindowState', 'setWindowTitle', 'show', 'showEvent', 'showFullScreen', 'showMaximized', 'showMinimized', 'showNormal', 'signalsBlocked', 'size', 'sizeHint', 'sizeIncrement', 'sizePolicy', 'stackUnder', 'startTimer', 'staticMetaObject', 'statusTip', 'style', 'styleSheet', 'swapBuffers', 'tabletEvent', 'testAttribute', 'thread', 'timerEvent', 'toolTip', 'tr', 'trUtf8', 'underMouse', 'ungrabGesture', 'unsetCursor', 'unsetLayoutDirection', 'unsetLocale', 'update', 'updateGL', 'updateGeometry', 'updateMicroFocus', 'updateOverlayGL', 'updatesEnabled', 'viewMatrix', 'visibleRegion', 'whatsThis', 'wheelEvent', 'width', 'widthMM', 'winId', 'window', 'windowFilePath', 'windowFlags', 'windowIcon', 'windowIconText', 'windowModality', 'windowOpacity', 'windowRole', 'windowState', 'windowTitle', 'windowType', 'x', 'y']
