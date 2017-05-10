from PySide import QtCore, QtGui, QtOpenGL
from glvw_abstract import GlvwAbstract
from glwidget_abstract import GLWidgetAbstract
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from util import util_calc_plane_rot_trans


class GlvwPlanar(GlvwAbstract):
    def __init__(self, worldwidget, parent=None):
        # title = 'Planes (t: toggle keep, s: show toggled)'
        title = 'Planes (right-mouse-click: iterate planes)'
        super(GlvwPlanar, self).__init__(parent, title=title, grid=False)

        self.widget_world = worldwidget
        self.li_grid_info = []

        self.idx_grid_selected = None
        self.marker_item = None

    # override
    def clear_items(self):
        super(GlvwPlanar, self).clear_items()
        del self.li_grid_info[:]

        self.idx_grid_selected = None
        self.marker_item = None

    def add_grid(self, abcd, npts, sac_dist_thresh):
        gg = gl.GLGridItem()
        rot_axis, rot_angle, trans = util_calc_plane_rot_trans(abcd)
        gg.rotate(rot_angle, rot_axis[0], rot_axis[1], rot_axis[2])
        gg.translate(trans[0], trans[1], trans[2])
        self.li_grid_info.append({
            'grid': gg,
            'abcd': abcd,
            'npts': npts,
            'sac_dist_thresh': sac_dist_thresh,
            'pose': (rot_axis, rot_angle, trans),
            'keep': False})
        self.append_item(gg)

    def get_grid_info_selected(self, what='grid'):
        return None if self.idx_grid_selected is None else self.li_grid_info[self.idx_grid_selected][what]

    def set_grid_info_selected_keep(self, keep):
        if self.idx_grid_selected is not None:
            self.li_grid_info[self.idx_grid_selected]['keep'] = keep

    def select_grid_next(self):
        if not self.li_grid_info:
            self.idx_grid_selected = None
            return

        if self.idx_grid_selected is None:
            self.idx_grid_selected = 0
        else:
            self.idx_grid_selected += 1
            if self.idx_grid_selected == len(self.li_grid_info):
                self.idx_grid_selected = 0

    def update_marker(self, trans, keep):
        if self.have_item(self.marker_item):
            self.remove_item(self.marker_item)

        color = (0.0, 1.0, 0.0, 0.5) if keep else (1.0, 0.0, 0.0, 0.5)
        self.marker_item = pg.opengl.GLScatterPlotItem(
            pos=np.array([trans]), size=1.5, color=color, pxMode=False)
        self.append_item(self.marker_item)

    # override
    def keyPressEvent(self, event):
        text = str(event.text())
        if text == 't':
            print 'toggle keep......'
            if self.get_grid_info_selected() is not None:
                pose = self.get_grid_info_selected('pose')
                keep = self.get_grid_info_selected('keep')
                self.set_grid_info_selected_keep(not keep)
                self.update_marker(pose[2], not keep)
            else:
                print 'cannot toggle keep; grid not selected...'
        elif text == 's':
            print 'show concatenated objs with keep toggled...'

            li_vbo_index_obj = []
            sum_npts_cloud_obj = 0
            npa_vbo_index = self.widget_world.get_mesh_info('npa_vbo_index')
            npa_vbo_vertex = self.widget_world.get_mesh_info('npa_vbo_vertex')
            for info in self.li_grid_info:
                if info['keep']:
                    npa_vbo_index_obj = GLWidgetAbstract.get_vbo_index_plane(
                        npa_vbo_index, npa_vbo_vertex,
                        info['abcd'], info['sac_dist_thresh'])
                    li_vbo_index_obj.append(npa_vbo_index_obj)
                    sum_npts_cloud_obj += info['npts']

            if li_vbo_index_obj:
                self.widget_world.post_upload_vbo_index_obj(
                    np.concatenate(li_vbo_index_obj), sum_npts_cloud_obj)
            else:
                self.widget_world.post_upload_vbo_index_obj(
                    np.array([]), sum_npts_cloud_obj)

    # override
    def mousePressEvent(self, ev):
        super(GlvwPlanar, self).mousePressEvent(ev)
        # x = ev.x()
        # y = ev.y()
        # print x, y
        # print ev.pos()
        # print ev.scenePos()  # error...

        if ev.button() == QtCore.Qt.MidButton:
            print 'MidButton hello'

        if ev.button() == QtCore.Qt.RightButton:
            self.select_grid_next()

            if self.get_grid_info_selected() is not None:
                pose = self.get_grid_info_selected('pose')
                keep = self.get_grid_info_selected('keep')
                self.update_marker(pose[2], keep)

                abcd = self.get_grid_info_selected('abcd')
                npts_cloud = self.get_grid_info_selected('npts')
                sac_dist_thresh = self.get_grid_info_selected('sac_dist_thresh')
                print abcd, npts_cloud, sac_dist_thresh

                npa_vbo_index_obj, d_ave, d_std = GLWidgetAbstract.get_vbo_index_plane(
                    self.widget_world.get_mesh_info('npa_vbo_index'),
                    self.widget_world.get_mesh_info('npa_vbo_vertex'),
                    abcd, sac_dist_thresh)
                # print npa_vbo_index_obj

                # npa_vbo_index_obj = np.array([9,10,11,6,7,8])
                # npa_vbo_index_obj = np.array(range(0, 6000))
                self.widget_world.post_upload_vbo_index_obj(
                    npa_vbo_index_obj, npts_cloud, d_ave, d_std)

        # if 0:
        #     # this breaks world/cam window!!!???
        #
        #     # todo: call glView.itemsAt((x, y, w, h)) to get a list of all items inside the box from (x,y) to (x+w,y+h), where each argument is expressed in pixels with (0,0) at the upper-left corner of the view..
        #     # https://groups.google.com/forum/#!topic/pyqtgraph/-G42sSLOOVc
        #     print self.itemsAt((x, y, x+100, y+100))  # fixme: error.........
        #     # using pyqtgraph (devel, 0.9.11) not helps.....
        #     # http://pyqtgraph.narkive.com/vmCELIKm/more-glviewwidget-don-t-work-at-the-same-time
        #     print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        #     print self.isSharing()  # yes


        # self.pltClicked.emit()
