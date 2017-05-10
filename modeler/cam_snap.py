from cam_abstract import AbstractCam
from OpenGL.GL import *

from glvw_snap import GlvwSnap
from glvw_planar import GlvwPlanar
from cam_ctrl import *
from util import *
from bin_if import Pcl

import ctypes
import os
from subprocess import call, Popen
import pyqtgraph as pg
import pcl  # using a local symlink to ../python-pcl-master/pcl
import numpy as np
from numpy import linalg as LA
import cv2
import glob


class SnapCam(AbstractCam):
    def __init__(self, glformat, parent=None, sharewidget=None):
        super(SnapCam, self).__init__(glformat, parent, sharewidget=sharewidget)

        self.mask_snap_enabled = False
        self.mask_overlay_enabled = False
        self.mask_stamp_last = ''

        self.shape_overlay_enabled = False
        # shape_file = '../worlds/world-e6p-full400/ellispsoid.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/a_model/ellipsoid_direct.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/a_model/ellipsoid_LMA.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/a_model/ellipsoid_ransac.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/a_raw/ellipsoid_direct.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/a_raw/ellipsoid_LMA.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/a_raw/ellipsoid_ransac.csv'
        shape_file = '../worlds/world-e6p-full400/shapes/b_model/ellipsoid_direct.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/b_model/ellipsoid_LMA.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/b_model/ellipsoid_ransac.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/b_raw/ellipsoid_direct.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/b_raw/ellipsoid_LMA.csv'
        # shape_file = '../worlds/world-e6p-full400/shapes/b_raw/ellipsoid_ransac.csv'

        self.shape_points = AbstractCam.read_shape_file(shape_file)
        self.shape_stamp_last = ''

        self.snap_task_posted = None
        self.measure_posted = None
        self.pcl = Pcl()

        # ------------------------ snap_win
        glvw = GlvwSnap()
        glvw.show()
        self.snap_win = glvw

        if self.shape_overlay_enabled:
            print '# of points:'
            print self.shape_points.shape
            rgb = [(1.0, 0.0, 0.0, 0.5), (0.0, 1.0, 0.0, 0.5), (0.0, 0.0, 1.0, 0.5)]
            glvw.addItem(pg.opengl.GLScatterPlotItem(
                pos=self.shape_points * self.widget_world.ff.scale,
                size=2.0, color=rgb[1], pxMode=True))
        # --------------------------

        # -------------------------- planar_win
        glvw = GlvwPlanar(self.widget_world)
        glvw.show()
        self.planar_win = glvw
        # --------------------------

        self.log_widget_measure = QtGui.QLineEdit()
        self.log_widget_measure.setReadOnly(True)
        self.measure_last = {'xyz': None, 'count': 0}

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateGL)
        timer.start(100)  # 10 fps

    def get_log_widget_measure(self):
        return self.log_widget_measure

    #override
    def mousePressEvent(self, event):
        super(SnapCam, self).mousePressEvent(event)

        if event.buttons() & QtCore.Qt.RightButton:
            self.post_measure(event.pos())

    #override
    def paintGL(self):
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()  # must be this location!! -------------------
        glLoadIdentity()

        glClearColor(.6, .8, .8, 1.0)  # default background color
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        pos_v, rot_v, stamp = self.get_pose_virt()
        AbstractCam.set_mat_mv(pos_v, rot_v)

        # fixme: without this block, meshes of the world become blur........
        cam_fake = self.widget_world.cam_fake
        if cam_fake > 0:
            cam_fake_pos = self.widget_world.cam_fake_pos
            self.draw_sth(cam_fake, cam_fake_pos)
            # for pos in self.na_pos_cam_bundle:
            #     self.draw_sth(cam_fake, pos)

        self.draw_mesh_world()
        self.depth_rendered = self.grab_depth_rendered()  # must be before glPopMatrix()
        glPopMatrix()  # must be this location!! -------------------

        self.widget_world.set_pose_cam_virt(pos_v, rot_v)

        if self.snap_task_posted is not None:
            use_raw = self.snap_task_posted['use_raw']
            self.snap_task_posted = None
            ff = self.widget_world.ff
            if ff is None:
                print 'ff binary iface isnt ready yet, not getting sensor_data_model'
                return

            rgb_rendered = self.grab_rgb_rendered()  # must be after glPopMatrix()
            # cv2.imshow('rgb rendered', rgb_rendered)
            # cv2.imshow('depth rendered [0, 1]', self.depth_rendered)
            # print self.depth_rendered

            if use_raw:
                depth_img, rgb_img = self.widget_world.get_raw_images(stamp)
                if depth_img is not None and rgb_img is not None:
                    cv2.imshow('depth_img', depth_img)
                    cv2.imshow('rgb_img', rgb_img)
                    self.__snap_model_raw(ff, depth_img, rgb_img,
                                          pos_v, rot_v, stamp, popup=True)
            else:
                if self.mask_snap_enabled:
                    depth_masked = AbstractCam.get_depth_rendered_mask(
                        self.depth_rendered, self.mask_dir, stamp)
                    self.__snap_model(ff, depth_masked, rgb_rendered,
                                      pos_v, rot_v, stamp, popup=True)
                else:
                    self.__snap_model(ff, self.depth_rendered, rgb_rendered,
                                      pos_v, rot_v, stamp, popup=True)

        if self.measure_posted is not None:
            pos = self.measure_posted['pos']
            self.measure_posted = None
            print pos

            rgb_rendered = self.grab_rgb_rendered()  # must be after glPopMatrix()
            self.proc_measure(pos, self.depth_rendered, rgb_rendered,
                              rot_v, pos_v, stamp)

        if self.mask_overlay_enabled and stamp != self.mask_stamp_last:
            self.mask_stamp_last = stamp
            self.overlay_mask(self.grab_rgb_rendered(),
                              self.mask_dir, stamp)

        if self.shape_overlay_enabled and stamp != self.shape_stamp_last:
            self.shape_stamp_last = stamp
            self.overlay_shape(self.widget_world.ff.scale,
                               self.shape_points, pos_v, rot_v, stamp)

    def __snap_model(self, ff, depth_rendered, rgb_rendered,
                     pos, rot, stamp, popup=False):
        af_hdata = SnapCam.convert_to_hdata(depth_rendered)
        depth_dummy = np.zeros((480, 640), dtype=np.int)

        li_model = ff.get_sensor_data(
            np.array(rot), np.array(pos/ff.scale),
            np.array(SnapCam.get_intrinsics_xtion()),
            depth_dummy, rgb_rendered, stamp,
            hdata=af_hdata, dumpfile=False)

        if len(li_model) > 0 and popup:
            self.proc_model(np.array(li_model), 8)  # x y z r g b px py

    def __snap_model_raw(self, ff, depth_img, rgb_img,
                         pos, rot, stamp, popup=False):
        li_model = ff.get_sensor_data(
            np.array(rot), np.array(pos/ff.scale),
            np.array(SnapCam.get_intrinsics_xtion()),
            depth_img, rgb_img, stamp, dumpfile=True)

        if len(li_model) > 0 and popup:
            self.proc_model(np.array(li_model), 8)  # x y z r g b px py

    def proc_measure(self, pos, depth_rendered, rgb_rendered,
                     rot_v, pos_v, stamp):
        depth_of_pos = depth_rendered[pos.y(), pos.x()]  # 1.0 := invalid
        if depth_of_pos > 0.99:
            print 'depth_of_pos is invalid for pos: %s' % pos
            return

        af_hdata = SnapCam.convert_to_hdata(depth_rendered)

        ff = self.widget_world.ff
        if ff is not None:
            model = ff.get_sensor_data(
                np.array(rot_v), np.array(pos_v/ff.scale),
                np.array(SnapCam.get_intrinsics_xtion()),
                np.zeros((480, 640), dtype=np.int),  # dummy
                rgb_rendered,
                stamp, hdata=af_hdata, dumpfile=False)
            # print model[0:40]  # Nx8
            model_mat = model.reshape(len(model)/8, 8)
            pos_mat = np.array([model_mat[:, 6],
                                model_mat[:, 7]]).T
            pos_mat = pos_mat.astype(np.int, copy=False)
            # print pos_mat

            # fixme: this is naive...
            idx_target = -1
            for idx, p in enumerate(pos_mat):
                if p[0] == pos.x() and p[1] == pos.y():
                    idx_target = idx
                    break

            if not idx_target < 0:
                # print model_mat[idx_target]  # 8-vector
                # print model_mat[idx_target][0:3]
                scale_to_meter = 20.0  # 10.0 -> 0.5 m according to ruler50 with Carmin model
                xyz = model_mat[idx_target][0:3]
                xyz_meter = xyz/scale_to_meter

                str_d = '-'
                str_xyz_last = '[-]'
                if self.measure_last['xyz'] is not None:
                    xyz_last_meter = self.measure_last['xyz']/scale_to_meter
                    d = LA.norm(xyz_last_meter - xyz_meter)
                    str_d = str(util_format_3f(d))
                    str_xyz_last = '[%s %s %s]' % (
                        util_format_3f(xyz_last_meter[0]),
                        util_format_3f(xyz_last_meter[1]),
                        util_format_3f(xyz_last_meter[2]))

                str_xyz = '[%s %s %s]' % (
                    util_format_3f(xyz_meter[0]),
                    util_format_3f(xyz_meter[1]),
                    util_format_3f(xyz_meter[2]))
                str_measure = '%s %s dist: %s (unit: meter)' % (
                    str_xyz, str_xyz_last, str_d)
                self.log_widget_measure.setText(str_measure)
                css_black = 'color: rgb(0, 0, 0);'
                css_alert = 'color: rgb(255, 0, 255);'

                count = self.measure_last['count']
                self.log_widget_measure.setStyleSheet(
                    css_black if count % 2 == 0 else css_alert)
                self.measure_last = {'xyz': xyz, 'count': count+1}

    def proc_entire_mesh(self):
        with util_measure_time('proc_entire_mesh......'):
            # 0 seconds; cubi10
            npa_vbo_vertex = self.widget_world.get_mesh_info('npa_vbo_vertex')
            print '# of vertices: %s' % util_format_k(npa_vbo_vertex.shape[0]/6)

            # http://stackoverflow.com/questions/4389517/in-place-type-conversion-of-a-numpy-array
            # pcl lib not accepting float64.  so convert to float32
            npa_vbo_vertex = npa_vbo_vertex.astype(np.float32, copy=False)
            # ncols=6: x y z r g b
        with util_measure_time('proc_entire_mesh......000000000'):
            # 20.44 seconds; cubi10
            self.proc_model(npa_vbo_vertex, 6, use_snap_win=False)

    def proc_entire_raw(self):
        print 'HARDCODED FOR EXPERIMENTS!!!!!!'
        with util_measure_time('proc_entire_raw......'):
            # 11.99 seconds for cubi10-kfs_5
            snap_raw_dir = '../worlds/cubi10-slowly-OK/exp_pts_raw/snap_raw_used_kfs_5'
            # snap_raw_dir = '../worlds/cubi10-slowly-OK/exp_pts_raw/snap_raw_used_kfs_10'
            # snap_raw_dir = '../worlds/cubi10-slowly-OK/exp_pts_raw/snap_raw_used_kfs_15'
            # snap_raw_dir = '../worlds/cubi10-slowly-OK/exp_pts_raw/snap_raw_used_kfs_20'

            li_csv = glob.glob(snap_raw_dir+'/*.csv')
            print '# of csv: %d' % len(li_csv)
            pts_accum = np.array([])
            for idx, csv in enumerate(li_csv):
                print csv
                pts = np.loadtxt(csv, delimiter=';', usecols=(2, 3, 4))
                # print pts.shape, pts
                pts_accum = pts if idx == 0 else np.concatenate((pts_accum, pts))
            pts_accum = pts_accum.reshape(1, -1) * 10.0  # ff.scale=10.0 hardcoded...
            pts_accum = pts_accum.astype(np.float32, copy=False)
            # print pts_accum.shape, pts_accum

        with util_measure_time('proc_entire_raw......0000000000'):
            # 15.91 seconds for cubi10-kfs_5
            self.proc_model(pts_accum, 3, use_snap_win=False)

    def proc_model(self, npa_model, ncols, use_snap_win=True):
        npa_model = npa_model.reshape(1, -1)  # enforce being a row vector
        # print model.dtype
        # print npa_model.shape
        print '# of valid records of model %d with %d cols' % (npa_model.shape[1]/ncols, ncols)
        # print model.shape, model
        model_mat = npa_model.reshape(-1, ncols)
        # print model_mat.shape, model_mat
        pos = np.array([model_mat[:, 0],
                        model_mat[:, 1],
                        model_mat[:, 2]]).T
        # print pos

        if use_snap_win:
            rgb = [(1.0, 0.0, 0.0, 0.5),
                   (0.0, 1.0, 0.0, 0.5),
                   (0.0, 0.0, 1.0, 0.5)]
            color = rgb[len(self.snap_win.get_items()) % 3]
            sp = pg.opengl.GLScatterPlotItem(
                pos=pos, size=2.0, color=color, pxMode=True)
            self.snap_win.append_item(sp)

        # test PCL here........................
        # not flexible?  pcd import/export features are valuable, though.
        if 0:
            p = pcl.PointCloud(pos)
            # print np.asarray(p)
            seg = p.make_segmenter()
            seg.set_model_type(pcl.SACMODEL_PLANE)
            seg.set_method_type(pcl.SAC_RANSAC)
            indices, model = seg.segment()

        if 1:
            tmp_cloud = 'cloud_snap.pcd'
            dirname = './tmp_planes'
            path_models_file = dirname + '/' + 'models.txt'

            pcl.save(pcl.PointCloud(pos), tmp_cloud)
            util_create_empty_dir(dirname)
            if self.widget_ctrl is not None:
                leaf_len_cm = self.widget_ctrl.value(CAM_CTRL_KEY_LEAF_LEN)
                sac_dist_thresh_cm = self.widget_ctrl.value(CAM_CTRL_KEY_SAC_DIST_THRESH)
                fpercent_left_skip = self.widget_ctrl.value(CAM_CTRL_KEY_FPERCENT_LEFT_SKIP)
                cluster_tol_cm = self.widget_ctrl.value(CAM_CTRL_KEY_CLUSTER_TOL)
            else:
                leaf_len_cm = CAM_CTRL_DEFAULT_LEAF_LEN
                sac_dist_thresh_cm = CAM_CTRL_DEFAULT_SAC_DIST_THRESH
                fpercent_left_skip = self.widget_ctrl.value(CAM_CTRL_KEY_FPERCENT_LEFT_SKIP)
                cluster_tol_cm = CAM_CTRL_DEFAULT_CLUSTER_TOL

            leaf_len = leaf_len_cm / CAM_CTRL_SCALE_TO_CM
            sac_dist_thresh = sac_dist_thresh_cm / CAM_CTRL_SCALE_TO_CM
            cluster_tol = cluster_tol_cm / CAM_CTRL_SCALE_TO_CM

            print 'SAC params: leaf_len, sac_dist_thresh, fpercent_left_skip, cluster_tol'
            print '[cm]%g %g %g %g(<-not used for planar seg)' % (leaf_len_cm, sac_dist_thresh_cm, fpercent_left_skip, cluster_tol_cm)
            print '[scaled]%g %g %g %g(<-not used for planar seg)' % (leaf_len, sac_dist_thresh, fpercent_left_skip, cluster_tol)
            self.pcl.bin.lib_process_cloud(
                ctypes.create_string_buffer(tmp_cloud),
                ctypes.create_string_buffer(dirname),
                ctypes.c_float(leaf_len),
                ctypes.c_float(sac_dist_thresh),
                ctypes.c_float(fpercent_left_skip),
                ctypes.c_float(cluster_tol)  # not used for planar seg
            )

            if not os.path.isfile(path_models_file):
                print 'No planar models found...'
            else:
                self.proc_clouds(dirname, path_models_file, sac_dist_thresh)

    def proc_clouds(self, dirname, path_models_file, sac_dist_thresh):
        npa_pcd_files = np.loadtxt(path_models_file, usecols=(0,), dtype='S')
        npa_pcd_points = np.loadtxt(path_models_file, usecols=(1,), dtype='i4')
        npa_abcd = np.loadtxt(path_models_file, usecols=(2, 3, 4, 5), dtype='f4')
        # print npa_pcd_files
        print '# of planar models found: %d' % npa_pcd_files.shape[0]
        # print npa_pcd_points
        # print npa_abcd
        if npa_pcd_files.size == 1:
            # fixme: kludge.......... npa_* becomes a scalar in this case.............
            li_pcd_paths = [dirname + '/' + '0.pcd']
            npa_abcd = [npa_abcd, ]
            npa_pcd_points = [npa_pcd_points, ]
        else:
            li_pcd_paths = [dirname + '/' + f for f in npa_pcd_files]

        # Launch a PLC viewer in background
        if 1:
            SnapCam.killall_pcl_viewer()
            # print li_pcd_paths
            Popen(['pcl_viewer'] + li_pcd_paths)
            # ---- legathy code
            # li_planes = glob.glob(dirname+'/*.pcd')
            # if li_planes:
            #     # call(['pcl_viewer'] + li_planes)
            #     # http://stackoverflow.com/questions/1196074/starting-a-background-process-in-python
            #     Popen(['pcl_viewer'] + li_planes)

        # Refresh planar_win with selectable planes
        max_planes = 50  # at most process max_planes planes
        print 'for planar_win, max_planes: %d' % max_planes
        if 1:
            self.planar_win.clear_items()

            # print li_pcd_paths[0:max_planes]
            for pcd_path, abcd, points in zip(
                    li_pcd_paths[0:max_planes],
                    npa_abcd[0:max_planes],
                    npa_pcd_points[0:max_planes]):
                # plot a planar point cloud
                print pcd_path
                npa_pc = np.asarray(pcl.load(pcd_path))
                print npa_pc.shape
                self.planar_win.append_item(pg.opengl.GLScatterPlotItem(
                    pos=npa_pc, size=0.05, color=(1.0, 1.0, 0.0, 0.5), pxMode=False))

                # plot a grid that fits the point cloud
                print abcd, points
                self.planar_win.add_grid(abcd, npa_pc.shape[0], sac_dist_thresh)

    @staticmethod
    def killall_pcl_viewer():
        print 'subprocess.call: killall pcl_viewer'
        call(['killall', 'pcl_viewer'])

    def toggle_snap_mask(self):
        self.mask_snap_enabled = not self.mask_snap_enabled
        print '[cam] toggle_snap_mask(): self.mask_snap_enabled: %s' % self.mask_snap_enabled

    def toggle_overlay_mask(self):
        self.mask_overlay_enabled = not self.mask_overlay_enabled
        print '[cam] toggle_overlay_mask(): self.mask_overlay_enabled: %s' % self.mask_overlay_enabled

    def post_snap_task(self, use_raw=False):
        print '[cam] post_snap_task(): use_raw: %s' % use_raw
        self.snap_task_posted = {'use_raw': use_raw}

    def post_measure(self, pos):
        self.measure_posted = {'pos': pos}

    def snap_clear(self):
        print '[cam] snap_clear(): hello'
        self.snap_win.clear_items()
        self.planar_win.clear_items()

    # pts_world: Nx3
    @staticmethod
    def overlay_shape(ff_scale, pts_world, pos, rot, stamp):
        print '[cam] updating shape overlay for stamp: '+stamp

        rgb_dir = '../data-fastfusion-tum/rgbd_dataset-e6p/rgb/'  # fixme: kludge!!
        print 'hardcoded: rgb_dir='+rgb_dir
        path_rgb = rgb_dir + stamp + '.png'
        img = cv2.imread(path_rgb, cv2.IMREAD_COLOR)

        # print pts_world
        R = np.array([[rot[0], rot[1], rot[2]],
                      [rot[3], rot[4], rot[5]],
                      [rot[6], rot[7], rot[8]]])
        # R_inv = np.transpose(R)
        # print R
        # print np.dot(R, R_inv)
        t = np.array([pos[0], pos[1], pos[2]]) / ff_scale
        # print t
        # http://stackoverflow.com/questions/12148351/efficiently-rotate-a-set-of-points-with-a-rotation-matrix-in-numpy
        # pts_cam.T = R_inv * (pts_world.T - t.T) and R_inv.T = R
        pts_cam = np.dot(pts_world - t, R)
        # print pts_cam

        if 1:
            fu, fv, cu, cv = SnapCam.get_intrinsics_xtion()
            pts_uv = None
            for x, y, z in pts_cam:
                u = fu * x / z + cu
                v = fv * y / z + cv
                cv2.circle(img, (int(u), int(v)), 2, (0, 255, 0), -1)

                # http://docs.scipy.org/doc/numpy/reference/generated/numpy.concatenate.html
                pts_uv = np.array([[u, v]]) if pts_uv is None else np.concatenate((pts_uv, np.array([[u, v]])))
            # print pts_uv

        cv2.imshow('shape projection', img)
        if 0:
            if stamp == '14.503098' or stamp == '14.637162' or stamp == '2.537886' or stamp == '2.806014':
                filename = stamp+'.png'
                cv2.imwrite(filename, img)
                print 'imwriting '+filename

    @staticmethod
    def overlay_mask(img_grab, dir_mask, stamp):
        print '[cam] updating mask overlay for stamp: '+stamp

        path_mask = dir_mask + '/%s.png' % stamp
        #print path_mask
        img_mask = cv2.imread(path_mask, cv2.IMREAD_COLOR)  # no alpha
        if img_mask is None:
            print 'mask img NOT found: ' + path_mask
        else:
            print 'mask img found: ' + path_mask
            #print 'size_grab %s' % img_grab.size
            #print img_grab.shape
            cv2.imshow('model', img_grab)

            ##img_mask = cv2.imread(path_mask, cv2.IMREAD_UNCHANGED)
            #print 'size_mask %s' % img_mask.size
            #print img_mask.shape  # (480, 640, 3)
            cv2.imshow('mask', img_mask)

            alpha = 0.3
            beta = 1.0 - alpha
            gamma = 0.0
            img_blend = cv2.addWeighted(img_grab, alpha, img_mask, beta, gamma)
            cv2.imshow('blend: model + mask', img_blend)

    @staticmethod
    def convert_to_hdata(depthimg):
        # get depth in cam coords
        # http://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
        near = 2.0  # todo: make it common param with xtion_func
        far = 100.0  # todo: make it common param with xtion_func

        # NumPy array calc is element-wise by default
        # http://wiki.scipy.org/NumPy_for_Matlab_Users
        arr_z_b = depthimg
        arr_z_n = 2.0 * arr_z_b - 1.0
        arr_z_e = 2.0 * near * far / (far + near - arr_z_n * (far - near))
        # for z_e in np.nditer(arr_z_e):
        #     if (z_e > 5.0) and (z_e < 99.0):
        #         sys.stdout.write('%s ' % z_e)
        return arr_z_e
