from PySide import QtGui, QtCore

CAM_CTRL_KEY_LEAF_LEN = 'leafLen'
CAM_CTRL_KEY_SAC_DIST_THRESH = 'sacDistThresh'
CAM_CTRL_KEY_FPERCENT_LEFT_SKIP = 'fpercentLeftSkip'
CAM_CTRL_KEY_CLUSTER_TOL = 'clusterTol'

# PCL example's default --------------------------
# CAM_CTRL_DEFAULT_LEAF_LEN = 0.05
# CAM_CTRL_DEFAULT_SAC_DIST_THRESH = 0.10
# CAM_CTRL_DEFAULT_FPERCENT_LEFT_SKIP = 0.30  # 30%
# CAM_CTRL_DEFAULT_CLUSTER_TOL = 0.10
# good for cubi10-slowly-OK, vrar_lite/jun18-best -----------------------
# * CAM_CTRL_SCALE_TO_CM for converting OpenGL's unit to cm unit
CAM_CTRL_SCALE_TO_CM = 5.0
CAM_CTRL_DEFAULT_LEAF_LEN = 0.12 * CAM_CTRL_SCALE_TO_CM
CAM_CTRL_DEFAULT_SAC_DIST_THRESH = 0.50 * CAM_CTRL_SCALE_TO_CM
CAM_CTRL_DEFAULT_FPERCENT_LEFT_SKIP = 0.05
CAM_CTRL_DEFAULT_CLUSTER_TOL = 0.40 * CAM_CTRL_SCALE_TO_CM


class CamCtrl(QtGui.QGridLayout):
    def __init__(self, glcam):
        super(CamCtrl, self).__init__()

        self.setSpacing(5)
        qpb_pose_reset = QtGui.QPushButton('ResetPose')
        qpb_snap_clear = QtGui.QPushButton('SnapClear')
        qpb_toggle_wire = QtGui.QPushButton('WireFrame')  # fixme: no state, not linked to menu gui...
        # ----
        qpb_proc_entire_mesh = QtGui.QPushButton('ProcEntireMesh')
        qpb_proc_entire_raw = QtGui.QPushButton('ProcEntireRaw-EXPER')
        qpb_snap_mesh = QtGui.QPushButton('SnapMesh')
        qpb_snap_raw = QtGui.QPushButton('SnapRaw')
        # ----
        qpb_pose_reset.clicked.connect(lambda: glcam.clear_pose_eye())
        qpb_snap_clear.clicked.connect(lambda: glcam.snap_clear())
        qpb_toggle_wire.clicked.connect(lambda: glcam.toggle_polygon_mode())
        # ----
        qpb_proc_entire_mesh.clicked.connect(lambda: glcam.proc_entire_mesh())
        qpb_proc_entire_raw.clicked.connect(lambda: glcam.proc_entire_raw())
        qpb_snap_mesh.clicked.connect(lambda: glcam.post_snap_task())
        qpb_snap_raw.clicked.connect(lambda: glcam.post_snap_task(use_raw=True))
        # ----

        # ----
        qpb_05_ff = QtGui.QPushButton('05_ff')
        qpb_05_nkf5 = QtGui.QPushButton('05_nkf5')
        qpb_05_nkf10 = QtGui.QPushButton('05_nkf10')
        qpb_05_nkf15 = QtGui.QPushButton('05_nkf15')
        qpb_05_nkf20 = QtGui.QPushButton('05_nkf20')
        qpb_10_ff = QtGui.QPushButton('10_ff')
        qpb_10_nkf5 = QtGui.QPushButton('10_nkf5')
        qpb_10_nkf10 = QtGui.QPushButton('10_nkf10')
        qpb_10_nkf15 = QtGui.QPushButton('10_nkf15')
        qpb_10_nkf20 = QtGui.QPushButton('10_nkf20')
        # ----
        basedir = 'worlds/cubi10-slowly-OK/exp_pts_raw/'
        qpb_05_ff.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_05_ff', basedir+'tmp_planes_05_ff/models.txt', 0.5))
        qpb_05_nkf5.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_05_nkf5', basedir+'tmp_planes_05_nkf5/models.txt', 0.5))
        qpb_05_nkf10.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_05_nkf10', basedir+'tmp_planes_05_nkf10/models.txt', 0.5))
        qpb_05_nkf15.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_05_nkf15', basedir+'tmp_planes_05_nkf15/models.txt', 0.5))
        qpb_05_nkf20.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_05_nkf20', basedir+'tmp_planes_05_nkf20/models.txt', 0.5))
        qpb_10_ff.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_10_ff', basedir+'tmp_planes_10_ff/models.txt', 1.0))
        qpb_10_nkf5.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_10_nkf5', basedir+'tmp_planes_10_nkf5/models.txt', 1.0))
        qpb_10_nkf10.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_10_nkf10', basedir+'tmp_planes_10_nkf10/models.txt', 1.0))
        qpb_10_nkf15.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_10_nkf15', basedir+'tmp_planes_10_nkf15/models.txt', 1.0))
        qpb_10_nkf20.clicked.connect(lambda: glcam.proc_clouds(basedir+'tmp_planes_10_nkf20', basedir+'tmp_planes_10_nkf20/models.txt', 1.0))

        self.addWidget(qpb_pose_reset, 0, 0)
        self.addWidget(qpb_snap_clear, 0, 1)
        self.addWidget(qpb_toggle_wire, 0, 2)
        # ----
        # self.addWidget(qpb_proc_entire_mesh, 1, 0)
        # self.addWidget(qpb_proc_entire_raw, 1, 1)  # for experiments only
        # self.addWidget(qpb_snap_mesh, 1, 2)
        # self.addWidget(qpb_snap_raw, 1, 3)  # for experiments only
        self.addWidget(qpb_proc_entire_mesh, 1, 0)
        self.addWidget(qpb_snap_mesh, 1, 1)
        # ----
        self.addWidget(QtGui.QLabel('%s (cm) [%g]' % (CAM_CTRL_KEY_LEAF_LEN, CAM_CTRL_DEFAULT_LEAF_LEN)), 2, 0)
        self.addWidget(QtGui.QLabel('%s (cm) [%g]' % (CAM_CTRL_KEY_SAC_DIST_THRESH, CAM_CTRL_DEFAULT_SAC_DIST_THRESH)), 2, 1)
        # self.addWidget(QtGui.QLabel('%s (cm) [%g]' % (CAM_CTRL_KEY_CLUSTER_TOL, CAM_CTRL_DEFAULT_CLUSTER_TOL)), 2, 2)  # not used
        self.addWidget(QtGui.QLabel('%s [%g]' % (CAM_CTRL_KEY_FPERCENT_LEFT_SKIP, CAM_CTRL_DEFAULT_FPERCENT_LEFT_SKIP)), 2, 2)
        # ----
        # self.addWidget(qpb_05_ff, 4, 0)
        # self.addWidget(qpb_05_nkf5, 4, 1)
        # self.addWidget(qpb_05_nkf10, 4, 2)
        # self.addWidget(qpb_05_nkf15, 4, 3)
        # self.addWidget(qpb_05_nkf20, 4, 4)
        # self.addWidget(qpb_10_ff, 5, 0)
        # self.addWidget(qpb_10_nkf5, 5, 1)
        # self.addWidget(qpb_10_nkf10, 5, 2)
        # self.addWidget(qpb_10_nkf15, 5, 3)
        # self.addWidget(qpb_10_nkf20, 5, 4)
        # ----

        di_params = {
            CAM_CTRL_KEY_LEAF_LEN: QtGui.QDoubleSpinBox(),
            CAM_CTRL_KEY_SAC_DIST_THRESH: QtGui.QDoubleSpinBox(),
            CAM_CTRL_KEY_CLUSTER_TOL: QtGui.QDoubleSpinBox(),
            CAM_CTRL_KEY_FPERCENT_LEFT_SKIP: QtGui.QDoubleSpinBox(),
        }
        self.di_params = di_params

        sb = di_params[CAM_CTRL_KEY_LEAF_LEN]
        sb.setValue(CAM_CTRL_DEFAULT_LEAF_LEN)
        sb.setSingleStep(0.01)
        self.addWidget(sb, 3, 0)

        sb = di_params[CAM_CTRL_KEY_SAC_DIST_THRESH]
        sb.setValue(CAM_CTRL_DEFAULT_SAC_DIST_THRESH)
        sb.setSingleStep(0.01)
        self.addWidget(sb, 3, 1)

        # sb = di_params[CAM_CTRL_KEY_CLUSTER_TOL]
        # sb.setValue(CAM_CTRL_DEFAULT_CLUSTER_TOL)
        sb = di_params[CAM_CTRL_KEY_FPERCENT_LEFT_SKIP]
        sb.setValue(CAM_CTRL_DEFAULT_FPERCENT_LEFT_SKIP)
        sb.setSingleStep(0.01)
        self.addWidget(sb, 3, 2)

    def value(self, key):
        return self.di_params[key].value()
