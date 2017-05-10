from PySide import QtGui


class WorldCtrl(QtGui.QGridLayout):
    def __init__(self, glworld):
        super(WorldCtrl, self).__init__()

        self.glworld = glworld
        
        self.setSpacing(5)
        # http://zetcode.com/gui/pyqt4/eventsandsignals/
        qpb_start = QtGui.QPushButton('|<')
        qpb_prev = QtGui.QPushButton('<')
        qpb_next = QtGui.QPushButton('>')
        qpb_end = QtGui.QPushButton('>|')
        qpb_show_traj = QtGui.QPushButton('ShowTraj')
        qpb_start.clicked.connect(lambda: glworld.kf_cmd('start'))
        qpb_prev.clicked.connect(lambda: glworld.kf_cmd('prev'))
        qpb_next.clicked.connect(lambda: glworld.kf_cmd('next'))
        qpb_end.clicked.connect(lambda: glworld.kf_cmd('end'))
        qpb_show_traj.clicked.connect(lambda: glworld.toggle_show_traj())
        
        qpb_datasetdir = QtGui.QPushButton('Load Dataset')
        qpb_datasetdir.clicked.connect(self.on_load_dataset)
        self.qpb_datasetdir = qpb_datasetdir

        qpb_build = QtGui.QPushButton('Build Model')
        qpb_build.clicked.connect(self.on_build_model)
        qpb_build.setEnabled(False)
        self.qpb_build = qpb_build

        qpb_save = QtGui.QPushButton('Save Model')
        qpb_save.clicked.connect(self.on_save_model)
        self.qpb_save = qpb_save

        qpb_clear = QtGui.QPushButton('Clear Model')
        qpb_clear.clicked.connect(lambda: glworld.post_clear_world())
        self.qpb_clear = qpb_clear

        # https://srinikom.github.io/pyside-docs/PySide/QtGui/QGridLayout.html
        # row, col, rowSpan, colSpan
        self.addWidget(qpb_datasetdir, 0, 0, 1, 1)
        self.addWidget(glworld.get_datasetdir_edit(), 0, 1, 1, 4)
        self.addWidget(qpb_build, 1, 0, 1, 3)
        self.addWidget(qpb_save, 1, 3, 1, 1)
        self.addWidget(qpb_clear, 1, 4, 1, 1)
        self.addWidget(qpb_start, 2, 0)
        self.addWidget(qpb_prev, 2, 1)
        self.addWidget(qpb_next, 2, 2)
        self.addWidget(qpb_end, 2, 3)
        self.addWidget(qpb_show_traj, 2, 4)

        self.build_running = False

    def set_datasetdir(self, dirpath):
        if self.build_running:
            print 'build running; do nothing'
            return

        if dirpath == '':  # dialog was canceled
            return

        has_assoc, path_assoc, c_frames = self.glworld.datasetdir_set(dirpath)
        print 'has_assoc: %r c_frames: %d' % (has_assoc, c_frames)

        if has_assoc:
            self.qpb_build.setEnabled(True)
            self.qpb_build.setText('Build Model (%d frames)' % c_frames)
        else:
            self.qpb_build.setEnabled(False)
            self.qpb_build.setText('Build Model')

    def on_load_dataset(self):
        # https://srinikom.github.io/pyside-docs/PySide/QtGui/QFileDialog.html#PySide.QtGui.PySide.QtGui.QFileDialog.getExistingDirectory
        self.set_datasetdir(
            QtGui.QFileDialog.getExistingDirectory(
                self.glworld, 'Open dataset', '../'))

    def on_save_model(self):
        savedir = QtGui.QFileDialog.getExistingDirectory(
            self.glworld, 'Save model to', self.glworld.datasetdir)
        if savedir != '':
            self.glworld.post_save_world(savedir)

    def on_build_model(self):
        self.set_build_iface(False)
        self.glworld.post_build_world()

    def set_build_iface(self, tf):
        self.qpb_datasetdir.setEnabled(tf)
        self.qpb_build.setEnabled(tf)
        self.qpb_save.setEnabled(tf)
        self.qpb_clear.setEnabled(tf)
        self.build_running = not tf