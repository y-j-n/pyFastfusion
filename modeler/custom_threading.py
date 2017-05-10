from PySide import QtCore


# http://stackoverflow.com/questions/19769710/start-a-new-thread-in-python-with-a-callback-method-in-main-thread-for-pyqt-appl
class CustomThread(QtCore.QThread):
    def __init__(self, target=None, args=None, on_finished=None):
        super(CustomThread, self).__init__()
        self.target = target
        self.args = args
        if on_finished:
            self.finished.connect(on_finished)

    def run(self, *args, **kwargs):
        # self.target(*args, **kwargs)
        if self.args is not None:
            self.target(*self.args)
        else:
            self.target()
