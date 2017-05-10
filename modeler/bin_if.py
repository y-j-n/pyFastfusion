import ctypes
import sys
import os
import time
from numpy import *
import cv2


class Svo(object):
    def __init__(self):
        if sys.platform == 'darwin':
            self.bin = ctypes.CDLL('../svo_slam_no_ros/rpg_svo/svo/lib/libsvo.dylib')
        else:
            self.bin = ctypes.CDLL('../svo_slam_no_ros/rpg_svo/svo/lib/libsvo.so')

    def run(self):
        print dir(self.bin)
        print self.bin.main

        # Make sure C functions are prefixed with extern "C", or symbol not found
        #http://stackoverflow.com/questions/11237072/ctypes-not-finding-symbols-in-shared-library-created-using-cmake
        #print self.bin.hoge

        self.bin.main()


class Pcl(object):
    def __init__(self):
        if sys.platform == 'darwin':
            self.bin = ctypes.CDLL('../pcl-sac/build/libpcl_ctypes.dylib')
        else:
            self.bin = ctypes.CDLL('../pcl-sac/build/libpcl_ctypes.so')


class FastFusion(object):
    # http://stackoverflow.com/questions/68645/static-class-variables-in-python
    static_scale_default = 10.0
    static_max_cam_distance_default = 10.0

    # https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
    # The depth images are scaled by a factor of 5000, i.e.,
    #  a pixel value of 5000 in the depth image corresponds to
    #  a distance of 1 meter from the camera, 10000 to 2 meter distance,
    #  etc. A pixel value of 0 means missing value/no data.
    static_image_depth_scale_default = 5000.0

    def __init__(self):
        if sys.platform == 'darwin':
            self.bin = ctypes.CDLL('../fastfusion-master/lib/libonlinefusionctypes.dylib')
        else:
            self.bin = ctypes.CDLL('../fastfusion-master/lib/libonlinefusionctypes.so')

        self.scale = FastFusion.static_scale_default
        self.image_depth_scale = FastFusion.static_image_depth_scale_default
        self.max_cam_distance = FastFusion.static_max_cam_distance_default

        self.path_rgb_dir = 'NA'

    def configure(self, dict_conf):
        filename_associate = dict_conf['filename_associate']
        filename_associate_c = ctypes.c_char_p(filename_associate)
        self.bin.lib_init(filename_associate_c)

        self.path_rgb_dir = os.path.dirname(filename_associate)

        #### use bin.lib_count_frames() instead
        #http://stackoverflow.com/questions/845058/how-to-get-line-count-cheaply-in-python
        #self.c_frames = sum(1 for line in open(filename_associate))
        #print 'c_frames: %s' % self.c_frames

        self.scale = dict_conf.get('scale_ply', FastFusion.static_scale_default)
        self.image_depth_scale = dict_conf.get('image_depth_scale', FastFusion.static_image_depth_scale_default)
        self.max_cam_distance = dict_conf.get('max_cam_distance', FastFusion.static_max_cam_distance_default)

    def count_frames(self):
        #return self.c_frames
        return self.bin.lib_count_frames()

    def delete(self):
        self.bin.lib_clean()

    def fusion_frame(self):
        self.bin.lib_fusion_frame()

    def increment_frame(self):
        self.bin.lib_increment_frame()

    def test_main(self):
        ##self.bin.lib_main()

        for i in range(4+1):
        # for i in range(0+1):
            self.fusion_frame()
            if i % 4 == 0:
                af_rot = self.get_rot_current()
                af_trans = self.get_trans_current(scale=False)
                af_intr = self.get_intrinsic_current()

                # PNG image data, 640 x 480, 16-bit grayscale, non-interlaced
                # must use cv2.IMREAD_UNCHANGED to keep 16-bit
                depth = cv2.imread(self.get_path_depth_current(), cv2.IMREAD_UNCHANGED)
                # cv2.imshow('depth', depth)  # fixme: why not working????
                # print depth.size
                # print depth.min(), depth.max()  # 0 19260

                # PNG image data, 640 x 480, 8-bit/color RGB, non-interlaced
                rgb = cv2.imread(self.get_path_rgb_current(), cv2.IMREAD_COLOR)  # no alpha

                stamp = self.get_str_timestamp_current()
                print 'ppp stamp: %s' % stamp

                sensor_data_raw = self.get_sensor_data(
                    af_rot, af_trans, af_intr,
                    depth, rgb, stamp, dumpfile=True)
                print '# of valid sensor records (raw): %s' % (len(sensor_data_raw)/8)
                print sensor_data_raw
                # -----------------
                af_hdata = load('%s_hdata.npy' % stamp)
                sensor_data_model = self.get_sensor_data(
                    af_rot, af_trans, af_intr,
                    depth, rgb, stamp, hdata=af_hdata, dumpfile=True)
                print '# of valid sensor records (model): %s' % (len(sensor_data_model)/8)
                print sensor_data_model

            self.increment_frame()

        self.delete()

    # deprecated
    def get_f_timestamp_current(self):
        # https://docs.python.org/2/library/ctypes.html#passing-pointers-or-passing-parameters-by-reference
        out = ctypes.c_double()
        self.bin.lib_get_f_timestamp_current(ctypes.byref(out))
        return out.value

    def get_str_timestamp_current(self):
        p = ctypes.create_string_buffer(256)
        self.bin.lib_get_str_timestamp_current(p)
        return p.value

    def get_rot_current(self):
        fa = (ctypes.c_float * 9)()
        self.bin.lib_get_rot_current(fa)
        return ctypeslib.as_array(fa)

    def get_trans_current(self, scale=True):
        fa = (ctypes.c_float * 3)()
        self.bin.lib_get_trans_current(fa)
        arr = ctypeslib.as_array(fa)
        return arr * self.scale if scale else arr

    def get_intrinsic_current(self):
        fa = (ctypes.c_float * 4)()
        self.bin.lib_get_intrinsic_current(fa)
        return ctypeslib.as_array(fa)

    def get_path_depth_current(self):
        p = ctypes.create_string_buffer(256)
        self.bin.lib_get_path_depth_current(p)
        return p.value

    def get_path_rgb_current(self):
        p = ctypes.create_string_buffer(256)
        self.bin.lib_get_path_rgb_current(p)
        return p.value

    # Get flat index data for glBufferData(GL_ELEMENT_ARRAY_BUFFER,...
    def get_vbo_index(self):
        c_elems = self.bin.lib_get_face_vertices_count()
        ia = (ctypes.c_int * c_elems)()
        self.bin.lib_get_vbo_index(c_elems, ia)
        return ctypeslib.as_array(ia)

    # Get flat index data for glBufferData(GL_ARRAY_BUFFER,...
    def get_vbo_vertex(self):
        c_elems = self.bin.lib_get_vertex_count() * 6
        af_scaled = (ctypes.c_float * c_elems)()
        self.bin.lib_get_vbo_vertex(c_elems, af_scaled,
                                    ctypes.c_float(self.scale))
        return ctypeslib.as_array(af_scaled)

    @staticmethod
    def init_posinfo_disk():
        try:
            os.remove('posinfo.csv')
        except OSError:
            pass

    def append_posinfo_disk(self):
        self.bin.lib_append_posinfo_disk()

    # native C calls in this method are thread-safe
    def get_sensor_data(self, af_rot, af_trans, af_intr,
                        depth, rgb, stamp, hdata=None, dumpfile=False):
        nxny = 640*480
        af_rot_ctypes = (ctypes.c_float * 9)()
        af_rot_ctypes[:] = af_rot

        af_trans_ctypes = (ctypes.c_float * 3)()
        af_trans_ctypes[:] = af_trans

        af_intr_ctypes = (ctypes.c_float * 4)()
        af_intr_ctypes[:] = af_intr

        ai_depth_ctypes = (ctypes.c_int16 * nxny)()
        #print depth
        depth.resize(1, nxny)
        ai_depth_ctypes[:] = depth.tolist()[0]
        # 0 19260
        # print 'min max of ai_depth_ctypes: %s %s'\
        #       % (min(ai_depth_ctypes), max(ai_depth_ctypes))

        b = array(rgb[:, :, 0])
        b.resize(1, nxny)
        g = array(rgb[:, :, 1])
        g.resize(1, nxny)
        r = array(rgb[:, :, 2])
        r.resize(1, nxny)
        ai_b_ctypes = (ctypes.c_uint8 * nxny)()
        ai_g_ctypes = (ctypes.c_uint8 * nxny)()
        ai_r_ctypes = (ctypes.c_uint8 * nxny)()
        ai_b_ctypes[:] = b.tolist()[0]
        ai_g_ctypes[:] = g.tolist()[0]
        ai_r_ctypes[:] = r.tolist()[0]

        af_hdata_opt_ctypes = ctypes.c_void_p(0)
        if hdata is not None:
            af_hdata_opt_ctypes = (ctypes.c_float * nxny)()
            hdata /= self.scale
            hdata.resize(1, nxny)
            list_hdata = hdata.tolist()[0]
            af_hdata_opt_ctypes[:] = list_hdata

        af_scaled_out = (ctypes.c_float * (nxny*8))()
        # https://docs.python.org/2/library/ctypes.html
        p_stamp = ctypes.create_string_buffer(stamp)

        c_elements = self.bin.lib_compute_sensor_data(
            af_rot_ctypes, af_trans_ctypes, af_intr_ctypes,
            ai_depth_ctypes, ai_b_ctypes, ai_g_ctypes, ai_r_ctypes,
            af_hdata_opt_ctypes,
            ctypes.c_float(self.image_depth_scale),
            ctypes.c_float(self.max_cam_distance),
            p_stamp,
            af_scaled_out, ctypes.c_float(self.scale),
            1 if dumpfile else 0)
        return ctypeslib.as_array(af_scaled_out)[0:c_elements]

    def write_ply_data_current(self, i_frame):
        self.bin.lib_write_ply(i_frame)
        print 'wrote frame_%s.ply' % i_frame

    def get_ply_data(self):
        scale = self.scale

        print 'coping vertex data...'
        c_vertex = self.bin.lib_get_vertex_count()
        print 'c_vertex: %s' % c_vertex

        af_0 = (ctypes.c_float * c_vertex)()
        af_1 = (ctypes.c_float * c_vertex)()
        af_2 = (ctypes.c_float * c_vertex)()
        ia_0 = (ctypes.c_int * c_vertex)()
        ia_1 = (ctypes.c_int * c_vertex)()
        ia_2 = (ctypes.c_int * c_vertex)()
        self.bin.lib_get_vertex_list(c_vertex, af_0, af_1, af_2, ia_0, ia_1, ia_2)
        list_vertex = array([  # these are numpy arrays
            ctypeslib.as_array(af_0)*scale,
            ctypeslib.as_array(af_1)*scale,
            ctypeslib.as_array(af_2)*scale,
            ctypeslib.as_array(ia_0)/255.,
            ctypeslib.as_array(ia_1)/255.,
            ctypeslib.as_array(ia_2)/255.]).T
        #print list_vertex

        print 'coping face data...'
        c_face_vertices = self.bin.lib_get_face_vertices_count()
        print 'c_face_vertices: %s' % c_face_vertices
        c_faces = c_face_vertices / 3
        print 'c_faces: %s' % c_faces
        # assuming all mesh elements are triangles
        IntArrayFace = ctypes.c_int * c_faces
        ia_face_0 = IntArrayFace()  # to be all 3's
        ia_face_1 = IntArrayFace()
        ia_face_2 = IntArrayFace()
        ia_face_3 = IntArrayFace()

        self.bin.lib_get_face_list(c_faces, ia_face_0, ia_face_1, ia_face_2, ia_face_3)
        list_face = array([  # these are numpy arrays
            ctypeslib.as_array(ia_face_0),
            ctypeslib.as_array(ia_face_1),
            ctypeslib.as_array(ia_face_2),
            ctypeslib.as_array(ia_face_3)]).T
        #print list_face

        return {'vertex': list_vertex, 'face': list_face}

    # DEPRECATED: This is slow and generating duplicate vertices.
    # Use get_vbo_index/vertex() instread.
    @staticmethod
    def get_list_vertex_packed(ply_data):
        list_ply_vertex = ply_data['vertex']
        list_ply_face = ply_data['face']
        list_vertex_packed = []
        for face in list_ply_face:
            for id_vertex in face[1:]:
                v = list_ply_vertex[id_vertex]
                list_vertex_packed.extend([v[0], v[1], v[2],
                                           v[3], v[4], v[5]])
        return list_vertex_packed


if __name__ == '__main__':
    dict_config = {
        'filename_associate': '../data-fastfusion-tum/rgbd_dataset-d2/associate.txt',
        'scale_ply': 10.0,
    }
    FastFusion(dict_config).test_main()
