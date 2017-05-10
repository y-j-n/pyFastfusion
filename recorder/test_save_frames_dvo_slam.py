#!/usr/bin/env python

from primesense import openni2
from primesense import _openni2 as c_api
import numpy as np
import time
import cv2
import sys
import os
import shutil
import threading


def get_timestamp_str(timestamp):
    f_timestamp = float(timestamp)/1000000
    return "{:0.6f}".format(f_timestamp)


def save_list_img_dir(name_dir, list_timestamp, list_img):
    if os.path.exists(name_dir):
        shutil.rmtree(name_dir)
    os.mkdir(name_dir)

    for index, img in enumerate(list_img):
        path_img = name_dir + '/%s.png' % list_timestamp[index]
        cv2.imwrite(path_img, img)

    name_txt = '%s.txt' % name_dir
    with open(name_txt, 'w') as f:
        for timestamp in list_timestamp:
            line = '%s %s/%s.png\n' % (timestamp, name_dir, timestamp)
            f.write(line)


def process_frames_color(list_frames, resol_x):
    list_timestamp = []
    list_img = []
    for frame in list_frames:
        list_timestamp.append(get_timestamp_str(frame.timestamp))

        img_rgb = np.ndarray((frame.height, frame.width, 3),
                             dtype=np.uint8,
                             buffer=frame.get_buffer_as_uint8())
        if resol_x != 320:
            # rbg order is different...
            img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

        cv2.flip(img_rgb, 1, img_rgb)
        list_img.append(img_rgb)
    return list_timestamp, list_img


def process_frames_depth(list_frames):
    list_timestamp = []
    list_img = []
    for frame in list_frames:
        list_timestamp.append(get_timestamp_str(frame.timestamp))

        img_depth = np.ndarray((frame.height, frame.width),
                               dtype=np.uint16,
                               buffer=frame.get_buffer_as_uint16())
        cv2.flip(img_depth, 1, img_depth)
        list_img.append(img_depth)
    return list_timestamp, list_img


def monitor_thread(li_frames_depth, li_frames_color):
    while 1:
        if li_frames_depth:
            frame = li_frames_depth[-1]
            img_depth = np.ndarray((frame.height, frame.width),
                                   dtype=np.uint16,
                                   buffer=frame.get_buffer_as_uint16())
            cv2.flip(img_depth, 1, img_depth)
            cv2.imwrite('depth_current.png', img_depth)
            # cv2.imshow('depth current', img_depth)  # fixme: why broken???
            print img_depth.shape

        # if li_frames_color:
        #     frame = li_frames_color[-1]
        #     img_rgb = np.ndarray((frame.height, frame.width, 3),
        #                          dtype=np.uint8,
        #                          buffer=frame.get_buffer_as_uint8())
        #     img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        #
        #     cv2.flip(img_rgb, 1, img_rgb)
        #     cv2.imshow('rgb current', img_rgb)
        #     cv2.imwrite('hoge.png', img_rgb)
        #     print img_rgb.shape, img_rgb
        time.sleep(1)


def main(arg_c_frames):

    openni2.initialize()  # can also accept the path of the OpenNI redistribution

    dev = openni2.Device.open_any()

    # 320x240 -> 60fps ok
    # 640x480 -> 30fps
    # 1280x1024 -> 30fps
    g_params = {
        'c_frames': arg_c_frames,
        'resol_x': 640,
        'resol_y': 480,
        'fps': 30,
        'pixel_format_rgb': c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
        # 'pixel_format_depth': c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
        'pixel_format_depth': c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
    }
    print g_params

    # create color stream
    color_stream = dev.create_color_stream()
    color_stream.set_video_mode(c_api.OniVideoMode(
        pixelFormat=g_params['pixel_format_rgb'],
        resolutionX=g_params['resol_x'],
        resolutionY=g_params['resol_y'],
        fps=g_params['fps']))

    # create depth stream
    depth_stream = dev.create_depth_stream()
    depth_stream.set_video_mode(c_api.OniVideoMode(
        pixelFormat=g_params['pixel_format_depth'],
        resolutionX=g_params['resol_x'],
        resolutionY=g_params['resol_y'],
        fps=g_params['fps']))

    list_frames_color = []
    list_frames_depth = []

    # crap: not working as expected...
    # th = threading.Thread(target=monitor_thread,
    #                       args=(list_frames_depth, list_frames_color))
    # th.daemon = True
    # th.start()

    # ==== BEGIN INTENSIVE ====
    print 'starting color stream'
    color_stream.start()
    print 'starting depth stream'
    depth_stream.start()
    time.sleep(2.0)  # wait for stream to be stable...

    # idling...
    for i in range(10):
        color_stream.read_frame()
        depth_stream.read_frame()

    # record now...
    for i in range(g_params['c_frames']):
        frame_color = color_stream.read_frame()
        frame_depth = depth_stream.read_frame()
        list_frames_color.append(frame_color)
        list_frames_depth.append(frame_depth)

    print 'stopping color stream'
    color_stream.stop()
    print 'stopping depth stream'
    depth_stream.stop()
    # ==== END INTENSIVE ====

    print 'processing color frames ...'
    list_timestamp_rgb, list_img_rgb = process_frames_color(
        list_frames_color, g_params['resol_x'])
    save_list_img_dir('rgb', list_timestamp_rgb, list_img_rgb)

    print 'processing depth frames ...'
    list_timestamp_depth, list_img_depth = process_frames_depth(
        list_frames_depth)
    save_list_img_dir('depth', list_timestamp_depth, list_img_depth)

    print 'generating assoc.txt ...'
    os.system('./associate.py rgb.txt depth.txt > assoc.txt')
    print '# of lines associations:'
    os.system('wc -l assoc.txt')

    path_dir_rgbd = './data-fastfusion-tum/rgbd_dataset'
    print 'saving data in '+path_dir_rgbd+' ...'
    if os.path.exists(path_dir_rgbd):
        shutil.rmtree(path_dir_rgbd)
    os.mkdir(path_dir_rgbd)
    cmd = 'mv rgb rgb.txt depth depth.txt assoc.txt '+path_dir_rgbd
    # print cmd
    os.system(cmd)

    openni2.unload()


if __name__ == '__main__':
    # img_depth = cv2.imread('../../../data-fastfusion-tum/rgbd_dataset-d2/depth/2.608433.png', cv2.IMREAD_UNCHANGED)
    # cv2.flip(img_depth, 1, img_depth)
    # cv2.imshow('depth current', img_depth)

    if len(sys.argv) != 2:
        print 'Usage: '+sys.argv[0]+' <c_frames>'
    else:
        c_frames = int(sys.argv[1])
        main(c_frames)
