"""Get Pointcloud and save it under ply format
   Using Depth_2_worldpoint + Depth_Mapped_to_Color
"""


import numpy as np
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import mapper
import time
import cv2
import sys
import os
import ctypes
import open3d as o3d

timee = time.time()
def depth_2_world(kinect, depth_frame_data, camera_space_point, as_array=False):
    """
    :param kinect: kinect class
     :param depth_frame_data: kinect._depth_frame_data
    :param camera_space_point: _CameraSpacePoint
    :param as_array: returns the data as a numpy array
    :return: returns the DepthFrame mapped to camera space
    """
    depth2world_points_type = camera_space_point * np.int(512 * 424)
    depth2world_points = ctypes.cast(depth2world_points_type(), ctypes.POINTER(camera_space_point))
    kinect._mapper.MapDepthFrameToCameraSpace(ctypes.c_uint(512 * 424), depth_frame_data, ctypes.c_uint(512 * 424), depth2world_points)
    points = ctypes.cast(depth2world_points, ctypes.POINTER(ctypes.c_float))
    data = np.ctypeslib.as_array(points, shape=(424, 512, 3))
    if not as_array:
        return depth2world_points
    else:
        return data
    
def color_2_depth_space(kinect, color_space_point, depth_frame_data, return_aligned_image=False):
    """
    :param kinect: kinect class
    :param color_space_point: _ColorSpacePoint from PyKinectV2
    :param depth_frame_data: kinect._depth_frame_data
    :param show: shows aligned image with color and depth
    :return: mapped depth to color frame
    """
    # Map Depth to Color Space
    depth2color_points_type = color_space_point * np.int(512 * 424)
    depth2color_points = ctypes.cast(depth2color_points_type(), ctypes.POINTER(color_space_point))
    kinect._mapper.MapDepthFrameToColorSpace(ctypes.c_uint(512 * 424), depth_frame_data, kinect._depth_frame_data_capacity, depth2color_points)
    # depth_x = depth2color_points[color_point[0] * 1920 + color_point[0] - 1].x
    # depth_y = depth2color_points[color_point[0] * 1920 + color_point[0] - 1].y
    colorXYs = np.copy(np.ctypeslib.as_array(depth2color_points, shape=(kinect.depth_frame_desc.Height * kinect.depth_frame_desc.Width,)))  # Convert ctype pointer to array
    colorXYs = colorXYs.view(np.float32).reshape(colorXYs.shape + (-1,))  # Convert struct array to regular numpy array https://stackoverflow.com/questions/5957380/convert-structured-array-to-regular-numpy-array
    colorXYs += 0.5
    colorXYs = colorXYs.reshape(kinect.depth_frame_desc.Height, kinect.depth_frame_desc.Width, 2).astype(np.int)
    colorXs = np.clip(colorXYs[:, :, 0], 0, kinect.color_frame_desc.Width - 1)
    colorYs = np.clip(colorXYs[:, :, 1], 0, kinect.color_frame_desc.Height - 1)
    if return_aligned_image:
        color_frame = kinect.get_last_color_frame()
        color_img = color_frame.reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
        align_color_img = np.zeros((424, 512, 4), dtype=np.uint8)
        align_color_img[:, :] = color_img[colorYs, colorXs, :]
        return align_color_img
    return colorXs, colorYs

def calculate_matrix(matrix):
    T_matrix = np.array([
        [-0.9836, 0.0055, 0.0006, 418.8573],
        [-0.0134, 0.9833, 0.0754, -414.6080],
        [0.0584, 0.0971, -0.9851, 855.8003],
        [0, 0, 0, 1]
    ])

def Get_Cloud():
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color|PyKinectV2.FrameSourceTypes_Depth)
    t = time.time()
    while True:
        if kinect.has_new_depth_frame:
            depth_frame = kinect.get_last_depth_frame
        if kinect.has_new_color_frame:
            color_frame = kinect.get_last_color_frame
        if kinect.has_new_depth_frame() and kinect.has_new_color_frame is not None and dt > 4:
            world_points = depth_2_world(kinect, kinect._depth_frame_data, _CameraSpacePoint, as_array= True)
            # world_points = ctypes.cast(world_points, ctypes.POINTER(ctypes.c_float))
            # world_points = np.ctypeslib.as_array(world_points, shape=(kinect.depth_frame_desc.Height * kinect.depth_frame_desc.Width, 3))
            world_points = world_points.reshape(424*512,3)
            world_points *= 1000  # transform to mm
            dynamic_point_cloud = np.ndarray(shape=(len(world_points), 3), dtype=np.float32)
            # transform to mm
            dynamic_point_cloud[:, 0] = world_points[:, 0]
            dynamic_point_cloud[:, 1] = world_points[:, 1]
            dynamic_point_cloud[:, 2] = world_points[:, 2]

            # update color for .ply file only
            color = np.zeros((len(dynamic_point_cloud), 3), dtype=np.float32)
            # map color to depth frame
            align_color_img = color_2_depth_space(kinect, _ColorSpacePoint, kinect._depth_frame_data, return_aligned_image= True)
            align_color_img = align_color_img.reshape(424*512,4).astype(np.uint8)
            # color_img = kinect.get_last_color_frame().reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
            # # make align rgb/d image
            # align_color_img = np.zeros((kinect.depth_frame_desc.Height, kinect.depth_frame_desc.Width, 4), dtype=np.uint8)
            # align_color_img[:, :] = color_img[Ys, Xs, :]
            # align_color_img = align_color_img.reshape((kinect.depth_frame_desc.Height * kinect.depth_frame_desc.Width, 4)).astype(np.uint8)
            align_color_img = align_color_img[:, :3:]  # remove the fourth opacity channel
            align_color_img = align_color_img[..., ::-1]
            color[:, 0] = align_color_img[:, 0]
            color[:, 1] = align_color_img[:, 1]
            color[:, 2] = align_color_img[:, 2]
            dt = time.time() - t
            export_to_ply(dynamic_point_cloud, color)
            break
        dt = time.time() - t

        def export_to_ply(dynamic_point_cloud, color):
                """
                Inspired by https://github.com/bponsler/kinectToPly
                Writes a kinect point cloud into a .ply file
                return None
                """
                # stack data
                data = np.column_stack((dynamic_point_cloud, color))
                data = data[np.all(data != float('-inf'), axis=1)]  # remove -inf
                # header format of ply file
                header_lines = ["ply",
                        "format ascii 1.0",
                        "comment generated by: python",
                        "element vertex {}".format(int(len(data))),
                        "property float x",
                        "property float y",
                        "property float z",
                        "property uchar red",
                        "property uchar green",
                        "property uchar blue",
                        "end_header"]
                # convert to string
                data = '\n'.join('{} {} {} {} {} {}'.format('%.2f' % x[0], '%.2f' % x[1], '%.2f' % x[2], int(x[3]), int(x[4]), int(x[5])) for x in data)
                header = '\n'.join(line for line in header_lines) + '\n'
                # write file
                file_name = "cloud_calib9989.ply"              #999 - 990 để calib pointcloud
                file_path = os.path.join(file_name)
                file = open(file_path, 'w')
                file.write(header)
                file.write(data)
                file.close()
                print(f"đã lưu file ply{file_name}")


kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color|PyKinectV2.FrameSourceTypes_Depth)
Get_Cloud()
print("đã lưu xong")
processtime = time.time() - timee
print("thời gian", processtime)
