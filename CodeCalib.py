import ctypes
import _ctypes
import sys
import threading
import time
import numpy as np
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
from pykinect2.PyKinectV2 import *
import cv2




def color_2_depth_space(kinect, color_space_point, depth_frame_data):
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
    colorXYs = colorXYs.reshape(kinect.depth_frame_desc.Height, kinect.depth_frame_desc.Width, 2).astype(np.int)  # shape
    colorXs = np.clip(colorXYs[:, :, 0], 0, kinect.color_frame_desc.Width - 1)
    colorYs = np.clip(colorXYs[:, :, 1], 0, kinect.color_frame_desc.Height - 1)
    
    color_frame = kinect.get_last_color_frame()
    color_img = color_frame.reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
    align_color_img = np.zeros((424, 512, 4), dtype=np.uint8)
    align_color_img[:, :] = color_img[colorYs, colorXs, :]
    align_color_img_copy = align_color_img.copy()
    align_color_img_copy = align_color_img_copy[:,:,:3:]
    # align_color_img[:,:230,:] = (0,0,0,0)
    # align_color_img[:,308:,:] = (0,0,0,0)
    # align_color_img[335:, :,:] = (0,0,0,0)
    cv2.imshow('img', align_color_img)
    image = cv2.GaussianBlur(align_color_img,(5,5),0) #Gaussian
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", gray)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,500,
 param1=220,param2=30,minRadius=43, maxRadius=49)
    if circles is not None:
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(image,(i[0],i[1]),1,(0,0,255),3)
        a,b,c = i[0], i[1], i[2]
        # print("a ",a, "b ", b)  #a <-> x pixel, b <-> y pixel
        a = np.int(a)
        b = np.int(b)
        # print("a' ",a, "b'", b)
        w_point = depth_2_world(kinect, kinect._depth_frame_data, _CameraSpacePoint, a,b , as_array= True)
        w_point = w_point*1000
        p1,p2,p3 = w_point
        text = f"Center({a},{b} Radius:{c})"
        text2 = f"world points: {p1:.2f}, {p2:.2f}, {p3:.2f} mm"
        cv2.putText(image, text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        cv2.putText(image, text2, (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow('detected circles', image)
        image_calib = image
        return a,b,w_point, image_calib
    else:
        return None, None, None, None
    


# def on_mouse_click(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:
#         depth_value = img[y,x]
#         depth_in_mm = depth_value   # đơn vị mm
#         world_value = depth_data[y,x]
#         world_points = world_value *1000   # đơn vị mm
#         depth = depth_data_0[y,x]
#         depth_mm = depth    # đơn vị mm
#         print(f"Depth at ({x:}, {y:}): {depth_in_mm:}  mm")
#         print(f"World points at ({x:}, {y:}): {world_points:}  mm")
#         print(f"Depth at ({x:}, {y:}): {depth_mm:}  mm")
#         #print("Depth at ({}, {}): {:} mm".format(x, y, depth_in_meters))

# cv2.namedWindow("img")
# cv2.setMouseCallback("img", on_mouse_click)

def depth_2_world(kinect, depth_frame_data, camera_space_point, a,b, as_array= True):
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
        return data[b,a]

count = 0
w1,w2,w3 = 0,0,0
kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
while True:
    if kinect.has_new_depth_frame():
        x,y,w_points, image_calib = color_2_depth_space(kinect, _ColorSpacePoint, kinect._depth_frame_data)
        if y is not None and x is not None and w_points is not None and image_calib is not None:
            count = count + 1
            if count >= 41:
                print(f"y: {y}, x: {x} pixel")
                print(f"w_points: {w_points} mm")
                w1 += w_points[0]
                w2 += w_points[1]
                w3 += w_points[2]
        if count == 50:
            w1 = w1/10
            w2 = w2/10
            w3 = w3/10
            cv2.imwrite("Image_Calib980.png", image_calib)
            print("đã save img_calib")
            print("count", count)
            print(f"world_point_final: {w1:.2f}, {w2:.2f}, {w3:.2f}")
            break
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
        # if cv2.waitKey(1) & 0xff == ord('s'):
        #      cv2.imwrite("Savedanh1_8.png", img)
        #      print("đã print")
    
    

cv2.destroyAllWindows()
