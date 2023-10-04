import numpy as np
import cv2
from pykinect2 import PyKinectRuntime
from pykinect2 import PyKinectV2
import mapper
import PointCloud
import open3d as o3d
import ctypes
from pykinect2.PyKinectV2 import *
import socket, time


##Khai báo các thông tin Servo robot
PORT_NUM = 48952
SEND_DATA_SIZE = 8
SEND_BUFFER_LEN = SEND_DATA_SIZE * 6
REC_DATA_SIZE = 12
REC_DATA_NUM = 7
REC_IO_DATA_SIZE = 3
REC_BUFFER_LEN = REC_DATA_SIZE * 6 + REC_IO_DATA_SIZE + REC_DATA_NUM


##define a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (('192.168.1.1', PORT_NUM))


## Khởi tạo các IO, liên quan đến biến V7%
MACHINE_ABS_LINEAR = 1  # MOVE BY ABS COORDINATE VALUE RESPECT MACHINE COORDINATE FRAME USING LINEAR INTERPOLATION
MACHINE_ABS_JOINT = 2  # ...
MACHINE_REALATIVE_LINEAR = 3  # ...
MACHINE_REALATIVE_JOINT = 4  # MOVE BY REALATIVE COORDINATE VALUE RESPECT MACHINE COORDINATE FRAME USING JOINT INTERPOLATION
JOINT_ABS_LINEAR = 5  # MOVE BY ABS COORDINATE VALUE RESPECT JOINT COORDINATE FRAME USING LINEAR INTERPOLATION
JOINT_ABS_JOINT = 6  # ...
JOINT_REALATIVE_LINEAR = 7  # ...
JOINT_REALATIVE_JOINT = 8  # MOVE BY REALATIVE COORDINATE VALUE RESPECT JOINT COORDINATE FRAME USING JOINT INTERPOLATION
OPEN_COMPRESSED_AIR = 9
CLOSE_COMPRESSED_AIR = 10


def socket_initalize():
    print('Connecting to {} port {}'.format(*server_address))
    # Connect the socket to the port where the server is listening
    sock.connect(server_address)
 
def socket_close():
    sock.close()


def tool_coordinate():   #recieve current tool position  
    M = "P"
    M = bytes(M, 'utf-8') #M = M.decode('utf-8')
    sock.sendall(M)
    data = sock.recv(1024)
    data = data.decode("utf-8")
    data = data.split(",")
    print("-----------------------")
    print("Current Tool Position")
    print("-----------------------")  
    ###
    print('X    :  ', data[0])
    print('Y    :  ', data[1])
    print('Z    :  ', data[2])
    print('Roll :  ', data[3])
    print('Pitch:  ', data[4])
    print('Yaw  :  ', data[5])


def joint_coordinate(): #receive Joint radiant
    M = "J"
    M = bytes(M, 'utf-8')
    sock.sendall(M)
    data = sock.recv(1024)
    data = data.decode("utf-8")
    data = data.split(",")
    print("-----------------------")
    print("Current Joint Position")
    print("-----------------------")
    ###
    print('Joint 1 :  ', data[0])
    print('Joint 2 :  ', data[1])
    print('Joint 3 :  ', data[2])
    print('Joint 4 :  ', data[3])
    print('Joint 5 :  ', data[4])
    print('Joint 6 :  ', data[5])


def move_robot(move_coord, move_mode):
    #wait for 'REA' status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        sock.sendall(M)
        signal = sock.recv(3)
        if signal == b'REA':
            break
    #prepare data send to Server
    x = "{0:8.2f}".format(move_coord[0])
    y = "{0:8.2f}".format(move_coord[1])
    z = "{0:8.2f}".format(move_coord[2])
    r = "{0:8.2f}".format(move_coord[3])
    p = "{0:8.2f}".format(move_coord[4])
    ya = "{0:8.2f}".format(move_coord[5])
    mode = "{:0>3d}".format(move_mode)
    #biding data and converting
    message = x + y + z + r + p + ya + mode
    message = bytes(message, 'utf-8')
    #send data 'message'
    sock.sendall(message)
    #wait for 'FIN' status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        signal = sock.recv(3)
        if signal == b'Fin':
            break
   
def IO_robot(move_mode):
    # wait for 'REA' status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        sock.sendall(M)
        signal = sock.recv(3)
        if signal == b'REA':
            break
    #data preparation
    x = "{0:8.2f}".format(0)
    y = "{0:8.2f}".format(0)
    z = "{0:8.2f}".format(0)
    r = "{0:8.2f}".format(0)
    p = "{0:8.2f}".format(0)
    ya = "{0:8.2f}".format(0)
    mode = "{:0>3d}".format(move_mode)
    # binding data and converting
    message = x + y + z + r + p + ya + mode
    message = bytes(message, 'utf-8')
    #send data 'message'
    sock.sendall(message)
    # wait for machine 'FIN' status
    M = bytes("A", 'utf-8')
    signal = "busy"
    while True:
        signal = sock.recv(3)
        if signal == b'FIN':
            break


#Tạo hàm ánh xạ ảnh màu, với ảnh độ sâu, tạo vùng ROI
#def color_2_depth():
#    color_2_depth = mapper.color_2_depth_space()
def color_2_depth_space(kinect, color_space_point, depth_frame_data):
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
    color_frame = kinect.get_last_color_frame()
    color_img = color_frame.reshape((kinect.color_frame_desc.Height, kinect.color_frame_desc.Width, 4)).astype(np.uint8)
    align_color_img = np.zeros((424, 512, 4), dtype=np.uint8)
    align_color_img[:, :] = color_img[colorYs, colorXs, :]
    align_color_img_copy = align_color_img.copy()
    align_color_img_copy = align_color_img_copy[:,:,:3:]
    align_color_img_copy[:,:231] = (0,0,0)
    align_color_img_copy[:,324:] = (0,0,0)
    align_color_img_copy[:120:, :] = (0,0,0)
    align_color_img_copy[290:, :] = (0,0,0)
    cv2.imshow('img', align_color_img_copy)
    angle, img_name = process2D(align_color_img_copy)
    if angle is not None and img_name is not None :
        return angle, img_name
    else:
        return None, None
   
   


#Hàm tạo cờ ngắt, để chụp ảnh'''
def process2D(image):
    image = cv2.GaussianBlur(image,(5,5),0) #Gaussian
    # Chuyển đổi ảnh sang ảnh xám
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", gray)
    # Tạo ngưỡng Threshold để tiền xử lý cho edge
    #ret, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    #cv2.imshow("thresh", thresh)
    # Áp dụng bộ lọc Canny để phát hiện biên
    edges = cv2.Canny(gray, 200, 255)
    cv2.imshow("edges", edges)
    # Tìm các đường biên hình vuông
    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Lọc các đường biên hình vuông
    square_contours = []
    for contour in contours:
        perimet = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimet, True)
        if 210 <= round(perimet,0) <= 235 and len(approx) == 4:
        #if round(perimet,0) >= 200 and len(approx) == 4:
            print("perimet", perimet)
            #        
            # time.sleep(1)#đợi 1 giây xong lưu ảnh lại
            square_contours.append(approx)
            rect = cv2.minAreaRect(contour)
            angle = rect[2]
            angle = angle +90 
            #angle = -angle  #không cần phủ định
            cv2.drawContours(image, square_contours, 0, (0, 255, 0), 2)        
            #print(f"đã lưu file {save_img}")
            text = f"{angle:.2f}"
            cv2.putText(image, text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            cv2.imshow("Square Object Detection", image)
            save_img = f"Image_Yaw_Calib{count_img}.png"
            cv2.imwrite(save_img, image)
            return angle, image
            #print("góc Yaw: ",angle)
    return None, None
    # # Vẽ đường biên hình vuông lên ảnh
    # cv2.drawContours(image, square_contours, 0, (0, 255, 0), 2)
    # # Hiển thị ảnh kết quả
    # cv2.imshow("Square Object Detection", image)
    # #return None
#Hàm xử lý ảnh 2D '''

count = 0 
if __name__ == '__main__':              #lúc chạy name = '_main_' tắt mấy file khác
    count_img = 900
    count = 0
    process = 0
    #socket_initalize()
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
    while (True):
        #tạo ra các điều kiện để thực hiện tuần tự
        if kinect.has_new_depth_frame() and kinect.has_new_color_frame:
            process2Dx, img_name = color_2_depth_space(kinect, _ColorSpacePoint, kinect._depth_frame_data)
            if process2Dx is not None and img_name is not None:
                process = process2Dx
                text = f"{process:.2f}"
                print(f"Angle trung bình: {process:.2f} ")
                cv2.putText(img_name, text, (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                while True:
                    cv2.imshow("angle", img_name)
                    if cv2.waitKey(1) & 0xff == ord('q'):
                        print(f"đã break{count}!!!!")
                        break
                break   
                        
                 
        #     # Quit using q
        if cv2.waitKey(1) & 0xff == ord('q'):
             break
        # if cv2.waitKey(1) & 0xff == ord('s'):
        #    cv2.imwrite("Image_Yaw.png", img)
        #    print("đã print")
    cv2.destroyAllWindows()
