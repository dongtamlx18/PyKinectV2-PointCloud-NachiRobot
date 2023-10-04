import numpy as np
import cv2
from pykinect2 import PyKinectRuntime
from pykinect2 import PyKinectV2
import open3d as o3d
import ctypes
from pykinect2.PyKinectV2 import *
import socket
import time, serial
import os

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
MOVEX_JOINT = 1  #Join
MOVEX_LINEAR = 2  #liner preprocess
MOVEX_LINEAR_LOWSPEED = 3  #liner main process
MOVEX_LINEAR_UPSPEED = 4  #liner main process upspeed
MOVEX_JOINT = 5  #Join 5%
OPEN_COMPRESSED_AIR = 10    #hút
CLOSE_COMPRESSED_AIR = 9

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
        if signal == b'FIN':
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



#------------- DEPTH_2_WORLD
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

#---------------- COLOR_2_DEPTH_SPACE
def color_2_depth_space(kinect, color_space_point, depth_frame_data, return_aligned_image=False):   #Tạo hàm ánh xạ ảnh màu, với ảnh độ sâu, tạo vùng ROI
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
    align_color_img_copy[:,:230] = (0,0,0)
    align_color_img_copy[:,316:] = (0,0,0)
    align_color_img_copy[:100:, :] = (0,0,0)
    align_color_img_copy[310:, :] = (0,0,0)
    #cv2.imshow('img1', align_color_img)
    cv2.imshow('img', align_color_img_copy)
    angle, save_img, check = processing_2D(align_color_img_copy)
    if return_aligned_image:
        return align_color_img
    if angle is not None and save_img is not None:    
        return angle, save_img, True
    else: 
        return None, None, False

#------------------ GET POINTCLOUD
def Get_Cloud(check_G_Cloud=False):
    while not check_G_Cloud:
        kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color|PyKinectV2.FrameSourceTypes_Depth)
        check_G_Cloud = True
    t = time.time()
    while True:
        if kinect.has_new_depth_frame() and kinect.has_new_color_frame is not None and dt > 4:
            world_points = depth_2_world(kinect, kinect._depth_frame_data, _CameraSpacePoint, as_array= True)
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
            align_color_img = align_color_img.reshape(424*512,4)
            align_color_img = align_color_img[:, :3:]  # remove the fourth opacity channel
            align_color_img = align_color_img[..., ::-1]
            color[:, 0] = align_color_img[:, 0]
            color[:, 1] = align_color_img[:, 1]
            color[:, 2] = align_color_img[:, 2]
            dt = time.time() - t
            file_name = export_to_ply(dynamic_point_cloud, color)
            print(f"thời gian đợi lấy Cloud: {dt} s")
            return file_name
        print("wait to get cloud")
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
                file_name = f"cloud{count_cloud}.ply"
                file_path = os.path.join(file_name)
                file = open(file_path, 'w')
                file.write(header)
                file.write(data)
                file.close()
                print(f"đã lưu file {file_name}")
                return file_name
    
#------------------ XỬ LÝ 2D
#Hàm tạo cờ ngắt, để chụp ảnh
def processing_2D(image_name):               #Xử lý 2D     
    image_name = cv2.GaussianBlur(image_name,(5,5),0) #Gaussian
    # Chuyển đổi ảnh sang ảnh xám
    gray = cv2.cvtColor(image_name, cv2.COLOR_BGR2GRAY)
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
        
        if 210 <= round(perimet,0) <= 240 and len(approx) == 4:         #lấy perimet cố gắng chính xác để khỏi phải sleep
            #print(perimet)
            square_contours.append(approx)
            rect = cv2.minAreaRect(contour)
            angle = rect[2]
            angle = angle + 90  #cần phủ định (for purpose)
            #if angle is not None:
            cv2.drawContours(image_name, square_contours, 0, (0, 255, 0), 2)
            text = f"{angle:.2f}"
            cv2.putText(image_name, text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            #cv2.imshow("Yaw angle", image_name)
            save_img = f"save_img{count_img}.png"
            cv2.imwrite(save_img, image_name)
            print(f"đã lưu file {save_img}")
            return angle, save_img, True          #return save_img name để làm gì ??? - hiển thị
            #print("góc Yaw: ",angle)
    
    return None, None, False
    # Vẽ đường biên hình vuông lên ảnh
    #cv2.drawContours(image_name, square_contours, 0, (0, 255, 0), 2)
    # Hiển thị ảnh kết quả
    #cv2.imshow("Square Object Detection", image_name)
    #return None

#-----------------XỬ LÝ 3D
def Processing_3D(file_name):           #Xử lý 3D
    t = time.time()
    #print("t ", t)
    #Khởi tạo
    pcd = o3d.io.read_point_cloud(file_name, format = 'ply') 
    print("b4 down sample: ", np.asarray(pcd.points).shape)
    pcd = pcd.voxel_down_sample(voxel_size= 4)
    print("after down sample: ", np.asarray(pcd.points).shape)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100,  origin= np.array([0.0, 0.0, 600]))

    #tạo vùng crop
    points = [[-55, -250, 630], [-55, -250, 753], [-55, 240, 630], [-55, 240, 778], [122, -250, 630], [122, -250, 753], [122, 240, 630], [122, 240, 778]]
    points=o3d.utility.Vector3dVector(points)                   
    oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points)
    pcd = pcd.crop(oriented_bounding_box)

    #Segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold= 9,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model

    inlier_cloud = pcd.select_by_index(inliers)    # vùng thuộc mặt phẳng, sẽ tô đỏ
    inlier_cloud.paint_uniform_color([1.0, 0, 0])   #tô đỏ
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    #Outlier Removal
    pcd, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.01) 
    
    def check_max_lable(pcd):
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pcd.cluster_dbscan(eps=10, min_points= 10))
        max_label = labels.max()
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=8, min_points= 8))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheck lại")  
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=8, min_points= 6))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheck lại") 
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=9, min_points= 4))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheck lại") 
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=8, min_points= 2))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheck lại")     
        return labels
    labels = check_max_lable(pcd)
    max_label = labels.max()
    #print("max_lable = ", max_label)            # Code sau này thêm Đkeienj MAx_lable = 1 mới là đúng 
    #print(f"point cloud has {max_label + 1} clusters")
    index_clouds = [pcd.select_by_index(np.where(labels == i)[0]) for i in range(max_label)]
    sub_clouds = [pcd.select_by_index(np.where(labels == i)[0], invert = True) for i in range(max_label)]

    pcd_sub = sub_clouds[0]         # dịnhd dạng pointcloud của sub_cloud là lỗ tròn  
    pcd_index = index_clouds[0]     # dịnhd dạng pointcloud của index_cloud là mặt phẳng

    data_pcd = np.asarray(pcd_sub.points)   #data lỗ tròn dưới dạng mảng numpy
    center = np.mean(data_pcd, axis=0)      # center của lỗ tròn, dạng numpy-mảng
    data_pcd_index = np.asarray(pcd_index.points)   #data mặt phẳng dưới dạng numpy
    center_index = np.mean(data_pcd_index, axis= 0) #center mặt phẳng, dạng numpy

    #chuyển numpy lỗ tròn thành vector và đổi màu center lỗ tròn
    center_point = o3d.geometry.PointCloud()
    center_point.points = o3d.utility.Vector3dVector([center]) # center_point pointcloud lỗ tròn
    center_point.paint_uniform_color([1,0,0])                   

    #chuyển numpy mặt phẳng thành vector và đổi màu center mặt phẳng
    center_point_index = o3d.geometry.PointCloud()
    center_point_index.points = o3d.utility.Vector3dVector([center_index])  # center_point_index pointcloud lỗ tròn
    center_point_index.paint_uniform_color([1, 0,0])          

    #Add thêm center pcloud tâm mặt phẳng
    vector = o3d.utility.Vector3dVector(np.asarray(pcd_index.points))   #tạo vector = vector(pcd_index)
    vector.append(center_index)        # add thêm điểm trọng tâm của pointcloud mặt phẳng
    pcd_index_center = o3d.geometry.PointCloud()
    pcd_index_center.points = o3d.utility.Vector3dVector(vector)        #finally add thêm trọng tâm

    #Add thêm center pcloud tâm lỗ tròn
    vector1 = o3d.utility.Vector3dVector(np.asarray(pcd_sub.points))   #tạo vector = vector(pcd_sub)
    vector1.append(center)        # add thêm điểm trọng tâm của pointcloud mặt phẳng
    pcd_index_center_sub = o3d.geometry.PointCloud()
    pcd_index_center_sub.points = o3d.utility.Vector3dVector(vector1)        #finally add thêm trọng tâm lỗ tròn
    #print("cd_index_center_sub  ", pcd_index_center_sub )

    #Chọn vùng KDTree để chuẩn bị tính pháp tuyến
    pcd_tree = o3d.geometry.KDTreeFlann(pcd_index_center)
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd_index_center.points[-1], 150)
    #print("idx  ", np.asarray(idx))
    pcd_index_center.paint_uniform_color([0.5,0.5,0.5])        #vì files txt không có màu nên cần tô màu để mảng color có dữ liệu
    np.asarray(pcd_index_center.colors)[idx[:], :] = [1, 1, 0]     #tô màu cho vùng liên quan

    #Tương tự với KDTree tâm lỗ tròn (chỉ để visualize)
    pcd_tree1 = o3d.geometry.KDTreeFlann(pcd_index_center_sub)
    [k1, idx1, _] = pcd_tree1.search_radius_vector_3d(pcd_index_center_sub.points[-1], 5)
    #print("idx1  ", np.asarray(idx1))
    pcd_index_center_sub.paint_uniform_color([0.5,0.5,0.5])        #vì files txt không có màu nên cần tô màu để mảng color có dữ liệu
    np.asarray(pcd_index_center_sub.colors)[idx1[:], :] = [1, 1, 0]     #tô màu cho vùng liên quan

    #Tạo biến neighbor_pcd để lọc ra những điểm thuộc vùng KDTree
    neighbor_pcd = o3d.geometry.PointCloud()
    neighbor_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd_index_center.points)[idx])
    neighbor_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(radius=15), fast_normal_computation=False)  #normal
    neighbor_pcd.orient_normals_to_align_with_direction(orientation_reference= np.array([0 ,0, -1]))

    normal_vector = neighbor_pcd.normals[0]

    #Vẽ vector pháp tuyến
    line_points = np.vstack([neighbor_pcd.points[0], neighbor_pcd.points[0] + normal_vector * 25])
    line_indices = np.array([[0, 1]])
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(line_points)
    line_set.lines = o3d.utility.Vector2iVector(line_indices)
    vis = o3d.visualization.Visualizer()
    def visual():  
        vis.create_window(width=640,height=480)
        vis.add_geometry(pcd_index_center_sub)
        vis.add_geometry(pcd_index_center)
        vis.add_geometry(line_set)
        vis.add_geometry(coordinate_frame)
        vis.create_window(width=640,height=480)
        ctr = vis.get_view_control()
        parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera.json")
        ctr.convert_from_pinhole_camera_parameters(parameters)
        vis.capture_screen_image("Image_Open3D_Stastic.png", do_render=True)
        vis.destroy_window()
        # o3d.visualization.draw_geometries([coordinate_frame, line_set, pcd_index_center, pcd_index_center_sub])
    print("center lỗ tròn", center)
    print("normal vector:", normal_vector)

    def calculate_normal(normal_vector, check_normal, oz_vector = np.array([0,0,-1])):
        dot_product = np.dot(normal_vector, oz_vector)  # Tính dot product của hai vector
        length_normal = np.linalg.norm(normal_vector)   # Tính độ dài của vector pháp tuyến
        angle_radians = np.arccos(dot_product / length_normal)# Tính góc giữa vector pháp tuyến và trục Oz bằng hàm arccos
        angle_degrees = np.degrees(angle_radians)   # Chuyển đổi radian sang độ
        if check_normal == 0: return 0, -angle_degrees
        elif check_normal == 1: return -angle_degrees, 0
        elif check_normal == 2: return 0, angle_degrees
        else:  return angle_degrees, 0
    def calculate_matrix(center_matrix):
        T_matrix = np.array([
            [-0.9836, 0.0078, -0.0651, 474.0989],
            [0.0047, 0.9965, 0.0254, -414.7187],
            [0.0532,  0.0141, -0.8008, 714.4704],
            [0, 0, 0, 1]
        ])
        result_matrix = np.dot(T_matrix, center_matrix)
        rounded_matrix = np.around(result_matrix, decimals=2)
        return rounded_matrix
    def Homogenous(normal_vector, center):
        Matrix_RoCam = np.array([[np.cos(np.pi), 0, np.sin(np.pi)],
                                 [0, 1, 0],
                                 [-np.sin(np.pi), 0, np.cos(np.pi)]])
        normal_vector = np.dot(Matrix_RoCam, normal_vector)
        print("new normal_vector", normal_vector)
        direction_vector = normal_vector*25
        T = np.eye(4)  # Ma trận đơn vị 4x4
        T[:3, 3] = direction_vector
        #add_point = np.array([1])          #no need anymore
        #point =  np.hstack((center,add_point))  # Gán vector dịch chuyển #bỏ gán ko cần
        transformed_point = np.dot(T, center) #Homogenous transformation
        if normal_vector[0] < 0 and normal_vector[1] < 0: check_normal = 0  #Trường hợp dùng Pitch < 0
        elif normal_vector[0] < 0 and normal_vector[1] > 0: check_normal = 1  #Trường hợp dùng Roll < 0
        elif normal_vector[0] > 0 and normal_vector[1] > 0: check_normal = 2  #Trường hợp dùng Pitch > 0
        else: check_normal = 3   #Trường hợp dùng Roll > 0
        return transformed_point, check_normal
    center = np.append(center,1)
    center_after_Transform = calculate_matrix(center)           #calculate center point under Robot coordinate
    print("center_transform", center_after_Transform)
    Homo, check_normal = Homogenous(normal_vector, center_after_Transform)    #Move Robot along normal vector
    print("homogenous: ", Homo)
    Roll, Pitch = calculate_normal(normal_vector, check_normal)
    end_time = time.time()
    dt = end_time - t
    print(f"Thời gian xử lý 3D: {dt:.2f} giây")
    visual()
    if check_normal == 0:
        return Pitch
        #if -18 < Pitch or Pitch < -22:  Pitch = -20 + round(np.random.uniform(-1.2,1.2), 2), print("góc Pitch:", Pitch, "độ")
    elif check_normal == 1:
        return Roll
        #if -18 < Roll or Roll < -22:  Roll = -20 + round(np.random.uniform(-1.2,1.2), 2), print("góc Roll:", Roll, "độ")
    elif check_normal == 2:
        return Pitch
        #if 22 < Pitch or Pitch < 18:  Pitch = 20 + round(np.random.uniform(-1.2,1.2), 2), print("góc Roll:", Pitch, "độ")
    else:
        return Roll
        #if 22 < Roll or Roll < 18:  Roll = 20 + round(np.random.uniform(-1.2,1.2), 2), print("góc Roll:", Roll, "độ")        
    return Roll, Pitch, center_after_Transform[0], center_after_Transform[1], center_after_Transform[2], Homo[0], Homo[1], Homo[2]

#-------------------- CODE XỬ LÝ XUNG
def write_read(pic):
   data = pic.readline()
   value_ok = data.decode('utf8', 'ignore')
   return value_ok
def startpoint(pic):
    ret = write_read(pic)
    ret = ret.split("\r")[0]
    ret = ret.split("\n")[0]
    ret = int(ret)
    if ret >= 1:
        xung = ret
        return xung
def currentpoint(pic,XX):
    ret = write_read(pic)
    ret = ret.split("\r")[0]
    ret = ret.split("\n")[0]
    ret = int(ret)
    xung = ret - XX
    return xung


home_pos = np.array([201.7, -273.67, 220.7,0, 0, 0])
position1 = np.array([190.62, -410.20, 25, 0, 0, 0])     #chờ hút
position2 = np.array([190.62, -410.20, 5, 0, 0, 0])       #hút
def robot_pick():
    print("chạy")
    move_robot(home_pos,1)
    print("move chờ")
    move_robot(position1,1)
    print("move hút")
    move_robot(position2,2)
    IO_robot(10)    #hút
    move_robot(position1,2) #về chờ
    move_robot(home_pos,1)  #về home

if __name__ == '__main__':
    socket_initalize()   #socket_initalize()
    print("run")
    robot_pick()
    count_cloud = 999999
    count_img = 9999
    check = False
    #kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
    while True: 
        while not check:   
            kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
            print("Check")
            check = True
        #tạo ra các điều kiện để thực hiện tuần tự
        #home position robot
        if kinect.has_new_depth_frame() and kinect.has_new_color_frame:
            print("wait to process 2D first")
            Yaw, img_name, check = color_2_depth_space(kinect, _ColorSpacePoint, kinect._depth_frame_data)
            print(Yaw, img_name)
            #check = False  #bổ sung để xử lý 2D tránh None
            if Yaw is not None and img_name is not None:
                cloud_name = Get_Cloud()
                Roll,Pitch,X,Y,Z,x,y,z = Processing_3D(cloud_name)             #Lưu ý toạ độ X,Y,Z là toạ độ để lắp, còn toạ độ x,y,z là toạ độ chờ
                if Pitch is not None and X is not None and Y is not None and Z is not None:
                    print("Roll,Pitch,Yaw,X,Y,Z",Roll,Pitch,Yaw,X,Y,Z)
                    coord_wait = [x- 2.275*np.cos(np.pi*Yaw/180), y- 2.275*np.cos(np.pi*(90-Yaw)/180), 170, Yaw, Pitch, Roll]            #chỗ này coor chờ
                    print("Coord_wait", coord_wait)
                    move_robot(coord_wait,2)
                    while True:                       
                        coord = [X- 2.275*np.cos(np.pi*Yaw/180), Y- 2.275*np.cos(np.pi*(90-Yaw)/180), 145, Yaw, Pitch, Roll]
                        print("Coord", coord)
                        move_robot(coord,3)
                        IO_robot(9)
                        move_robot(coord_wait,2)
                        dt_show = time.time()
                        while True:
                            cv2.imshow("OpenCV", cv2.imread(img_name))
                            cv2.imshow("3D Process", cv2.imread("Image_Open3D_stastic.png"))
                            cv2.waitKey(1)
                            dt_off = time.time()
                            timming = dt_off - dt_show
                            if timming > 7:
                                break
                        cv2.destroyAllWindows()
                        kinect.close()
                        robot_pick()      
                        check = False
                        count_cloud -= 1
                        count_img -= 1
                        break
                    # while True:
                    #     move_robot(home_pos,1)
                    
                    #transformation calculate X,Y,Z
                    ### GIẢ SỬ CHO TAY MÁY CHẠY HÚT VẬT SAU KHI XỬ LÝ XONG 3D (PICK OBJECT THEN MOVE TO PRE_PROCESS POSITION)
                    # start = startpoint(pic)
                    #robot_pick()           check coi bao nhiêu giây
                    #robot_preprocess()     check coi bao nhieu giây - y + start + xung 
                    # while True:
                    #     current_position = currentpoint(pic,start)
                    #     if current_position = 150:
                    #           process()
                    #            break          xong vòng lặp như cũ
                    
                    # count_cloud -= 1
                    # count_img -= 1
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
cv2.destroyAllWindows()
