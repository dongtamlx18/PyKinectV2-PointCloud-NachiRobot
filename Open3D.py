import open3d as o3d
import numpy as np
import time
import cv2



def Processing_3D(file_name):
    t = time.time()
    print("t ", t)
    #Khởi tạo
    pcd = o3d.io.read_point_cloud(file_name, format = 'ply') 
    abc1 = pcd
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100,  origin= np.array([0.0, 0.0, 600]))
    o3d.visualization.draw_geometries([pcd, coordinate_frame])
    print("b4 down sample: ", np.asarray(pcd.points).shape)
    pcd = pcd.voxel_down_sample(voxel_size= 4)
    print("after down sample: ", np.asarray(pcd.points).shape)
    

    #tạo vùng crop
    points = [[-55, -250, 630], [-55, -250, 753], [-55, 240, 630], [-55, 240, 778], [122, -250, 630], [122, -250, 753], [122, 240, 630], [122, 240, 778]]
    points=o3d.utility.Vector3dVector(points)                   
    oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points)
    pcd = pcd.crop(oriented_bounding_box)
    
    #o3d.visualization.draw_geometries([pcd, coordinate_frame])  #visualze crop
    #Segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold= 9,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model

    inlier_cloud = pcd.select_by_index(inliers)    # vùng thuộc mặt phẳng, sẽ tô đỏ
    inlier_cloud.paint_uniform_color([1.0, 0, 0])   #tô đỏ
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    #o3d.visualization.draw_geometries([outlier_cloud, coordinate_frame])      #visualize plane segmentation

    #Outlier Removal
    pcd, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.01) 
    #o3d.visualization.draw_geometries([pcd, coordinate_frame])
    def check_max_lable(pcd):       #bảo đảm phân ra được 2 cụm #khỏi thu Pcl lại
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
                    pcd.cluster_dbscan(eps=9, min_points= 9))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheckkkk lạiaaaaaaaaa")   
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=8, min_points= 8))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheckkkk lạiaaaaaaaaa") 
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=9, min_points= 7))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheckkkk lạiaaaaaaaaa") 
        if max_label < 1:
            print("check 1 lần duy nhất")
            with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    pcd.cluster_dbscan(eps=10, min_points= 1))
            max_label = labels.max()
            print("MAX_LABLE = ", max_label, "\n đã có chheckkkk lạiaaaaaaaaa") 
        return labels
    labels = check_max_lable(pcd)
    max_label = labels.max()
    print("max_lable = ", max_label)            # Code sau này thêm Đkeienj MAx_lable = 1 mới là đúng 
    print(f"point cloud has {max_label + 1} clusters")
    index_clouds = [pcd.select_by_index(np.where(labels == i)[0]) for i in range(max_label)]
    sub_clouds = [pcd.select_by_index(np.where(labels == i)[0], invert = True) for i in range(max_label)]
    pcd_sub = sub_clouds[0]         # dịnhd dạng pointcloud của sub_cloud là lỗ tròn  
    pcd_index = index_clouds[0]     # dịnhd dạng pointcloud của index_cloud là mặt phẳng
    pcd_sub.paint_uniform_color([1,0,0])
    pcd_index.paint_uniform_color([1,0.706,0])
    #o3d.visualization.draw_geometries([pcd_sub, pcd_index, coordinate_frame])
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
    print("cd_index_center_sub  ", pcd_index_center_sub )

    #Chọn vùng KDTree để chuẩn bị tính pháp tuyến
    pcd_tree = o3d.geometry.KDTreeFlann(pcd_index_center)
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd_index_center.points[-1], 150)
    print("idx  ", np.asarray(idx))
    pcd_index_center.paint_uniform_color([0.5,0.5,0.5])        #vì files txt không có màu nên cần tô màu để mảng color có dữ liệu
    np.asarray(pcd_index_center.colors)[idx[:], :] = [1, 1, 0]     #tô màu cho vùng liên quan

    #Tương tự với KDTree tâm lỗ tròn (chỉ để visualize)
    pcd_tree1 = o3d.geometry.KDTreeFlann(pcd_index_center_sub)
    [k1, idx1, _] = pcd_tree1.search_radius_vector_3d(pcd_index_center_sub.points[-1], 5)
    print("idx1  ", np.asarray(idx1))
    pcd_index_center_sub.paint_uniform_color([0.5,0.5,0.5])        #vì files txt không có màu nên cần tô màu để mảng color có dữ liệu
    np.asarray(pcd_index_center_sub.colors)[idx1[:], :] = [1, 0, 0]     #tô màu cho vùng liên quan

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
    # def visual():  
    #     vis.create_window(width=640,height=480)
    #     vis.add_geometry(pcd_index_center_sub)
    #     vis.add_geometry(pcd_index_center)
    #     vis.add_geometry(line_set)
    #     vis.add_geometry(coordinate_frame)
    #     vis.create_window(width=640,height=480)
    #     ctr = vis.get_view_control()
    #     parameters = o3d.io.read_pinhole_camera_parameters("ScreenCamera.json")
    #     ctr.convert_from_pinhole_camera_parameters(parameters)
    #     vis.capture_screen_image("test_image_open3d.png", do_render=True)
    #     vis.destroy_window()
    #     # o3d.visualization.draw_geometries([coordinate_frame, line_set, pcd_index_center, pcd_index_center_sub])
    o3d.visualization.draw_geometries([coordinate_frame, line_set, pcd_index_center, pcd_index_center_sub])
    print("center lỗ tròn", center)
    print("normal vector:", normal_vector)

    def calculate_normal(normal_vector,check_normal ,oz_vector = np.array([0,0,-1])):
        dot_product = np.dot(normal_vector, oz_vector)  # Tính dot product của hai vector
        length_normal = np.linalg.norm(normal_vector)   # Tính độ dài của vector pháp tuyến
        angle_radians = np.arccos(dot_product / length_normal)# Tính góc giữa vector pháp tuyến và trục Oz bằng hàm arccos
        angle_degrees = np.degrees(angle_radians)   # Chuyển đổi radian sang độ
        if check_normal == 0: return 0, -angle_degrees
        elif check_normal == 1: return -angle_degrees, 0
        elif check_normal == 2: return 0, angle_degrees
        else:  return angle_degrees, 0

    def Homogenous(normal_vector, center):
        Matrix_RoCam = np.array([[np.cos(np.pi), 0, np.sin(np.pi)],
                                 [0, 1, 0],
                                 [-np.sin(np.pi), 0, np.cos(np.pi)]])
        normal_vector = np.dot(Matrix_RoCam, normal_vector)
        print("new normal_vector", normal_vector)
        direction_vector = normal_vector*10
        T = np.eye(4)  # Ma trận đơn vị 4x4
        T[:3, 3] = direction_vector
        # add_point = np.array([1])
        # point =  np.hstack((center,add_point))  # Gán vector dịch chuyển
        print(f"normalvector[0]={normal_vector[0]} và normalvector[1]={normal_vector[1]}")
        transformed_point = np.dot(T, center_after_Transform) #Homogenous transformation #không cần gán point nữa
        if normal_vector[0] < 0 and normal_vector[1] < 0: check_normal = 0  #Trường hợp dùng Pitch < 0
        elif normal_vector[0] < 0 and normal_vector[1] > 0: check_normal = 1  #Trường hợp dùng Roll < 0
        elif normal_vector[0] > 0 and normal_vector[1] > 0: check_normal = 2  #Trường hợp dùng Pitch > 0
        else: check_normal = 3 #Trường hợp dùng Roll > 0
        return transformed_point, check_normal
    def calculate_matrix(center_matrix):
        T_matrix = np.array([
            [ -0.9853, 0.0091, -0.0144, 433.0721],
            [0.0407, 0.9974, -0.1133, -314.1733],
            [0.0668, 0.0222, -1.2305, 1038.6766],
            [0, 0, 0, 1]
        ])
        result_matrix = np.dot(T_matrix, center_matrix)
        rounded_matrix = np.around(result_matrix, decimals=2)
        return rounded_matrix
    center = np.append(center,1)
    center_after_Transform = calculate_matrix(center)
    print("center_transform", center_after_Transform)
    Homo, check_normal = Homogenous(normal_vector, center_after_Transform)
    print("homogenous: ", Homo)
    Roll, Pitch = calculate_normal(normal_vector, check_normal)
    end_time = time.time()
    print("end_time", end_time)
    dt = end_time - t
    # visual()
    print("check normal = ", check_normal)
    print(f"Thời gian xử lý: {dt:.2f} giây")
          
    return Roll, Pitch, center_after_Transform[0], center_after_Transform[1], center_after_Transform[2], Homo[0], Homo[1], Homo[2]

Roll,Pitch,X,Y,Z,x,y,z = Processing_3D(file_name="cloud_depth_7.ply")
print(f"Roll: {Roll} Pitch:{Pitch} X: {X} Y: {Y} Z: {Z}")

cv2.destroyAllWindows()

