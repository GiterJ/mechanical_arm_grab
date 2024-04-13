import subprocess
import sys
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import cv2
from math import sqrt
from position_in_camera import position_in_camera



class Detector(object):

    def __init__(self, fx, fy, cx, cy, width, height, debug = False) -> None:
        
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height
        self.camera_matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=self.width, height=self.height, fx=self.fx,
                                                                     fy=self.fy, cx=self.cx, cy=self.cy)
        self.pipeline = rs.pipeline()

        # 创建一个配置并启用流
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # 创建一个align对象
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # if open debug mode
        self.debug = debug
        
    def visualize_pointcloud(self, filepath):
        """
        Visualize a pcd format pointcloud
        :param filepath: string, pcd file path like "xxx.pcd"
        :return:bool, True
        """
        pcd = o3d.io.read_point_cloud(filepath)
        print(pcd)
        o3d.visualization.draw_geometries([pcd])
        return True

    def picture2pointcloud(self, depth, rgb):
        """
        Transform rgb and depth picture to pointcloud
        :param depth&rgb: depth and rgb picture in cv2 or numpy format
        :return:pcd, pointcloud
        """
        depth_o3d = o3d.geometry.Image(np.asarray(depth))
        rgb_o3d = o3d.geometry.Image(np.asarray(rgb))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.pinhole_camera_intrinsic)
        return pcd
    
    def get_target_position(self, rgb, depth, target="bottle"):
        """
        Get the target's position
        :param rgb&depth: depth and rgb picture in cv2 or numpy format
        :param target: string, target object's name
        :return: labeled picture in cv2 format & target's position in camera frame & bool to show success or fail
        """
        flag, image, position = position_in_camera(rgb, depth, self.camera_matrix, target)
        if self.debug:
            cv2.imshow("img_label", image)
            cv2.waitKey(2000)
        return flag, image, position

    def get_picture_just_once(self):
        """
        Get rgb and depth picture
        :return: rgb and depth picture
        """
        # 从已创建的pipeline中获取一组帧
        frames = self.pipeline.wait_for_frames()

        # 对齐深度帧到彩色帧
        aligned_frames = self.align.process(frames)

        # 获取对齐后的深度帧和彩色帧
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        # 获取深度图的rgb图
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(aligned_color_frame.get_data())

        # 写入到一整张图
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        if self.debug:
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(3000)
        return color_image, depth_image

    def get_picture_from_flow(self):
        """
        Get rgb and depth picture consistently, press "s" to choose a frame
        :return: rgb and depth picture
        """
        print("Start the flow, press 's' to save current frame")
        while True:
            # 从已创建的pipeline中获取一组帧
            frames = self.pipeline.wait_for_frames()

            # 对齐深度帧到彩色帧
            aligned_frames = self.align.process(frames)

            # 获取对齐后的深度帧和彩色帧
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            # 获取深度图的rgb图
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())

            # 写入到一整张图
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            if cv2.waitKey(500) & 0xFF == ord('s'):
                break
        return color_image, depth_image

    def process_output(self, proc_out, proc_err, num=5):
        """
        Process the stdout to position and orientation data
        :param proc_out: byte flow, stdout from the cpp executable file
        :param proc_err: byte flow, stderr from the cpp executable file
        :return: list of ***string***, consist of 5 candidate grasp position and orientation, string to avoid waste source
        """
        if proc_err.decode() != "":
            return [], proc_err.decode()
        # print(proc_out.decode())
        results = proc_out.decode().split("@")[1]
        results = results.split("\n")
        num = len(results)-1
        for i in range(num):
            results[i] = results[i].split(",")
            results[i][0] = results[i][0].split(" ")
            results[i][1] = results[i][1].split("<")
            for j in range(3):
                results[i][1][j] = results[i][1][j].split(" ")
        return results[:num], proc_err.decode()
    
    def process_file_output(self, output_fromfile, num=5):
        """
        Process the output from file
        :param output_fromfile: 2dimension string list, mixture of position and orientation
        :param num:int, amount of candidate position
        :return:float list of position and orientation
        """
        results = []
        for i in range(num):
            trans_list = []
            trans_list_orientation = []
            trans_list.append(list(output_fromfile[i]))
            trans_list_orientation.append(list(output_fromfile[i+1]))
            trans_list_orientation.append(list(output_fromfile[i+2]))
            trans_list_orientation.append(list(output_fromfile[i+3]))
            trans_list.append(trans_list_orientation)
            results.append(trans_list)
        return results


    def Judge(self, target_position, processed_proc_out, processed_proc_err, r_thershold=0.3, x_thershold=0.035, y_thershold=0.04, z_thershold=0.05):
        """
        Judge if the grasp location is suitable
        :param target_position:list of float, target's position in camera frame
        :param processed_proc_out: list of string, consist of candidate grasp point, output of process_output
        :return:bool, success or fail; grasp position; grasp_orientation
        """
        if processed_proc_err != "":
            if not self.debug:
                print("Error occered while running GPD!")
            else:
                print("Error occered while running GPD!\tError:{}".format(processed_proc_err))
            return False, [], []
        for grasp_point in processed_proc_out:
            # dist = sqrt((target_position[0]-float(grasp_point[0][0]))**2+(target_position[1]-float(grasp_point[0][1]))**2+(target_position[2]-float(grasp_point[0][2]))**2)
            # if self.debug:
                # print("dist:{}".format(dist))
            if (target_position[0]-float(grasp_point[0][0]))<x_thershold and (target_position[0]-float(grasp_point[0][0]))<0 and\
                abs(target_position[2]-float(grasp_point[0][2]))<z_thershold and abs(target_position[1]-float(grasp_point[0][1]))<y_thershold:
                print("Found Grasp Point!")
                for i in range(3):
                    grasp_point[0][i] = float(grasp_point[0][i])
                    for j in range(3):
                        grasp_point[1][i][j] = float(grasp_point[1][i][j])
                return True, grasp_point[0], grasp_point[1] 
        print("Beyond the thershold")
        return False, [], []

    def close_detector(self):
        """
        Final process of the pipeline
        """
        self.pipeline.stop()

    def get_grasp_pose(self, rgb, depth, gpd_file_path, target_position, pointcloud_path="./pointcloud.pcd", grasp_count_thershold=20):
        # 调用picture2pointcloud()，得到点云
        pointcloud = self.picture2pointcloud(depth=depth, rgb=rgb)
        # 将点云写入文件，进行调用
        o3d.io.write_point_cloud(pointcloud_path, pointcloud, write_ascii=True)
        if self.debug:
            self.visualize_pointcloud("./pointcloud.pcd")
        
        # 运行gpd主程序，通过标准输出获取候选抓取位置 ***need to modify the parameter***
        proc = subprocess.Popen([gpd_file_path+"/build/detect_grasps", gpd_file_path+"/cfg/eigen_params.cfg", pointcloud_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proc_out, proc_err = proc.communicate()     #获取标准输出
        processed_proc_out, processed_proc_err = self.process_output(proc_out, proc_err)    #处理标准输出
        flag_grasp, grasp_position, grasp_orientation = self.Judge(target_position, processed_proc_out, processed_proc_err)    #判断有无符合要求抓取点
        grasp_count = 1                               #抓取点识别次数
        # 若有符合要求抓取点，进行抓取；若没有，继续识别，最多识别grasp_cout_thershold次
        while not flag_grasp and grasp_count<grasp_count_thershold:
            if self.debug:
                print("Loop {}".format(grasp_count))
            proc = subprocess.Popen([gpd_file_path+"/build/detect_grasps", gpd_file_path+"/cfg/eigen_params.cfg", pointcloud_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            proc_out, proc_err = proc.communicate()     #获取标准输出
            processed_proc_out, processed_proc_err = self.process_output(proc_out, proc_err)    #处理标准输出
            flag_grasp, grasp_position, grasp_orientation = self.Judge(target_position, processed_proc_out, processed_proc_err)    #判断有无符合要求抓取点
            grasp_count = grasp_count + 1
        return flag_grasp, grasp_position, grasp_orientation


def grasp(grasp_position, grasp_orientation):
    """
    Abstract function that describe grasp action
    """
    print("Grasp the target in {} with orientation {}".format(grasp_position, grasp_orientation))



























# if __name__ == "__main__":

#     # 创建detector对象
#     detector = Detector(width=848,height=480,cx=427.5453796386719,cy = 236.88723754882812,
#                         fx=425.6839904785156,fy = 425.6839904785156)
#     # 调用get_picture()，调取相机获取rgb图与深度图
#     rgb, depth = detector.get_picture()
#     if detector.debug:
#         print("RGB shape:{}\tDEPTH shape:{}".format(rgb.shape, depth.shape))
#     # 调用get_position()，获取物体位置
#     flag_target, img_label, target_position = detector.get_target_position(rgb=rgb, depth=depth, target="bottle")
#     if detector.debug:
#         cv2.imshow("img_label", img_label)
#         cv2.waitKey(3000)
#     if flag_target:
#         # 调用picture2pointcloud()，得到点云
#         pointcloud = detector.picture2pointcloud(depth=depth, rgb=rgb)
#         if detector.debug:
#             detector.visualize_pointcloud("cloud.pcd")
#         # 将点云写入文件，进行调用
#         o3d.io.write_point_cloud("./cloud.pcd", pointcloud, write_ascii=True)
        
#         # 运行gpd主程序，通过标准输出获取候选抓取位置 ***need to modify the parameter***
#         proc = subprocess.Popen(["/home/jlinux/gpd-copy-version2-Detector/build/detect_grasps", "/home/jlinux/gpd-copy-version2-Detector/cfg/eigen_params.cfg", "cloud.pcd"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#         proc_out, proc_err = proc.communicate()     #获取标准输出
#         processed_proc_out, processed_proc_err = detector.process_output(proc_out, proc_err)    #处理标准输出
#         if detector.debug:
#             print("\nStdout:{}".format(proc_out.decode()))
#             print("\nStderr:{}".format(proc_err.decode()))
#             print("\nProcessed Stdout{}".format(processed_proc_out))


#         flag_grasp, grasp_position, grasp_orientation = detector.Judge(target_position, processed_proc_out, processed_proc_err, thershold=0.2)    #判断有无符合要求抓取点
#         grasp_count = 0                                 #抓取点识别次数
#         grasp_count_thershold = 20
#         # 若有符合要求抓取点，进行抓取；若没有，继续识别，最多识别grasp_cout_thershold次
#         while not flag_grasp and grasp_count<grasp_count_thershold:
#             proc = subprocess.Popen(["/home/jlinux/gpd-copy-version2-Detector/build/detect_grasps", "/home/jlinux/gpd-copy-version2-Detector/cfg/eigen_params.cfg", "cloud.pcd"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#             proc_out, proc_err = proc.communicate()     #获取标准输出
#             processed_proc_out, processed_proc_err = detector.process_output(proc_out, proc_err)    #处理标准输出
#             flag_grasp, grasp_position, grasp_orientation = detector.Judge(target_position, processed_proc_out, processed_proc_err)
#             grasp_count = grasp_count + 1
        
#         # 抓取或输出提示信息
#         if flag_grasp:
#             grasp(grasp_position, grasp_orientation)
#         else:
#             print("Can't Grasp!")

#     # 关闭视频流
#     detector.close_detector()
    
if __name__ == '__main__':
# 创建detector对象
    detector = Detector(width=848,height=480,cx=427.5453796386719,cy = 236.88723754882812,
                        fx=425.6839904785156,fy = 425.6839904785156, debug=True)

    # 获取rgb图和深度图
    rgb, depth = detector.get_picture_from_flow()

    # 调用get_position()，获取物体位置
    flag_target, img_label, target_position = detector.get_target_position(rgb=rgb, depth=depth, target="bottle")
    print(img_label, target_position)
