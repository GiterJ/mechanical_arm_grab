#include <string>

#include <gpd/grasp_detector.h>
#include <X11/Xlib.h>
#include <Eigen/Core>
#include <gpd/candidate/hand.h>
#include <iostream>
#include <fstream>
#ifdef Success 
#undef Success
#endif
namespace gpd {
namespace apps {
namespace detect_grasps {

bool checkFileExists(const std::string &file_name) {
  std::ifstream file;
  file.open(file_name.c_str());
  if (!file) {
    std::cout << "File " + file_name + " could not be found!\n";
    return false;
  }
  file.close();
  return true;
}

int DoMain(int argc, char *argv[]) {
  // Read arguments from command line.
  if (argc < 3) {
    std::cout << "Error: Not enough input arguments!\n\n";
    std::cout << "Usage: detect_grasps CONFIG_FILE PCD_FILE [NORMALS_FILE]\n\n";
    std::cout << "Detect grasp poses for a point cloud, PCD_FILE (*.pcd), "
                 "using parameters from CONFIG_FILE (*.cfg).\n\n";
    std::cout << "[NORMALS_FILE] (optional) contains a surface normal for each "
                 "point in the cloud (*.csv).\n";
    return (-1);
  }

  std::string config_filename = argv[1];
  std::string pcd_filename = argv[2];
  if (!checkFileExists(config_filename)) {
    printf("Error: config file not found!\n");
    return (-1);
  }
  if (!checkFileExists(pcd_filename)) {
    printf("Error: PCD file not found!\n");
    return (-1);
  }

  // Read parameters from configuration file.
  util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();

  // Set the camera position. Assumes a single camera view.
  std::vector<double> camera_position =
      config_file.getValueOfKeyAsStdVectorDouble("camera_position",
                                                 "0.0 0.0 0.0");
  Eigen::Matrix3Xd view_points(3, 1);
  view_points << camera_position[0], camera_position[1], camera_position[2];

  // Load point cloud from file.
  util::Cloud cloud(pcd_filename, view_points);
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cout << "Error: Input point cloud is empty or does not exist!\n";
    return (-1);
  }

  // Load surface normals from file.
  if (argc > 3) {
    std::string normals_filename = argv[3];
    cloud.setNormalsFromFile(normals_filename);
    std::cout << "Loaded surface normals from file: " << normals_filename
              << "\n";
  }

  GraspDetector detector(config_filename);

  // Preprocess the point cloud.
  detector.preprocessPointCloud(cloud);

  // If the object is centered at the origin, reverse all surface normals.
  bool centered_at_origin =
      config_file.getValueOfKey<bool>("centered_at_origin", false);
  if (centered_at_origin) {
    printf("Reversing normal directions ...\n");
    cloud.setNormals(cloud.getNormals() * (-1.0));
  }

  // Detect grasp poses.
  std::vector<std::unique_ptr<candidate::Hand>> clusters;
  clusters = detector.detectGrasps(cloud);
  
  std::cout << "@";
    for (int i = 0; i < clusters.size(); i++)
  {
    // 调用Hand对象的print方法
    Eigen::Vector3d position_eigen = clusters[i]->getPosition().transpose();
    double *position = position_eigen.data();
    Eigen::Matrix3d orientation_ori = clusters[i]->getFrame();
    double *orientation = orientation_ori.data();
    // //0 3 6 1 4 7 2 5 8
    // 或者使用其他方式来显示Hand对象的信息，例如

    // outfile << position[0] <<" "<< position[1] <<" "<< position[2] << endl;
    // outfile << orientation[0] <<" "<< orientation[1] <<" "<< orientation[2] << endl;
    // outfile << orientation[3] <<" "<< orientation[4] <<" "<< orientation[5] << endl;
    // outfile << orientation[6] <<" "<< orientation[7] <<" "<< orientation[8] << endl;

    std::cout << position[0] <<" "<< position[1] <<" "<< position[2] << ",";
    std::cout << orientation[0] <<" "<< orientation[1] <<" "<< orientation[2] <<"<" ;
    std::cout << orientation[3] <<" "<< orientation[4] <<" "<< orientation[5] <<"<" ;
    std::cout << orientation[6] <<" "<< orientation[7] <<" "<< orientation[8];
    std::cout << "\n" ;
    
  }
  return 0;
}

}  // namespace detect_grasps
}  // namespace apps
}  // namespace gpd

int main(int argc, char *argv[]) {
  XInitThreads();
  return gpd::apps::detect_grasps::DoMain(argc, argv);
}
