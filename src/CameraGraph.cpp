#include "CameraGraph.hpp"

void CameraGraph::loadGraph(std::string file_path) {
  cv::FileStorage cam_adjustments_file(file_path, cv::FileStorage::READ);
  int file_num_of_cams, file_origin_camera_id;


  cam_adjustments_file["number_of_cameras"]  >> file_num_of_cams;
  cam_adjustments_file["origin_camera_id"] >> file_origin_camera_id;

  if(file_num_of_cams != number_of_cameras) {
    throw cv::Exception(9007, "File :" + file_path + " is calibrated for " + std::to_string(file_num_of_cams) + " cameras. Expected " + std::to_string(number_of_cameras) + " cameras.",
                                    "CameraGraph::loadGraph", __FILE__, __LINE__);
  }

  if(file_origin_camera_id != origin_camera_id) {
    throw cv::Exception(9007, "File :" + file_path + " is calibrated for origin at " + std::to_string(file_origin_camera_id) + ". Expected origin camera at " + std::to_string(origin_camera_id),
                                    "CameraGraph::loadGraph", __FILE__, __LINE__);
  }

  for(int n = 1; n <= number_of_cameras; n++) {
    std::string cam = "cam_" + std::to_string(n);
    cv::Point2f adjustment;

    cam_adjustments_file[cam] >> adjustment;
    cam_coordinate_adjustments[n] = adjustment;
  }

}

cv::Point2f CameraGraph::getGlobalPos(int camera_id, cv::Point2f pos_in_camera) {
  return pos_in_camera + cam_coordinate_adjustments.at(camera_id);
}
