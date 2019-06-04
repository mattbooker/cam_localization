#include <opencv2/imgproc/imgproc.hpp>

#include <unordered_map>
#include <string>

class CameraGraph {
public:
  CameraGraph(int number_of_cameras = 0, int origin_camera_id = 1)
              : number_of_cameras(number_of_cameras), origin_camera_id(origin_camera_id)  {};

  void loadGraph(std::string file_path);

  cv::Point2f getGlobalPos(int camera_id, cv::Point2f pos_in_camera);

private:
  std::unordered_map<int, cv::Point2f> cam_coordinate_adjustments; // {camera_id : Point(x_shift, y_shift)}

  int number_of_cameras;
  int origin_camera_id;
};
