#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/aruco.h>

#include <unordered_map>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <fstream>

class CameraGraphCalibrator {
public:
  CameraGraphCalibrator(int number_of_cameras = 0, int origin_camera_id = 1)
              : number_of_cameras(number_of_cameras), origin_camera_id(origin_camera_id)  {};


  // Adds the marker m to vertex and uses it to build up the graph
  void addVertexInfo(int vertex, aruco::Marker m, cv::Point2f marker_pos);

  // Uses a BFS to determine if the graph is connected
  bool isConnected();

  void saveGraph(std::string file_path);

  cv::Point2f getGlobalPos(int camera_id, cv::Point2f pos_in_camera);

private:
  void addEdge(int fromVertex, int toVertex, int marker_id);

  void calculateCameraCoordinateShifts();

  // Represents an undirected graph where key is the vertex and the value is a vector of vertexes it connects to paired to the marker_id it connects through
  std::unordered_map<int, std::vector<std::pair<int,int>>> adjacency_list; //{camera_id : [(camera_id,marker_id)]}

  // Contains all the markers that are located at that vertex
  std::unordered_map<int, std::unordered_map<int, cv::Point2f>> vertex_info; // {camera_id : {marker_id : Marker_pos}}

  std::unordered_map<int, cv::Point2f> cam_coordinate_adjustments; // {camera_id : Point(x_shift, y_shift)}

  int number_of_cameras;
  int origin_camera_id;
};
