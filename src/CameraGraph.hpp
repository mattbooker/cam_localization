#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/aruco.h>

#include <map>
#include <vector>
#include <set>
#include <queue>

class CameraGraph {
public:
  CameraGraph(int number_of_cameras = 0, int origin_camera_id = 1)
              : number_of_cameras(number_of_cameras), origin_camera_id(origin_camera_id)  {};


  // Adds the marker m to vertex and uses it to build up the graph
  void addVertexInfo(int vertex, aruco::Marker m);

  // Uses a BFS to determine if the graph is connected
  bool isConnected();

  void saveGraph(std::string output_file);

  void loadGraph(std::string input_file);

  cv::Point2f getGlobalPos(int camera_id, cv::Point2f pos_in_camera);

private:
  void addEdge(int fromVertex, int toVertex);

  void calculateCameraCoordinateShifts();

  // Represents an undirected graph where key is the vertex and the value is a vector of vertexs it connects to
  std::map<int, std::vector<int>> adjacency_list; //{camera_id : [camera_id]}

  // Contains all the markers that are located at that vertex
  std::map<int, std::vector<aruco::Marker>> vertex_info; // {camera_id : [Markers]}

  std::map<int, cv::Point2f> cam_coordinate_shifts;

  int number_of_cameras;
  int origin_camera_id;
};
