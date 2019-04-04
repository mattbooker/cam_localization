#include "CameraGraphCalibrator.hpp"

void CameraGraphCalibrator::addEdge(int fromVertex, int toVertex, int marker_id) {

  // Only add if not a duplicate
  for(int i = 0; i < adjacency_list[fromVertex].size(); i++) {
    if(adjacency_list[fromVertex][i].first == toVertex)
      return;
  }

  adjacency_list[fromVertex].push_back(std::make_pair(toVertex, marker_id));
  adjacency_list[toVertex].push_back(std::make_pair(fromVertex, marker_id));
}

void CameraGraphCalibrator::addVertexInfo(int vertex, aruco::Marker m, cv::Point2f marker_pos) {

  // Add or replace the marker to ensure always the most up to date marker pos
  vertex_info[vertex][m.id] = marker_pos;

  // Lastly add any edges that this new marker may create a connection to
  for(auto& v : vertex_info) {
    // Any vertex that contains this markers_id we add an edge to (add edge handles duplicates)
    if(v.first != vertex && v.second.count(m.id) == 1)
      addEdge(v.first, vertex, m.id);
  }
}

bool CameraGraphCalibrator::isConnected() {

  // If the graph is not the same size as the number of cameras then it cant be connected
	if(adjacency_list.size() != number_of_cameras)
		return false;

	std::set<int> visited;
	std::queue<int> bfs;

	// Enque the origin_camera_id
	bfs.push(origin_camera_id);

	// Perform a Breadth-first search to check connectedness
	while(!bfs.empty()) {
		int vertex = bfs.front();
		bfs.pop();

		// If we have visited the vertex already skip it, Otherwise add the vertex to visited
		if(visited.count(vertex) != 0)
			continue;
    else
    	visited.insert(vertex);

		// Enque all of the neighbours of the vertex as long as we havent visited them
		for(std::pair<int, int> n : adjacency_list[vertex]) {
			if(visited.count(n.first) == 0)
				bfs.push(n.first);
		}
	}

	// After completing the BFS check if the size of our visited set is the same
	// as the number of cameras. If it is that means it is fully connected.
	if(visited.size() == number_of_cameras)
		return true;
	else
		return false;
}

void CameraGraphCalibrator::saveGraph(std::string file_path) {
  calculateCameraCoordinateShifts();

  cv::FileStorage cam_adjustments_file(file_path, cv::FileStorage::WRITE);

  cam_adjustments_file << "number_of_cameras" << number_of_cameras;
  cam_adjustments_file << "origin_camera_id" << origin_camera_id;

  for(std::pair<int, cv::Point2f> n : cam_coordinate_adjustments) {
    std::string cam = "cam_" + std::to_string(n.first);
    cam_adjustments_file << cam << n.second;
  }

  cam_adjustments_file.release();
}

cv::Point2f CameraGraphCalibrator::getGlobalPos(int camera_id, cv::Point2f pos_in_camera) {

  // If the camera id exists in our graph then return it with the respective adjustments
  if(cam_coordinate_adjustments.count(camera_id) == 1)
    return pos_in_camera + cam_coordinate_adjustments.at(camera_id);

  // Otherwise return its position
  return pos_in_camera;
}

void CameraGraphCalibrator::calculateCameraCoordinateShifts() {

  // Set the origin camera to have no adjustments (since it is the origin all coordinates
  //  within its frame are not changed)
  cam_coordinate_adjustments[origin_camera_id] = cv::Point2f(0, 0);

  std::queue<std::pair<int,int>> bfs; // A queue of (parent_cam, camera_id)



  // Add all cameras that are reached by the origin camera
  for(std::pair<int, int> p : adjacency_list[origin_camera_id]) {
    bfs.push(std::make_pair(origin_camera_id, p.first));
  }

  while(!bfs.empty()) {


    int parent_cam = bfs.front().first;
    int current_cam = bfs.front().second;
    bfs.pop();

    int connecting_marker;

    // Check to ensure that we havent already done the calculations
    if(cam_coordinate_adjustments.count(current_cam) == 0) {

      for(std::pair<int, int>& n : adjacency_list[parent_cam]) {
        if(n.first == current_cam) {
          connecting_marker = n.second;
          break;
        }
      }
      // parent adjustments + pos in parent - pos in child
      cam_coordinate_adjustments[current_cam] = cam_coordinate_adjustments[parent_cam] + vertex_info[parent_cam][connecting_marker] - vertex_info[current_cam][connecting_marker];

    }

    for(std::pair<int, int> p : adjacency_list[current_cam]) {
      // zonly add neigbour vertices that havent been calculated
      // we check twice (first was above) since it is possible to have duplicates in the queue
      if(cam_coordinate_adjustments.count(p.first) == 0)
        bfs.push(std::make_pair(current_cam, p.first));
    }


  }
}
