#include "CameraGraph.hpp"

void CameraGraph::addEdge(int fromVertex, int toVertex) {

  // Only add if no duplicates
  for(int i = 0; i < adjacency_list[fromVertex].size(); i++) {
    if(adjacency_list[fromVertex][i] == toVertex)
      return;
  }

  adjacency_list[fromVertex].push_back(toVertex);
  adjacency_list[toVertex].push_back(fromVertex);
}

void CameraGraph::addVertexInfo(int vertex, aruco::Marker m) {

  bool contains = false;

  // If it already exists replace it (ensures we have most up to date marker)
  for(int i = 0; i < vertex_info[vertex].size(); i++) {
    if(vertex_info[vertex][i].id == m.id) {
      vertex_info[vertex][i] = m;
      contains = true;
    }
  }

  // If it doesnt exist then add it in
  if(!contains)
    vertex_info[vertex].push_back(m);

  // Lastly add any edges that this new marker may create a connection to
  for(auto& v : vertex_info) {
    // Dont search through the vertex we just added the marker to
    if(v.first == vertex)
      continue;

    for(auto& marker : v.second) {
      if(marker.id == m.id)
        addEdge(v.first, vertex);
    }
  }
}

bool CameraGraph::isConnected() {

  // If the graph is empty then it cant be connected
	if(adjacency_list.size() != number_of_cameras)
		return false;

	std::set<int> visited;
	std::queue<int> bfs;

	// Enque the first element of our adjacency_list
	bfs.push(adjacency_list.begin()->first);

	// Perform a Breadth-first search to check connectedness
	while(!bfs.empty()) {
		int vertex = bfs.front();
		bfs.pop();

		// If we have visited the vertex already skip it
		if(visited.count(vertex) != 0)
			continue;

		// Otherwise add the vertex to visited
		visited.insert(vertex);

		// Enque all of the neighbours of the vertex as long as we havent visited them
		for(int n : adjacency_list[vertex]) {
			if(visited.count(n) == 0)
				bfs.push(n);
		}
	}

	// After completing the BFS check if the size of our visited set is the same
	// as the number of cameras. If it is that means it is fully connected.
	if(visited.size() == number_of_cameras)
		return true;
	else
		return false;
}

// void CameraGraph::saveGraph(std::string output_file);
//
// void CameraGraph::loadGraph(std::string input_file);
//
// Point2f CameraGraph::getAdjustedPos(int camera_id, Point2f pos_in_camera);

void CameraGraph::calculateCameraCoordinateShifts() {

}
