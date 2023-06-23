#include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "I2Cdev.h"
#include <map>
#include <vector>
#include <NewPing.h>
#include <cstdlib>


inline size_t key(float i,float j) {return (size_t) i << 32 | (unsigned int) j;}

std::unordered_map<size_t, std::unordered_map<float, bool>> nodes;

struct WHash {
  size_t operator()(const weighted_node &node) const {
    return key(node.first.first, node.first.second);
  }
};
struct CoordinateHash {
  size_t operator()(const coords &coordinate) const {
    return key(coordinate.first, coordinate.second);
  }
};

typedef std::pair<float, float> coords;
typedef std::pair<float, coords> iPair;
typedef std::pair<coords, float> weighted_node;
typedef std::unordered_map<coords, std::unordered_set<weighted_node, WHash>, CoordinateHash> AdjacencyList;
AdjacencyList adjList;
AdjacencyList angle_list; //like adjlist but weight is angle
coords start;
coords last_node;
coords current_node;
coords furthest_node;
float distance;
float last_yaw;

float find_distance(coords src, coords dest){
    float distance = std::sqrt(std::pow((dest.first - src.first), 2) + std::pow((dest.second - src.second), 2));
    return distance;
}

void addEdge(AdjacencyList &adjList, coords &src, coords &dest) {  
  float weight = find_distance(src, dest);
  adjList[src].insert(std::make_pair(dest, weight));
  adjList[dest].insert(std::make_pair(src, weight));
}

void addEdge_angle(AdjacencyList &angle_list, coords &src, coords &dest){
  float angle;
  if(last_yaw < 0){
    angle = last_yaw+0.5*3,14159;
  }
  else{
    angle = last_yaw_0.5*3.14159;
  }
  adjList[src].insert(std::make_pair(dest, last_yaw));
  adjList[dest].insert(std::make_pair(src, angle));

}

std::vector<int> is_node(int x, int y, std::unordered_map<size_t, std::unordered_map<float, bool>> nodes){
    std::unordered_map<size_t, std::unordered_map<float, bool>>::iterator it;
    std::vector<int> coord(2,555);
    for (it = nodes.begin(); it!=nodes.end(); it++){
        int tmp_x = it->first>>32;
        int tmp_y = it->first - (tmp_x<<32);
        int x_diff = tmp_x-position[0];
        int y_diff = tmp_y-position[1];
        if (((x_diff>-0.2) && (x_diff<0.2)) && ((y_diff>-0.2) && (y_diff<0.2))){
            coord[0] = tmp_x;
            coord[1] = tmp_y;
            return coord;
        }
    }
    return coord;
}

void send_sp(std::vector<coords> path){
    
    DynamicJsonDocument jsonDocument(2048);
    // Construct the JSON array for the path
    JsonArray jsonPath = jsonDocument.createNestedArray("path");
    for (int i = path.size() - 1; i >= 0; --i) {
      float tmp_x= (path[i].second)*400/3.6 +10;
      float tmp_y = (3.6-path[i].first)*400/3.6;
      JsonObject coordsObj = jsonPath.createNestedObject();
      coordsObj["x"] = tmp_x;
      coordsObj["y"] = tmp_y;
    }

    // Serialize the JSON document to a string
    String output;
    serializeJson(jsonDocument, output);
    client.connect(websockets_server_host, websockets_server_port, "/");
    client.send(output);
    jsonDocument.clear();
}

std::vector<coords> shortestPath(coords src, coords dest, AdjacencyList adjList) {
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
  std::unordered_map<coords, int, CoordinateHash> dist;
  std::unordered_map<coords, coords, CoordinateHash> prev;
  for (const auto& pair : adjList) {
        dist[pair.first] = 100000;
    }
  pq.push(std::make_pair(0, src));
  dist[src] = 0;
  
  while (!pq.empty()) {
    coords u = pq.top().second;
    pq.pop();

    for (const auto& neighbor : adjList[u]) {
      // Get vertex label and weight of the current
      // neighbor of u.
      coords v = neighbor.first;
      float weight = neighbor.second;

      // If there is a shorter path to v through u.
      if (dist[v] > dist[u] + weight) {
        // Updating the distance of v
        dist[v] = dist[u] + weight;
        prev[v] = u;
        pq.push(std::make_pair(dist[v], v));
      }
    }
  }
  //printf("Shortest distance from Source to Dest: %d\n", dist[dest]);

  // Construct the path from source to destination
  std::vector<coords> path;
  coords current = dest;
  while (current != src) {
    path.push_back(current);
    current = prev[current];
  }
  path.push_back(src);
  send_sp(path);

  // Print the path
//   std::cout << "Path: ";
//   for (int i = path.size() - 1; i >= 0; --i) {
//     std::cout << "(" << path[i].first << " " << path[i].second << ")";
//     if (i != 0) {
//       std::cout << " -> ";
//     }
//   }
//   std::cout << endl;
  return path;
    

}



//start
start = std::make_pair(10, 400);

//in the loop at every node

current_node = std::make_pair(position[0], position[1]);
addEdge(adjList, last_node, current_node);
addEdge_angle(angle_list, last_node, current_node);
last_node = current_node;


distance = find_distance(start, current_node);
max_distance = find_distance(start, furthest_node);
if (distance > max_distance){
    furthest_node = current_node;
}


std::vector<coords> sp = shortestPath(start, furthest_node, adjList);

//will reach this point when all paths at this node has been visited

float find_next_angle(){
    std::vector<int> node_coord = is_node(nodes);
    coords tmp = std::make_pair(node_coord[0], node_coord[1]);
    for (const auto& neighbor :angle_list[tmp].first){
        auto it = explored_nodes.find(neighbor.first);
        if (!it.second){
            return neighbor.second;
        }
    }
}

