#include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

typedef pair<int, int> coords;
typedef pair<coords, double> weighted_node;
// Hash function using the provided key function
inline size_t key(int i, int j) { return (size_t)i << 32 | (unsigned int)j; }
// Custom hash function for Coordinate
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
// Define a data structure to represent the adjacency list
typedef unordered_map<coords, unordered_set<weighted_node, WHash>, CoordinateHash> AdjacencyList;

// Function to add an edge to the adjacency list
void addEdge(AdjacencyList &adjList, coords &src, coords &dest) {  
  double weight = sqrt(pow((dest.first - src.first), 2) + pow((dest.second - src.second), 2));
  adjList[src].insert(make_pair(dest, weight));
  adjList[dest].insert(make_pair(src, weight));
}

typedef pair<int, coords> iPair;

void shortestPath(coords src, coords dest, AdjacencyList adjList) {
  priority_queue<iPair, vector<iPair>, greater<iPair>> pq;
  unordered_map<coords, int, CoordinateHash> dist;
  unordered_map<coords, coords, CoordinateHash> prev;
  for (const auto& pair : adjList) {
        dist[pair.first] = 100000;
    }
  pq.push(make_pair(0, src));
  dist[src] = 0;
  
  while (!pq.empty()) {
    coords u = pq.top().second;
    pq.pop();

    for (const auto& neighbor : adjList[u]) {
      // Get vertex label and weight of the current
      // neighbor of u.
      coords v = neighbor.first;
      int weight = neighbor.second;

      // If there is a shorter path to v through u.
      if (dist[v] > dist[u] + weight) {
        // Updating the distance of v
        dist[v] = dist[u] + weight;
        prev[v] = u;
        pq.push(make_pair(dist[v], v));
      }
    }
  }
  printf("Shortest distance from Source to Dest: %d\n", dist[dest]);

  // Construct the path from source to destination
  vector<coords> path;
  coords current = dest;
  while (current != src) {
    path.push_back(current);
    current = prev[current];
  }
  path.push_back(src);

  // Print the path
  cout << "Path: ";
  for (int i = path.size() - 1; i >= 0; --i) {
    cout << "(" << path[i].first << " " << path[i].second << ")";
    if (i != 0) {
      cout << " -> ";
    }
  }
  cout << endl;
}



// Function to print the adjacency list
void printAdjacencyList(const AdjacencyList &adjList) {
  for (const auto &pair : adjList) {
    cout << "Node (" << pair.first.first << ", " << pair.first.second << "): ";
    for (const auto &neighbor : pair.second) {
      cout << "( (" << neighbor.first.first << ", " << neighbor.first.second
           << "), " << neighbor.second << ")";
    }
    cout << endl;
  }
}
int main() {
  AdjacencyList adjList;
  coords a = make_pair(100, 0);
  coords b = make_pair(0, 0);
  coords c = make_pair(0, 500);
  coords d = make_pair(200, 500);
  coords e = make_pair(300, 500);
  coords f = make_pair(200, 0);
  coords g = make_pair(260, 0);
  coords h = make_pair(600, 300);
  coords i = make_pair(270, 350);
  coords j = make_pair(400, 200);
  coords k = make_pair(600, 500);
  coords l = make_pair(850, 500);
  coords m = make_pair(850, 300);
  coords n = make_pair(650, 0);
  coords o = make_pair(850, 0);
  // Add edges to the adjacency list
  addEdge(adjList, a, b);
  addEdge(adjList, b, c);
  addEdge(adjList, c, d);
  addEdge(adjList, d, e);
  addEdge(adjList, e, i);
  addEdge(adjList, i, j);
  addEdge(adjList, d, f);
  addEdge(adjList, f, g);
  addEdge(adjList, g, h);
  addEdge(adjList, h, n);
  addEdge(adjList, n, o);
  addEdge(adjList, h, m);
  addEdge(adjList, m, l);
  addEdge(adjList, l, k);
  addEdge(adjList, k, h);
  addEdge(adjList, k, e);

  // Print the adjacency list
  printAdjacencyList(adjList);
  shortestPath(c, o, adjList);

  return 0;
}
typedef std::unordered_map<size_t, std::unordered_map<int, bool>> node_map;
coords is_node(int x, int y, node_map nodes){
    node_map::iterator it;
    coords coord = make_pair(555,555);
    for (it = nodes.begin(); it!=nodes.end(); it++){
        int tmp_x = it->first>>32;
        int tmp_y = ((it->first)<<32)>>32;
        int x_diff = tmp_x-x;
        int y_diff = tmp_y-y;
        if ((x_diff>-20) & (x_diff<20) & (y_diff>-20) & (x_diff<20)){
            coord = make_pair(tmp_x,tmp_y);
            return coord;
        }
    }
    return coord;
}


int main(){
    int x, y;
    bool turn;
    int camera_command;
    int direction;
    bool check_node;
    node_map nodes; //map of nodes, coordinates as key, value is
    //a map of key being angles of paths, and boolean ifVisited
    AdjacencyList adj_list;
    coords last_node = std::make_pair(0,0);

    if (check_node){//is a node
        coords pos = std::make_pair(x,y);
        if (adjList[last_node].count(pos) < 1) {
            addEdge(adj_list, pos, last_node);
        };
        
        if (is_node(x,y,nodes).firsrt == 555){ // not a defined node
            turn = true;
            int min = 360;
            int max = -180;
            bool is_path = false; //to indicate end of a path
            //need to add turning function here
            if (camera_command == 'f'){
                is_path = true;
                if (direction < min){
                    min = direction;
                }
                if (direction > max){
                    max = direction;
                }
            }
            if (camera_command == 's' & is_path){
                nodes[key(x,y)][(min+max)/2] = false;
                min = 360;
                max = -180;
            }    
        }
        else{//is a defined node
            coords tmp = is_node(x,y,nodes);
            int tmp_x = tmp.first;
            int tmp_y = tmp.second;
            std::unordered_map<int,bool> angles;
            angles = nodes[key(tmp_x, tmp_y)];
            for (auto it = angles.begin(); it != angles.end(); it++){
                if (((it->first-direction)<185 & (it->first-direction)>175) | ((it->first-direction)>-185 & (it->first-direction)<-175)){
                    it->second = true;
                }
            }
        }
        last_node = pos;
    }
    
}
