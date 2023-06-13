#include <unordered_map>
#include <vector>

inline size_t key(int i,int j) {return (size_t) i << 32 | (unsigned int) j;}

//this is for checking if current position is at a node
//returns 555 if node a node
//return x, y coordinates of the node if it is
std::vector<int> is_node(int x, int y, std::unordered_map<size_t, std::unordered_map<int, bool>> nodes){
    std::unordered_map<size_t, std::unordered_map<int, bool>>::iterator it;
    std::vector<int> coord(2,555);
    for (it = nodes.begin(); it!=nodes.end(); it++){
        int tmp_x = it->first>>32;
        int tmp_y = ((it->first)<<32)>>32;
        int x_diff = tmp_x-x;
        int y_diff = tmp_y-y;
        if ((x_diff>-20) & (x_diff<20) & (y_diff>-20) & (x_diff<20)){
            coord[0] = tmp_x;
            coord[1] = tmp_y;
            return coord;
        }
    }
    return coord;
}

//to add a position as a new node and mark visited paths
//return whether this is a new node
//if it is, turn one round and add angles to the map
// bool edit_node(int x, int y, std::unordered_map<size_t, std::unordered_map<int, bool>> &nodes ){
//     if (!is_node)
// }

int main(){
    int x, y;
    bool turn;
    int camera_command;
    int direction;
    bool check_node;
    std::unordered_map<size_t, std::unordered_map<int, bool>> nodes;
    //map of nodes, coordinates as key, value is
    //a map of key being angles of paths, and boolean ifVisited

    if (check_node){
        if (is_node(x,y,nodes)[0] == 555){ // not a defined node
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
            std::vector<int> tmp = is_node(x,y,nodes);
            int tmp_x = tmp[0];
            int tmp_y = tmp[1];
            std::unordered_map<int,bool> angles;
            angles = nodes[key(tmp_x, tmp_y)];
            for (auto it = angles.begin(); it != angles.end(); it++){
                if (((it->first-direction)<185 & (it->first-direction)>175) | ((it->first-direction)>-185 & (it->first-direction)<-175)){
                    it->second = true;
                }
            }
        }
    }
    
}
