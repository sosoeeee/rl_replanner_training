#include "map_voronoi/voronoigraph.h"
#include <unordered_map>
#include <unordered_set>

// void VoronoiGraph::visualizeVoronoi(const std::string& filename, int type) {
//     switch (type) {
//         case 0:
//             if (voronoi_static) {
//                 voronoi_static->visualize(filename.c_str());
//             }
//             else {
//                 std::cerr << "Voronoi object is not initialized." << std::endl;
//             }
//             break;
//         case 1:
//             if (voronoi_modified) {
//                 voronoi_modified->visualize(filename.c_str());
//             } else {
//                 std::cerr << "Modified Voronoi object is not initialized." << std::endl;
//             }
//             break;
//         default:
//             std::cerr << "Invalid type for Voronoi visualization." << std::endl;
//             break;
//     }
// }

// 由于没有初始图构建，可视化函数可以替换：
void VoronoiGraph::visualizeVoronoi(const std::string& filename) {
    if (voronoi_modified) {
        voronoi_modified->visualize(filename.c_str());
    } else {
        std::cerr << "Modified Voronoi object is not initialized." << std::endl;
    }
}

bool** VoronoiGraph::getBoolMap(std::shared_ptr<Costmap2D> costmap){
    int sizeX = costmap->getSizeInCellsX();
    int sizeY = costmap->getSizeInCellsY();
    bool **map = new bool*[sizeX];
    for (int x = 0; x < sizeX; ++x) {
        map[x] = new bool[sizeY];
        for (int y = 0; y < sizeY; ++y) {
            map[x][y] = (costmap->getCost(x, y) >= nav2_costmap_2d::MAX_NON_OBSTACLE);
        }
    }
    return map;
}

// TODO: Use BFS to speed up the building process
// we merge the same node during the process of finding the path 
// if the node is near the last node, then we see it as the same node
// void VoronoiGraph::buildGraph(){
//     int sizeX = voronoi_static->getSizeX();
//     int sizeY = voronoi_static->getSizeY();
//     int id=0;
//     for(int y = sizeY-1; y >=0; y--){
//         for(int x = 0; x<sizeX; x++){
//             if (voronoi_static->isVoronoiAlternative(x,y)) {
//                 int num=0;
//                 for(int nx=-1; nx<=1; nx++){
//                   for(int ny=-1; ny<=1; ny++){
//                     if (nx==0 && ny==0) continue;
//                     if(nx*ny==-1||nx*ny==1) continue;
//                     if (x+nx<0 || x+nx>=sizeX || y+ny<0 || y+ny>=sizeY) continue;
//                     if (voronoi_static->isVoronoiAlternative(x+nx,y+ny)) {
//                       num++;
//                     }
//                   }
//                 }
//                 if(num>=3){

//                     // if the node is near the last node , then we see it as the same node
//                     if(voronoi_nodes.size()!=0){
//                         VoronoiNode last_node = voronoi_nodes.back();
//                         MapPoint position=last_node.getPosition();
//                         if(abs(position.x-x)<=1 && abs(position.y-y)<=1){
//                             id--;
//                         }
//                     }
                    
//                     VoronoiNode node(id, MapPoint{x, y});  //same node 
//                     voronoi_nodes.push_back(node);
//                     id++;
//                 }
//             }
//         }
//     }

//     //find path:
//     int node_size = voronoi_nodes.size();
//     for(int i=0;i<node_size;i++){
        
//         bool** map_flag = new bool*[sizeX];
//         for (int x = 0; x < sizeX; x++) {
//             map_flag[x] = new bool[sizeY]();
//         }

//         while(true){
//             int j=i;
//             VoronoiNode node = voronoi_nodes[i];
//             int start_id = node.getId();
//             MapPoint start = node.getPosition();
//             Path center_path;
            
//             while(j!=0){
//                 VoronoiNode last_node = voronoi_nodes[j-1];
//                 int last_id = last_node.getId();
//                 if(last_id==start_id){
//                     j=j-1;
//                 }
//                 else{
//                     break;
//                 }
//             }

//             MapPoint end;
//             end.x = start.x;
//             end.y = start.y;
//             center_path.path_points.push_back(start);
//             bool flag = false;

//             map_flag[end.x][end.y]=true;

//             while(true){
//                 if(end.x+1<sizeX && map_flag[end.x+1][end.y]==false && voronoi_static->isVoronoiAlternative(end.x+1,end.y)){
//                     end.x = end.x+1;
//                     center_path.path_points.push_back(end);
//                     map_flag[end.x][end.y]=true;
//                 }else if(end.y+1<sizeY && map_flag[end.x][end.y+1]==false && voronoi_static->isVoronoiAlternative(end.x,end.y+1)){
//                     end.y = end.y+1;
//                     center_path.path_points.push_back(end);
//                     map_flag[end.x][end.y]=true;
//                 }else if(end.x-1>=0 && map_flag[end.x-1][end.y]==false && voronoi_static->isVoronoiAlternative(end.x-1,end.y)){
//                     end.x = end.x-1;
//                     center_path.path_points.push_back(end);
//                     map_flag[end.x][end.y]=true;
//                 }else if(end.y-1>=0 && map_flag[end.x][end.y-1]==false && voronoi_static->isVoronoiAlternative(end.x,end.y-1)){
//                     end.y = end.y-1;
//                     center_path.path_points.push_back(end);
//                     map_flag[end.x][end.y]=true;
//                 }else{
//                     flag=true;
//                     break;
//                 }
//                 bool is_end = false;
//                 for(int k=0;k<node_size;k++){
//                     VoronoiNode node2 = voronoi_nodes[k];
//                     MapPoint start2 = node2.getPosition();
//                     int target_id = node2.getId();
//                     if(end.x==start2.x && end.y==start2.y){
//                         map_flag[end.x][end.y]=false;  // 两个节点间有两个路径的临时解决办法
//                         // 检查target_id是否已经是start_id的邻接点
//                         std::vector<std::pair<int, float>> adjacent = voronoi_nodes[j].getAllAdjacent();
//                         for(const auto& adj : adjacent) {
//                             if(adj.first == target_id) {
//                                 is_end = true;
//                                 break;
//                             }
//                         }
//                         if(is_end) break;

//                         if(start_id==target_id){
//                             map_flag[end.x][end.y]=true;   // 防止重复访问要被删除的节点
//                             is_end = true;
//                             break;
//                         }
//                         if(k!=0){
//                             VoronoiNode node3 = voronoi_nodes[k-1];
//                             int last_id = node3.getId();
//                             MapPoint last_position = node3.getPosition();
//                             if(target_id==last_id){
//                                 center_path.end_node_id = last_id;
//                                 center_path.path_points.push_back(last_position);
//                                 voronoi_nodes[j].addAdjacent(last_id);
//                                 voronoi_nodes[j].addPath(center_path);
//                                 is_end = true;
//                                 break;
//                             }

//                         }
//                         center_path.end_node_id = target_id;
//                         voronoi_nodes[j].addAdjacent(target_id);
//                         voronoi_nodes[j].addPath(center_path);
//                         is_end = true;
//                         break;
//                     }
//                 }
//                 if(is_end==true){
//                     break;
//                 }
//             }
//             if(flag==true){
//                 break;
//             }
//         }
//         for (int x = 0; x < sizeX; x++) {
//             delete[] map_flag[x];
//         }
//         delete[] map_flag;
//     }
//     for(int i=voronoi_nodes.size()-1;i>=0;i--){
//         VoronoiNode node = voronoi_nodes[i];
//         std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
//         if(adjacent.size()==0){
//             voronoi_nodes.erase(voronoi_nodes.begin()+i);
//         }
//     }
//     //TEST
//     // int num=voronoi_nodes.size();
//     // for(int i=0;i<num;i++)
//     // {
//     //     std::cout<<voronoi_nodes[i].getId()<<std::endl;
//     //     std::cout<<"Position: ("<<voronoi_nodes[i].getPosition().x<<","<<voronoi_nodes[i].getPosition().y<<")"<<std::endl;
//     //     std::vector<std::pair<int, float>> adjacent = voronoi_nodes[i].getAllAdjacent();
//     //     for (const auto& pair : adjacent) {
//     //         std::cout << "Adjacent Node ID: " << pair.first << ", Probability: " << pair.second << std::endl;
//     //     }
//     //     std::cout << std::endl;
//     // }
//     //TEST
// }

void VoronoiGraph::resetAllProbabilities()
{
    for (auto& node : voronoi_nodes_modified) {
        node.resetProbability();
    }
}

std::vector<int> VoronoiGraph::getPassbyNodes(int start_id, int end_id)
{   
    resetAllProbabilities();
    bool activated;

    std::vector<int> passby_nodes;  

    // add start node
    passby_nodes.push_back(start_id);

    if (start_id < 0 || start_id >= voronoi_nodes_modified.size()) throw std::out_of_range("Invalid start_id");
    std::vector<std::pair<int, float>> adjacent_nodes = voronoi_nodes_modified[start_id].getAllAdjacent();
    
    // debug
    LOGGER_INFO("VoronoiGraph", "Start node ID: %d. Its adjacent nodes are:", start_id);
    for (const auto& pair : adjacent_nodes) {
        LOGGER_INFO("VoronoiGraph", "Adjacent Node ID: %d, Probability: %f", pair.first, pair.second);
    }

    for (const auto& pair : adjacent_nodes) {
        activated = voronoi_nodes_modified[pair.first].deactivate(start_id);
        if (!activated && pair.first != end_id) {
            activated = voronoi_nodes_modified[start_id].deactivate(pair.first);
            if (!activated) 
            {
                LOGGER_ERROR("VoronoiGraph", "Node %d has no adjacent nodes. That should not happen.", start_id);
                assert(false);                
            }
        }
    }

    int next_id = voronoi_nodes_modified[start_id].getAdjacent();
    int backtrack_id = -1;
    while (next_id != end_id) {
        passby_nodes.push_back(next_id);
        adjacent_nodes = voronoi_nodes_modified[next_id].getAllAdjacent();

        // debug
        LOGGER_INFO("VoronoiGraph", "Next node ID: %d. Its adjacent nodes are:", next_id);
        for (const auto& pair : adjacent_nodes) {
            LOGGER_INFO("VoronoiGraph", "Adjacent Node ID: %d, Probability: %f", pair.first, pair.second);
        }

        // deactivate the adjacent nodes
        bool backtracking = false;
        for (const auto& pair : adjacent_nodes) {
            activated = voronoi_nodes_modified[pair.first].deactivate(next_id);
            if (!activated && pair.first != end_id) {
                activated = voronoi_nodes_modified[next_id].deactivate(pair.first);
                if (!activated) 
                {
                    // LOGGER_WARN("VoronoiGraph", "Node %d has no adjacent nodes. Start backtracking.", next_id);
                    backtracking = true;
                }
            }
        }

        //backtrack
        if (backtracking)
        {
            do {
                backtrack_id = next_id;
                passby_nodes.pop_back();
                next_id = passby_nodes.back();

                adjacent_nodes = voronoi_nodes_modified[backtrack_id].getAllAdjacent();
                for (const auto& pair : adjacent_nodes) {
                    if (pair.first != next_id) {
                    activated = voronoi_nodes_modified[pair.first].activate(backtrack_id);
                    }
                }

                // debug
                LOGGER_INFO("VoronoiGraph", "Backtracking to node ID: %d. Its adjacent nodes are:", next_id);
                adjacent_nodes = voronoi_nodes_modified[next_id].getAllAdjacent();
                for (const auto& pair : adjacent_nodes) {
                    LOGGER_INFO("VoronoiGraph", "Adjacent Node ID: %d, Probability: %f", pair.first, pair.second);
                }

            } while (voronoi_nodes_modified[next_id].hasAdjacent() == false);
        }

        next_id = voronoi_nodes_modified[next_id].getAdjacent();
    }

    passby_nodes.push_back(end_id);

    return passby_nodes;
}

std::vector<std::vector<int>> VoronoiGraph::findAllPaths(int start_id, int end_id) {
    std::vector<std::vector<int>> all_paths;
    std::vector<int> path;
    std::vector<bool> visited(voronoi_nodes_modified.size(), false);
    
    std::function<void(int)> dfs = [&](int current_id) {
        visited[current_id] = true;
        path.push_back(current_id);
        
        if (current_id == end_id) {
            all_paths.push_back(path);
        } else {
            for (const auto& neighbor : voronoi_nodes_modified[current_id].getNeighbors()) {
                if (!visited[neighbor]) {
                    dfs(neighbor);
                }
            }
        }
        
        path.pop_back();
        visited[current_id] = false;
    };
    
    dfs(start_id);

    // TEST: print all paths
    std::cout << all_paths.size() << std::endl;
    // for (const auto& path : all_paths) {
    //     for (const auto& node_id : path) {
    //         std::cout << node_id << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // Log each path
    // for (const auto& path : all_paths) {
    //     std::string path_str = "Path: ";
    //     for (const auto& node_id : path) {
    //         path_str += std::to_string(node_id) + " ";
    //     }
    //     LOGGER_INFO("VoronoiGraph", "%s", path_str.c_str());
    // }
    // TEST: end
    return all_paths;
}


void VoronoiGraph::getVoronoiGraph(unsigned int start_mx, unsigned int start_my, unsigned int end_mx, unsigned int end_my)
{   
    /*
    // 1. modify the bool map to generate the modified voronoi
    bool ** bool_map = getBoolMap(costmap);
    bool_map[start_mx][start_my] = false; // set start point as free
    bool_map[end_mx][end_my] = false; // set end point as free
    voronoi_modified->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), bool_map);
    voronoi_modified->update();
    voronoi_modified->updateAlternativePrunedDiagram();  // prune the Voronoi
    //release the bool map
    for (int x = 0; x < costmap->getSizeInCellsX(); x++) {
        delete[] bool_map[x];
    }
    delete[] bool_map;

    // 2. build the graph
    // TODO: change the buildGraph function, we can select buildGraph according to the static voronoi or modified voronoi
    buildGraph(); 

    // 3. floodfill to get the start and end related voronoi node

    // 4. add and remove related path
    */

    /*=======================================================================================================*/

    LOGGER_INFO("VoronoiGraph", "Start point: (%d, %d), End point: (%d, %d)", start_mx, start_my, end_mx, end_my);

    // Old Version
    // step1：获得添加起点和终点后的Voronoi图
    bool ** bool_map = getBoolMap(costmap);
    bool_map[start_mx][start_my] = true;
    bool_map[end_mx][end_my] = true;
    voronoi_modified->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), bool_map);
    voronoi_modified->update();
    voronoi_modified->updateAlternativePrunedDiagram();  // prune the Voronoi

    // // test 1:
    // LOGGER_INFO("VoronoiGraph", "step 1: Get start and end neighbors completed.");


    // step2: get the modified voronoi nodes
    voronoi_nodes_modified.clear();

    // 从voronoi_modified中获取基本节点
    int sizeX = voronoi_modified->getSizeX();
    int sizeY = voronoi_modified->getSizeY();
    int id = 0;
    for(int y = sizeY-1; y >= 0; y--) {
        for(int x = 0; x < sizeX; x++) {
            if (voronoi_modified->isVoronoiAlternative(x,y)) {
                int num = 0;
                for(int nx = -1; nx <= 1; nx++) {
                    for(int ny = -1; ny <= 1; ny++) {
                        if (nx == 0 && ny == 0) continue;
                        if(nx*ny == -1 || nx*ny == 1) continue;
                        if (x+nx < 0 || x+nx >= sizeX || y+ny < 0 || y+ny >= sizeY) continue;
                        if (voronoi_modified->isVoronoiAlternative(x+nx,y+ny)) {
                            num++;
                        }
                    }
                }
                if(num >= 3) {
                                        
                    // if the node is near the last node , then we see it as the same node
                    if(voronoi_nodes_modified.size()!=0){
                        VoronoiNode last_node = voronoi_nodes_modified.back();
                        MapPoint position=last_node.getPosition();
                        if(abs(position.x-x)<=1 && abs(position.y-y)<=1){
                            id--;
                        }
                    }

                    VoronoiNode node(id, MapPoint{x, y});
                    voronoi_nodes_modified.push_back(node);
                    id++;
                }
            }
        }
    }

    // // test 2:
    // LOGGER_INFO("VoronoiGraph", "step 2: Get modified voronoi nodes completed. Total nodes: %zu", voronoi_nodes_modified.size());
    // LOGGER_INFO("VoronoiGraph", "Testing voronoi_nodes_modified size: %zu", voronoi_nodes_modified.size());
    // LOGGER_INFO("VoronoiGraph", "All nodes in voronoi_nodes_modified:");
    // for (const auto& node : voronoi_nodes_modified) {
    //     LOGGER_INFO("VoronoiGraph", "Node ID: %d, Position: (%d, %d)", 
    //         node.getId(), node.getPosition().x, node.getPosition().y);
        
    //     // 输出该节点的所有邻接点信息
    //     std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
    //     LOGGER_INFO("VoronoiGraph", "  Adjacent nodes (%zu):", adjacent.size());
    //     for (const auto& adj : adjacent) {
    //         LOGGER_INFO("VoronoiGraph", "    Node ID: %d, Probability: %.3f", adj.first, adj.second);
    //     }
    // }
//  


    // step3: 为其他节点建立连接关系
    int node_size = voronoi_nodes_modified.size();
    // 对每个节点初始化一个flag：
    for(int i=0;i<node_size;i++){
        bool** map_flag = new bool*[sizeX];
        for (int x = 0; x < sizeX; x++) {
            map_flag[x] = new bool[sizeY]();
        }

        while(true){
            int j=i;
            VoronoiNode node = voronoi_nodes_modified[j];
            int start_id = node.getId();
            MapPoint start = node.getPosition();
            Path center_path;
            
            // if the node is near the last node , then we see it as the same node
            while(j!=0){
                VoronoiNode last_node = voronoi_nodes_modified[j-1];
                int last_id = last_node.getId();
                if(last_id==start_id){
                    j=j-1;
                }
                else{
                    break;
                }
            }

            MapPoint end;
            end.x = start.x;
            end.y = start.y;
            center_path.path_points.push_back(start);
            bool flag = false;

            map_flag[end.x][end.y]=true;

            while(true){
                if(end.x+1<sizeX && map_flag[end.x+1][end.y]==false && voronoi_modified->isVoronoiAlternative(end.x+1,end.y)){
                    end.x = end.x+1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.y+1<sizeY && map_flag[end.x][end.y+1]==false && voronoi_modified->isVoronoiAlternative(end.x,end.y+1)){
                    end.y = end.y+1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.x-1>=0 && map_flag[end.x-1][end.y]==false && voronoi_modified->isVoronoiAlternative(end.x-1,end.y)){
                    end.x = end.x-1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.y-1>=0 && map_flag[end.x][end.y-1]==false && voronoi_modified->isVoronoiAlternative(end.x,end.y-1)){
                    end.y = end.y-1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else{
                    flag=true;
                    break;
                }
                bool is_end = false;
                for(int k=0;k<node_size;k++){
                    VoronoiNode node2 = voronoi_nodes_modified[k];
                    MapPoint start2 = node2.getPosition();
                    int target_id = node2.getId();
                    if(end.x==start2.x && end.y==start2.y){
                        map_flag[end.x][end.y]=false;  // 两个节点间有两个路径的临时解决办法
                        // 检查target_id是否已经是start_id的邻接点
                        std::vector<std::pair<int, float>> adjacent = voronoi_nodes_modified[j].getAllAdjacent();
                        for(const auto& adj : adjacent) {
                            if(adj.first == target_id) {
                                is_end = true;
                                break;
                            }
                        }
                        if(is_end) break;

                        if(start_id==target_id){
                            map_flag[end.x][end.y]=true;   // 防止重复访问要被删除的节点
                            is_end = true;
                            break;
                        }
                        if(k!=0){
                            VoronoiNode node3 = voronoi_nodes_modified[k-1];
                            int last_id = node3.getId();
                            MapPoint last_position = node3.getPosition();
                            if(target_id==last_id){
                                center_path.end_node_id = last_id;
                                center_path.path_points.push_back(last_position);
                                voronoi_nodes_modified[j].addAdjacent(last_id);
                                voronoi_nodes_modified[j].addPath(center_path);
                                is_end = true;
                                break;
                            }

                        }
                        center_path.end_node_id = target_id;
                        voronoi_nodes_modified[j].addAdjacent(target_id);
                        voronoi_nodes_modified[j].addPath(center_path);
                        is_end = true;
                        break;
                    }
                }
                if(is_end==true){
                    break;
                }
            }
            if(flag==true){
                break;
            }
        }
        for (int x = 0; x < sizeX; x++) {
            delete[] map_flag[x];
        }
        delete[] map_flag;
    }

    // // test 3:
    // LOGGER_INFO("VoronoiGraph", "step 3: Build connections between modified voronoi nodes completed. Total nodes: %zu", voronoi_nodes_modified.size());

// // 
// LOGGER_INFO("VoronoiGraph", "Testing voronoi_nodes_modified size: %zu", voronoi_nodes_modified.size());
// LOGGER_INFO("VoronoiGraph", "All nodes in voronoi_nodes_modified:");
// for (const auto& node : voronoi_nodes_modified) {
//     LOGGER_INFO("VoronoiGraph", "Node ID: %d, Position: (%d, %d)", 
//         node.getId(), node.getPosition().x, node.getPosition().y);
    
//     // 输出该节点的所有邻接点信息
//     std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
//     LOGGER_INFO("VoronoiGraph", "  Adjacent nodes (%zu):", adjacent.size());
//     for (const auto& adj : adjacent) {
//         LOGGER_INFO("VoronoiGraph", "    Node ID: %d, Probability: %.3f", adj.first, adj.second);
//     }
// }
// //  

    // 删除重复id的节点：
    for(int i=voronoi_nodes_modified.size()-1;i>0;i--){
        VoronoiNode& node = voronoi_nodes_modified[i];
        int id = node.getId();
        int j=i-1;
        int last_id = voronoi_nodes_modified[j].getId();
        if(id==last_id){
            voronoi_nodes_modified.erase(voronoi_nodes_modified.begin()+i);
        }
    }

    // // test 4:
    // LOGGER_INFO("VoronoiGraph", "step 4: Remove nodes without adjacent nodes completed. Remaining nodes: %zu", voronoi_nodes_modified.size());

    // step4: 获取起点和终点的邻接节点
    voronoi_nodes_startNeighbor.clear();
    bool** visited = new bool*[sizeX];
    for (int x = 0; x < sizeX; x++) {
        visited[x] = new bool[sizeY]();
    }

    std::queue<MapPoint> q;
    q.push(MapPoint{static_cast<int>(start_mx), static_cast<int>(start_my)});
    visited[start_mx][start_my] = true;

    while (!q.empty()) {
        MapPoint current = q.front();
        q.pop();
        for(int nx = -1; nx <= 1; nx++) {
            for(int ny = -1; ny <= 1; ny++) {
                if (nx == 0 && ny == 0) continue;
                if(nx*ny == -1 || nx*ny == 1) continue;
                
                int new_x = current.x + nx;
                int new_y = current.y + ny;
                
                if (new_x < 0 || new_x >= sizeX || new_y < 0 || new_y >= sizeY) continue;
                if (visited[new_x][new_y]) continue;
                
                if (!voronoi_modified->isVoronoiAlternative(new_x, new_y)) {
                    q.push(MapPoint{new_x, new_y});
                    visited[new_x][new_y] = true;
                }
                else{
                    // 找到Voronoi点，检查是否是节点
                    for(const auto& node : voronoi_nodes_modified) {
                        if (node.getPosition().x == new_x && node.getPosition().y == new_y) {
                            voronoi_nodes_startNeighbor.push_back(node);
                            break; // 找到匹配的节点后跳出循环
                        }
                    }
                }
            }
        }
    }

    // 清理内存
    for (int x = 0; x < sizeX; x++) {
        delete[] visited[x];
    }
    delete[] visited;
    
    LOGGER_INFO("VoronoiGraph", "Found %zu start neighbor nodes by diffusion", voronoi_nodes_startNeighbor.size());

    // 获取终点的邻接节点
    voronoi_nodes_endNeighbor.clear();

    visited = new bool*[sizeX];
    for (int x = 0; x < sizeX; x++) {
        visited[x] = new bool[sizeY]();
    }

    // std::queue<MapPoint> q;
    q.push(MapPoint{static_cast<int>(end_mx), static_cast<int>(end_my)});
    visited[end_mx][end_my] = true;

    while (!q.empty()) {
        MapPoint current = q.front();
        q.pop();
        for(int nx = -1; nx <= 1; nx++) {
            for(int ny = -1; ny <= 1; ny++) {
                if (nx == 0 && ny == 0) continue;
                if(nx*ny == -1 || nx*ny == 1) continue;
                
                int new_x = current.x + nx;
                int new_y = current.y + ny;
                
                if (new_x < 0 || new_x >= sizeX || new_y < 0 || new_y >= sizeY) continue;
                if (visited[new_x][new_y]) continue;
                
                if (!voronoi_modified->isVoronoiAlternative(new_x, new_y)) {
                    q.push(MapPoint{new_x, new_y});
                    visited[new_x][new_y] = true;
                }
                else{
                    // 找到Voronoi点，检查是否是节点
                    for(const auto& node : voronoi_nodes_modified) {
                        if (node.getPosition().x == new_x && node.getPosition().y == new_y) {
                            voronoi_nodes_endNeighbor.push_back(node);
                            break; // 找到匹配的节点后跳出循环
                        }
                    }
                }
            }
        }
    }

    // 清理内存
    for (int x = 0; x < sizeX; x++) {
        delete[] visited[x];
    }
    delete[] visited;
    
    LOGGER_INFO("VoronoiGraph", "Found %zu end neighbor nodes by diffusion", voronoi_nodes_endNeighbor.size());

    // step5: 添加起点作为节点
    start_point_node_id = id++;
    VoronoiNode start_node(start_point_node_id, MapPoint{static_cast<int>(start_mx), static_cast<int>(start_my)});

    // 为起点添加邻节点和路径
    for (const auto& neighbor : voronoi_nodes_startNeighbor) {
        // 添加邻接关系
        start_node.addAdjacent(neighbor.getId());
        voronoi_nodes_modified[neighbor.getId()].addAdjacent(start_point_node_id);
        
        // 创建从起点到邻节点的直线路径
        Path path;
        // 使用Bresenham算法生成直线路径点
        std::vector<MapPoint> line_points;
        int x1 = start_node.getPosition().x, y1 = start_node.getPosition().y;
        int x2 = neighbor.getPosition().x, y2 = neighbor.getPosition().y;
        
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            line_points.push_back(MapPoint{x1, y1});
            if (x1 == x2 && y1 == y2) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
        path.path_points = line_points;
        path.end_node_id = neighbor.getId();
        start_node.addPath(path);
        
        // 创建从邻节点到起点的反向路径
        Path reverse_path;
        reverse_path.path_points = line_points;
        std::reverse(reverse_path.path_points.begin(), reverse_path.path_points.end());
        reverse_path.end_node_id = start_point_node_id;
        voronoi_nodes_modified[neighbor.getId()].addPath(reverse_path);
        
        // 打印路径信息
        LOGGER_INFO("VoronoiGraph", "Added bidirectional path between start node (ID: %d) and neighbor (ID: %d):", 
            start_point_node_id, neighbor.getId());
        LOGGER_INFO("VoronoiGraph", "  Path points: (%d,%d) <-> (%d,%d)", 
            start_node.getPosition().x, start_node.getPosition().y,
            neighbor.getPosition().x, neighbor.getPosition().y);
    }

    // 添加终点作为节点
    end_point_node_id = id++;
    VoronoiNode end_node(end_point_node_id, MapPoint{static_cast<int>(end_mx), static_cast<int>(end_my)});

    // 为终点添加邻节点和路径
    for (const auto& neighbor : voronoi_nodes_endNeighbor) {
        // 添加邻接关系
        end_node.addAdjacent(neighbor.getId());
        voronoi_nodes_modified[neighbor.getId()].addAdjacent(end_point_node_id);
        
        // 创建从终点到邻节点的直线路径
        Path path;
        // 使用Bresenham算法生成直线路径点
        std::vector<MapPoint> line_points;
        int x1 = end_node.getPosition().x, y1 = end_node.getPosition().y;
        int x2 = neighbor.getPosition().x, y2 = neighbor.getPosition().y;
        
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            line_points.push_back(MapPoint{x1, y1});
            if (x1 == x2 && y1 == y2) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
        path.path_points = line_points;
        path.end_node_id = neighbor.getId();
        end_node.addPath(path);
        
        // 创建从邻节点到终点的反向路径
        Path reverse_path;
        reverse_path.path_points = line_points;
        std::reverse(reverse_path.path_points.begin(), reverse_path.path_points.end());
        reverse_path.end_node_id = end_point_node_id;
        voronoi_nodes_modified[neighbor.getId()].addPath(reverse_path);
        
        // 打印路径信息
        LOGGER_INFO("VoronoiGraph", "Added bidirectional path between end node (ID: %d) and neighbor (ID: %d):", 
            end_point_node_id, neighbor.getId());
        LOGGER_INFO("VoronoiGraph", "  Path points: (%d,%d) <-> (%d,%d)", 
            end_node.getPosition().x, end_node.getPosition().y,
            neighbor.getPosition().x, neighbor.getPosition().y);
    }

    voronoi_nodes_modified.push_back(start_node);
    voronoi_nodes_modified.push_back(end_node);

    // 删除voronoi_nodes_startNeighbor之间的相邻关系
    LOGGER_INFO("VoronoiGraph", "Removing connections between start neighbor nodes...");
    for (const auto& node : voronoi_nodes_startNeighbor) {
        int node_id = node.getId();
        if (node_id >= voronoi_nodes_modified.size()) {
            LOGGER_WARN("VoronoiGraph", "Invalid node ID %d in start neighbors", node_id);
            continue;
        }
        
        std::vector<std::pair<int, float>> adjacent = voronoi_nodes_modified[node_id].getAllAdjacent();
        for (const auto& adj : adjacent) {
            // 检查相邻节点是否也在voronoi_nodes_startNeighbor中
            bool is_in_start_neighbor = false;
            for (const auto& start_node : voronoi_nodes_startNeighbor) {
                if (adj.first == start_node.getId()) {
                    is_in_start_neighbor = true;
                    break;
                }
            }
            // 如果在startNeighbor中，删除双向连接
            if (is_in_start_neighbor && adj.first < voronoi_nodes_modified.size()) {
                // 从当前节点删除连接
                voronoi_nodes_modified[node_id].removeAdjacent(adj.first);
                // 从目标节点删除连接
                voronoi_nodes_modified[adj.first].removeAdjacent(node_id);
                LOGGER_INFO("VoronoiGraph", "Removed bidirectional connection between start neighbors: %d <-> %d", 
                    node_id, adj.first);
            }
        }
    }

    // 删除voronoi_nodes_endNeighbor之间的相邻关系
    LOGGER_INFO("VoronoiGraph", "Removing connections between end neighbor nodes...");
    for (const auto& node : voronoi_nodes_endNeighbor) {
        int node_id = node.getId();
        if (node_id >= voronoi_nodes_modified.size()) {
            LOGGER_WARN("VoronoiGraph", "Invalid node ID %d in end neighbors", node_id);
            continue;
        }
        
        std::vector<std::pair<int, float>> adjacent = voronoi_nodes_modified[node_id].getAllAdjacent();
        for (const auto& adj : adjacent) {
            // 检查相邻节点是否也在voronoi_nodes_endNeighbor中
            bool is_in_end_neighbor = false;
            for (const auto& end_node : voronoi_nodes_endNeighbor) {
                if (adj.first == end_node.getId()) {
                    is_in_end_neighbor = true;
                    break;
                }
            }
            // 如果在endNeighbor中，删除双向连接
            if (is_in_end_neighbor && adj.first < voronoi_nodes_modified.size()) {
                // 从当前节点删除连接
                voronoi_nodes_modified[node_id].removeAdjacent(adj.first);
                // 从目标节点删除连接
                voronoi_nodes_modified[adj.first].removeAdjacent(node_id);
                LOGGER_INFO("VoronoiGraph", "Removed bidirectional connection between end neighbors: %d <-> %d", 
                    node_id, adj.first);
            }
        }
    }

    for (auto& node : voronoi_nodes_modified) {
        int node_id = node.getId();
        std::vector<std::pair<int, float>> adjacents = node.getAllAdjacent();
        std::vector<int> to_remove;
        for (const auto& adj : adjacents) {
            int neighbor_id = adj.first;
            auto it = std::find_if(voronoi_nodes_modified.begin(), voronoi_nodes_modified.end(),
                [neighbor_id](const VoronoiNode& n){ return n.getId() == neighbor_id; });
            if (it == voronoi_nodes_modified.end()) {
                to_remove.push_back(neighbor_id);
                continue;
            }
            bool found = false;
            for (const auto& back : it->getAllAdjacent()) {
                if (back.first == node_id) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                to_remove.push_back(neighbor_id);
            }
        }
        for (int nid : to_remove) {
            node.removeAdjacent(nid);
            // LOGGER_INFO("VoronoiGraph", "Removed asymmetric adjacency: %d -> %d", node_id, nid);
        }
    }

    // 打印起点和终点节点的ID
    // LOGGER_INFO("VoronoiGraph", "Start point node ID: %d", start_point_node_id);
    // LOGGER_INFO("VoronoiGraph", "End point node ID: %d", end_point_node_id);
    
    // 在函数结束前打印所有节点的邻接点信息
    // LOGGER_INFO("VoronoiGraph", "All nodes in modified graph (%zu nodes):", voronoi_nodes_modified.size());
    // for (const auto& node : voronoi_nodes_modified) {
    //     LOGGER_INFO("VoronoiGraph", "Node ID: %d, Position: (%d, %d)", 
    //         node.getId(), node.getPosition().x, node.getPosition().y);
        
    //     // 打印该节点的所有邻接点
    //     std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
    //     LOGGER_INFO("VoronoiGraph", "  Adjacent nodes (%zu):", adjacent.size());
    //     for (const auto& adj : adjacent) {
    //         LOGGER_INFO("VoronoiGraph", "    Node ID: %d, Probability: %.3f", adj.first, adj.second);
    //     }
    // }

    // 在函数结束前调用打印最小距离的函数
    // printMinDistOnAllPathsInModifiedGraph();
    // 在函数结束前打印节点数量：
    LOGGER_INFO("VoronoiGraph", "Modified Voronoi graph built with %zu nodes.", voronoi_nodes_modified.size());
}

// void VoronoiGraph::printMinDistOnAllPathsInModifiedGraph() {
//     LOGGER_INFO("VoronoiGraph", "==== Min Distance to Obstacle on All Paths (Modified Graph) ====");
//     for (const auto& node : voronoi_nodes_modified) {
//         int from_id = node.getId();
//         const auto& adjacents = node.getAllAdjacent();
//         for (const auto& adj : adjacents) {
//             int to_id = adj.first;
//             // 获取路径点
//             const auto& path_points = node.getPathById(to_id);
//             float min_dist = std::numeric_limits<float>::max();
//             for (const auto& pt : path_points) {
//                 float dist = voronoi_static->getDistance(pt.x, pt.y);
//                 if (dist < min_dist) min_dist = dist;
//             }
//             LOGGER_INFO("VoronoiGraph", "From Node %d to Node %d: MinDist = %.3f (PathLen=%zu)",
//                 from_id, to_id, min_dist, path_points.size());
//         }
//     }
// }

void VoronoiGraph::pruneEdgesByObstacleClearance(float map_resolution, float robot_radius) {
    LOGGER_INFO("VoronoiGraph", "==== Pruning edges with clearance < robot radius ====");
    for (auto& node : voronoi_nodes_modified) {
        int from_id = node.getId();
        std::vector<std::pair<int, float>> adjacents = node.getAllAdjacent();
        std::vector<int> to_remove;
        for (const auto& adj : adjacents) {
            int to_id = adj.first;
            const auto& path_points = node.getPathById(to_id);
            float min_dist = std::numeric_limits<float>::max();
            for (const auto& pt : path_points) {
                float dist = voronoi_static->getDistance(pt.x, pt.y) * map_resolution;
                if (dist < min_dist) min_dist = dist;
            }
            if (min_dist < (robot_radius*2)) {
                to_remove.push_back(to_id);
                LOGGER_INFO("VoronoiGraph", "Prune edge: Node %d -> Node %d, MinDist = %.3f < RobotRadius = %.3f", from_id, to_id, min_dist, robot_radius);
            }
        }
        for (int nid : to_remove) {
            node.removeAdjacent(nid);
        }
    }
}