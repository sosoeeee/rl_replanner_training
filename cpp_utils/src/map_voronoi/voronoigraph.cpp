#include "map_voronoi/voronoigraph.h"

void VoronoiGraph::visualizeVoronoi(const std::string& filename, int type) {
    switch (type) {
        case 0:
            if (voronoi_static) {
                voronoi_static->visualize(filename.c_str());
            }
            else {
                std::cerr << "Voronoi object is not initialized." << std::endl;
            }
            break;
        case 1:
            if (voronoi_modified) {
                voronoi_modified->visualize(filename.c_str());
            } else {
                std::cerr << "Modified Voronoi object is not initialized." << std::endl;
            }
            break;
        default:
            std::cerr << "Invalid type for Voronoi visualization." << std::endl;
            break;
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
void VoronoiGraph::buildGraph(){
    int sizeX = voronoi_static->getSizeX();
    int sizeY = voronoi_static->getSizeY();
    int id=0;
    for(int y = sizeY-1; y >=0; y--){
        for(int x = 0; x<sizeX; x++){
            if (voronoi_static->isVoronoiAlternative(x,y)) {
                int num=0;
                for(int nx=-1; nx<=1; nx++){
                  for(int ny=-1; ny<=1; ny++){
                    if (nx==0 && ny==0) continue;
                    if(nx*ny==-1||nx*ny==1) continue;
                    if (x+nx<0 || x+nx>=sizeX || y+ny<0 || y+ny>=sizeY) continue;
                    if (voronoi_static->isVoronoiAlternative(x+nx,y+ny)) {
                      num++;
                    }
                  }
                }
                if(num>=3){

                    // if the node is near the last node , then we see it as the same node
                    if(voronoi_nodes.size()!=0){
                        VoronoiNode last_node = voronoi_nodes.back();
                        MapPoint position=last_node.getPosition();
                        if(abs(position.x-x)<=1 && abs(position.y-y)<=1){
                            id--;
                        }
                    }
                    
                    VoronoiNode node(id, MapPoint{x, y});  //same node 
                    voronoi_nodes.push_back(node);
                    id++;
                }
            }
        }
    }

    //find path:
    int node_size = voronoi_nodes.size();
    for(int i=0;i<node_size;i++){
        
        bool** map_flag = new bool*[sizeX];
        for (int x = 0; x < sizeX; x++) {
            map_flag[x] = new bool[sizeY]();
        }

        while(true){
            int j=i;
            VoronoiNode node = voronoi_nodes[i];
            int start_id = node.getId();
            MapPoint start = node.getPosition();
            Path center_path;
            
            // if the node is near the last node , then we see it as the same node
            if(i!=0){
                VoronoiNode last_node = voronoi_nodes[i-1];
                MapPoint last_position = last_node.getPosition();
                if(abs(last_position.x-start.x)<=1 && abs(last_position.y-start.y)<=1){
                    center_path.path_points.push_back(last_position);
                    j=i-1;
                }
            }

            MapPoint end;
            end.x = start.x;
            end.y = start.y;
            center_path.path_points.push_back(start);
            bool flag = false;

            map_flag[end.x][end.y]=true;

            while(true){
                if(end.x+1<sizeX && map_flag[end.x+1][end.y]==false && voronoi_static->isVoronoiAlternative(end.x+1,end.y)){
                    end.x = end.x+1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.y+1<sizeY && map_flag[end.x][end.y+1]==false && voronoi_static->isVoronoiAlternative(end.x,end.y+1)){
                    end.y = end.y+1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.x-1>=0 && map_flag[end.x-1][end.y]==false && voronoi_static->isVoronoiAlternative(end.x-1,end.y)){
                    end.x = end.x-1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.y-1>=0 && map_flag[end.x][end.y-1]==false && voronoi_static->isVoronoiAlternative(end.x,end.y-1)){
                    end.y = end.y-1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else{
                    flag=true;
                    break;
                }
                bool is_end = false;
                for(int k=0;k<node_size;k++){
                    VoronoiNode node2 = voronoi_nodes[k];
                    MapPoint start2 = node2.getPosition();
                    int target_id = node2.getId();
                    if(end.x==start2.x && end.y==start2.y){
                        if(start_id==target_id){
                            is_end = true;
                            break;
                        }
                        if(k!=0){
                            VoronoiNode node3 = voronoi_nodes[k-1];
                            int last_id = node3.getId();
                            MapPoint last_position = node3.getPosition();
                            if(target_id==last_id){
                                center_path.end_node_id = last_id;
                                center_path.path_points.push_back(last_position);
                                voronoi_nodes[j].addAdjacent(last_id);
                                voronoi_nodes[j].addPath(center_path);
                                is_end = true;
                                break;
                            }

                        }
                        center_path.end_node_id = target_id;
                        voronoi_nodes[j].addAdjacent(target_id);
                        voronoi_nodes[j].addPath(center_path);
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
    for(int i=voronoi_nodes.size()-1;i>=0;i--){
        VoronoiNode node = voronoi_nodes[i];
        std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
        if(adjacent.size()==0){
            voronoi_nodes.erase(voronoi_nodes.begin()+i);
        }
    }
    //TEST
    // int num=voronoi_nodes.size();
    // for(int i=0;i<num;i++)
    // {
    //     std::cout<<voronoi_nodes[i].getId()<<std::endl;
    //     std::cout<<"Position: ("<<voronoi_nodes[i].getPosition().x<<","<<voronoi_nodes[i].getPosition().y<<")"<<std::endl;
    //     std::vector<std::pair<int, float>> adjacent = voronoi_nodes[i].getAllAdjacent();
    //     for (const auto& pair : adjacent) {
    //         std::cout << "Adjacent Node ID: " << pair.first << ", Probability: " << pair.second << std::endl;
    //     }
    //     std::cout << std::endl;
    // }
    //TEST
}

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
    bool ** bool_map = getBoolMap(costmap);
    bool_map[start_mx][start_my] = true; // set start point as free
    voronoi_modified->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), bool_map);
    voronoi_modified->update();
    voronoi_modified->updateAlternativePrunedDiagram();  // prune the Voronoi
    getStartNeighbor();

    // 打印起点邻接点信息
    LOGGER_INFO("VoronoiGraph", "Start point neighbors (%zu nodes):", voronoi_nodes_startNeighbor.size());
    for (const auto& node : voronoi_nodes_startNeighbor) {
        LOGGER_INFO("VoronoiGraph", "  Node ID: %d, Position: (%d, %d)", 
            node.getId(), node.getPosition().x, node.getPosition().y);
    }

    bool_map[end_mx][end_my] = true; // set end point as free
    voronoi_modified->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), bool_map);
    voronoi_modified->update();
    voronoi_modified->updateAlternativePrunedDiagram();  // prune the Voronoi
    getEndNeighbor();

    // 打印终点邻接点信息
    LOGGER_INFO("VoronoiGraph", "End point neighbors (%zu nodes):", voronoi_nodes_endNeighbor.size());
    for (const auto& node : voronoi_nodes_endNeighbor) {
        LOGGER_INFO("VoronoiGraph", "  Node ID: %d, Position: (%d, %d)", 
            node.getId(), node.getPosition().x, node.getPosition().y);
    }

    //release the bool map
    for (int x = 0; x < costmap->getSizeInCellsX(); x++) {
        delete[] bool_map[x];
    }
    delete[] bool_map;

    // 清空修改后的节点列表
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
                    VoronoiNode node(id, MapPoint{x, y});
                    voronoi_nodes_modified.push_back(node);
                    id++;
                }
            }
        }
    }


    // 为其他节点建立连接关系
    int node_size = voronoi_nodes_modified.size();
    for(int i=0;i<node_size;i++){
        bool** map_flag = new bool*[sizeX];
        for (int x = 0; x < sizeX; x++) {
            map_flag[x] = new bool[sizeY]();
        }

        while(true){
            int j=i;
            VoronoiNode node = voronoi_nodes_modified[i];
            int start_id = node.getId();
            MapPoint start = node.getPosition();
            Path center_path;
            
            // if the node is near the last node , then we see it as the same node
            if(i!=0){
                VoronoiNode last_node = voronoi_nodes_modified[i-1];
                MapPoint last_position = last_node.getPosition();
                if(abs(last_position.x-start.x)<=1 && abs(last_position.y-start.y)<=1){
                    center_path.path_points.push_back(last_position);
                    j=i-1;
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
                        if(start_id==target_id){
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

    // 四连通问题补丁（与上面的代码相关）
    // 删除没有邻接点的节点
    for(int i=voronoi_nodes_modified.size()-1;i>=0;i--){
        VoronoiNode& node = voronoi_nodes_modified[i];
        std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
        if(adjacent.size()==0){
            voronoi_nodes_modified.erase(voronoi_nodes_modified.begin()+i);
        }
    }
    // 四连通问题补丁

    //更新邻接点id
    LOGGER_INFO("VoronoiGraph", "Updating start neighbor node IDs...");
    std::vector<VoronoiNode> updated_start_neighbors;
    for (const auto& start_neighbor : voronoi_nodes_startNeighbor) {
        bool found_match = false;
        MapPoint start_pos = start_neighbor.getPosition();
        
        // 遍历modified_nodes寻找匹配的节点
        for (const auto& modified_node : voronoi_nodes_modified) {
            MapPoint modified_pos = modified_node.getPosition();
            if (start_pos.x == modified_pos.x && start_pos.y == modified_pos.y) {
                // 找到匹配的节点，创建新节点
                VoronoiNode new_node(modified_node.getId(), start_pos);
                updated_start_neighbors.push_back(new_node);
                LOGGER_INFO("VoronoiGraph", "Updated start neighbor node ID: %d -> %d, Position: (%d, %d)", 
                    start_neighbor.getId(), modified_node.getId(), start_pos.x, start_pos.y);
                found_match = true;
                break;
            }
        }
        
        if (!found_match) {
            LOGGER_INFO("VoronoiGraph", "Removed start neighbor node with no match, Position: (%d, %d)", 
                start_pos.x, start_pos.y);
        }
    }
    
    // 更新voronoi_nodes_startNeighbor
    voronoi_nodes_startNeighbor = updated_start_neighbors;
    LOGGER_INFO("VoronoiGraph", "Start neighbor nodes after update: %zu", voronoi_nodes_startNeighbor.size());
    for (const auto& node : voronoi_nodes_startNeighbor) {
        LOGGER_INFO("VoronoiGraph", "  Node ID: %d, Position: (%d, %d)", 
            node.getId(), node.getPosition().x, node.getPosition().y);
    }

    // 更新终点邻接点ID
    LOGGER_INFO("VoronoiGraph", "Updating end neighbor node IDs...");
    std::vector<VoronoiNode> updated_end_neighbors;
    for (const auto& end_neighbor : voronoi_nodes_endNeighbor) {
        bool found_match = false;
        MapPoint end_pos = end_neighbor.getPosition();
        
        // 遍历modified_nodes寻找匹配的节点
        for (const auto& modified_node : voronoi_nodes_modified) {
            MapPoint modified_pos = modified_node.getPosition();
            if (end_pos.x == modified_pos.x && end_pos.y == modified_pos.y) {
                // 找到匹配的节点，创建新节点
                VoronoiNode new_node(modified_node.getId(), end_pos);
                updated_end_neighbors.push_back(new_node);
                LOGGER_INFO("VoronoiGraph", "Updated end neighbor node ID: %d -> %d, Position: (%d, %d)", 
                    end_neighbor.getId(), modified_node.getId(), end_pos.x, end_pos.y);
                found_match = true;
                break;
            }
        }
        
        if (!found_match) {
            LOGGER_INFO("VoronoiGraph", "Removed end neighbor node with no match, Position: (%d, %d)", 
                end_pos.x, end_pos.y);
        }
    }
    
    // 更新voronoi_nodes_endNeighbor
    voronoi_nodes_endNeighbor = updated_end_neighbors;
    LOGGER_INFO("VoronoiGraph", "End neighbor nodes after update: %zu", voronoi_nodes_endNeighbor.size());
    for (const auto& node : voronoi_nodes_endNeighbor) {
        LOGGER_INFO("VoronoiGraph", "  Node ID: %d, Position: (%d, %d)", 
            node.getId(), node.getPosition().x, node.getPosition().y);
    }

    // 添加起点作为节点
    start_point_node_id = id++;
    VoronoiNode start_node(start_point_node_id, MapPoint{static_cast<int>(start_mx), static_cast<int>(start_my)});
    voronoi_nodes_modified.push_back(start_node);

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
    voronoi_nodes_modified.push_back(end_node);

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

    // 更新voronoi_nodes_modified中的起点和终点节点
    size_t start_node_index = voronoi_nodes_modified.size() - 2;  // 倒数第二个位置
    size_t end_node_index = voronoi_nodes_modified.size() - 1;    // 倒数第一个位置

    LOGGER_INFO("VoronoiGraph", "Updating start node at index %zu and end node at index %zu", 
        start_node_index, end_node_index);

    voronoi_nodes_modified[start_node_index] = start_node;
    voronoi_nodes_modified[end_node_index] = end_node;

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

    // 在函数结束前打印所有节点的邻接点信息
    LOGGER_INFO("VoronoiGraph", "All nodes in modified graph (%zu nodes):", voronoi_nodes_modified.size());
    for (const auto& node : voronoi_nodes_modified) {
        LOGGER_INFO("VoronoiGraph", "Node ID: %d, Position: (%d, %d)", 
            node.getId(), node.getPosition().x, node.getPosition().y);
        
        // 打印该节点的所有邻接点
        std::vector<std::pair<int, float>> adjacent = node.getAllAdjacent();
        LOGGER_INFO("VoronoiGraph", "  Adjacent nodes (%zu):", adjacent.size());
        for (const auto& adj : adjacent) {
            LOGGER_INFO("VoronoiGraph", "    Node ID: %d, Probability: %.3f", adj.first, adj.second);
        }
    }

    // 打印起点和终点节点的ID
    LOGGER_INFO("VoronoiGraph", "Start point node ID: %d", start_point_node_id);
    LOGGER_INFO("VoronoiGraph", "End point node ID: %d", end_point_node_id);
}


// 获取修改的图中与起点相邻的节点
void VoronoiGraph::getStartNeighbor()
{
    voronoi_nodes_startNeighbor.clear();
    
    // 获取原始Voronoi图和修改后的Voronoi图的所有节点
    std::vector<VoronoiNode>& original_nodes = voronoi_nodes;
    std::vector<VoronoiNode> modified_nodes;
    
    // 从voronoi_modified中获取节点
    int sizeX = voronoi_modified->getSizeX();
    int sizeY = voronoi_modified->getSizeY();
    int id = 0;
    
    // 使用与getVoronoiGraph()相同的逻辑来获取节点
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
                    VoronoiNode node(id, MapPoint{x, y});
                    modified_nodes.push_back(node);
                    id++;
                }
            }
        }
    }
    
    // 遍历修改后的图中的所有节点
    for (const auto& modified_node : modified_nodes) {
        bool is_unique = true;
        
        // 检查该节点是否在原始图中存在
        for (const auto& original_node : original_nodes) {
            // 如果节点位置相同，则认为不是特有节点
            if (modified_node.getPosition().x == original_node.getPosition().x &&
                modified_node.getPosition().y == original_node.getPosition().y) {
                is_unique = false;
                break;
            }
        }
        
        // 如果是特有节点，添加到voronoi_nodes_startNeighbor中
        if (is_unique) {
            voronoi_nodes_startNeighbor.push_back(modified_node);
        }
    }
}

// 获取修改的图中与终点相邻的节点
void VoronoiGraph::getEndNeighbor()
{
    voronoi_nodes_endNeighbor.clear();
    
    // 获取原始Voronoi图和修改后的Voronoi图的所有节点
    std::vector<VoronoiNode>& original_nodes = voronoi_nodes;
    std::vector<VoronoiNode> modified_nodes;
    
    // 从voronoi_modified中获取节点
    int sizeX = voronoi_modified->getSizeX();
    int sizeY = voronoi_modified->getSizeY();
    int id = 0;
    
    // 使用与getVoronoiGraph()相同的逻辑来获取节点
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
                    VoronoiNode node(id, MapPoint{x, y});
                    modified_nodes.push_back(node);
                    id++;
                }
            }
        }
    }
    
    // 遍历修改后的图中的所有节点
    for (const auto& modified_node : modified_nodes) {
        bool is_unique = true;
        
        // 检查该节点是否在原始图中存在
        for (const auto& original_node : original_nodes) {
            // 如果节点位置相同，则认为不是特有节点
            if (modified_node.getPosition().x == original_node.getPosition().x &&
                modified_node.getPosition().y == original_node.getPosition().y) {
                is_unique = false;
                break;
            }
        }
        
        // 检查该节点是否在startNeighbor中存在
        if (is_unique) {
            for (const auto& start_node : voronoi_nodes_startNeighbor) {
                if (modified_node.getPosition().x == start_node.getPosition().x &&
                    modified_node.getPosition().y == start_node.getPosition().y) {
                    is_unique = false;
                    break;
                }
            }
        }
        
        // 如果是特有节点，添加到voronoi_nodes_endNeighbor中
        if (is_unique) {
            voronoi_nodes_endNeighbor.push_back(modified_node);
        }
    }
}
