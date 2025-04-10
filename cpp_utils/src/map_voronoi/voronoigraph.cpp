#include "map_voronoi/voronoigraph.h"

void VoronoiGraph::visualizeVoronoi(const std::string& filename) {
    if (voronoi) {
        voronoi->visualize(filename.c_str());  // 使用 c_str() 转换
    } else {
        std::cerr << "Error: Voronoi object is not initialized." << std::endl;
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

void VoronoiGraph::getVoronoiGraph(){
    int sizeX = voronoi->getSizeX();
    int sizeY = voronoi->getSizeY();
    int id=0;
    for(int y = sizeY-1; y >=0; y--){
        for(int x = 0; x<sizeX; x++){
            if (voronoi->isVoronoiAlternative(x,y)) {
                int num=0;
                for(int nx=-1; nx<=1; nx++){
                  for(int ny=-1; ny<=1; ny++){
                    if (nx==0 && ny==0) continue;
                    if(nx*ny==-1||nx*ny==1) continue;
                    if (x+nx<0 || x+nx>=sizeX || y+ny<0 || y+ny>=sizeY) continue;
                    if (voronoi->isVoronoiAlternative(x+nx,y+ny)) {
                      num++;
                    }
                  }
                }
                if(num>=3){

                    // For What?
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
        
        // Simplified memory allocation_flag=new bool*[sizeX];
        // for(int x=0;x<sizeX;x++){
        //   map_flag[x]=new bool[sizeY];
        // }
        // for(int x=0;x<sizeX;x++){
        //   for(int y=0;y<sizeY;y++){
        //     map_flag[x][y]=false;
        //   }
        // }
        // bool** map_flag=new bool*[sizeX];
        // for(int x=0;x<sizeX;x++){
        //   map_flag[x]=new bool[sizeY];
        // }
        // for(int x=0;x<sizeX;x++){
        //   for(int y=0;y<sizeY;y++){
        //     map_flag[x][y]=false;
        //   }
        // }
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
            
            // for what?
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
                if(end.x+1<sizeX && map_flag[end.x+1][end.y]==false && voronoi->isVoronoiAlternative(end.x+1,end.y)){
                    end.x = end.x+1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.y+1<sizeY && map_flag[end.x][end.y+1]==false && voronoi->isVoronoiAlternative(end.x,end.y+1)){
                    end.y = end.y+1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.x-1>=0 && map_flag[end.x-1][end.y]==false && voronoi->isVoronoiAlternative(end.x-1,end.y)){
                    end.x = end.x-1;
                    center_path.path_points.push_back(end);
                    map_flag[end.x][end.y]=true;
                }else if(end.y-1>=0 && map_flag[end.x][end.y-1]==false && voronoi->isVoronoiAlternative(end.x,end.y-1)){
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
    int num=voronoi_nodes.size();
    for(int i=0;i<num;i++)
    {
        std::cout<<voronoi_nodes[i].getId()<<std::endl;
        std::cout<<"Position: ("<<voronoi_nodes[i].getPosition().x<<","<<voronoi_nodes[i].getPosition().y<<")"<<std::endl;
        std::vector<std::pair<int, float>> adjacent = voronoi_nodes[i].getAllAdjacent();
        for (const auto& pair : adjacent) {
            std::cout << "Adjacent Node ID: " << pair.first << ", Probability: " << pair.second << std::endl;
        }
        std::cout << std::endl;
    }
    //TEST
}

std::vector<int> VoronoiGraph::getPassbyNodes(int start_id, int end_id)
{   
    std::vector<int> passby_nodes;
    std::vector<std::pair<int, float>> adjacent_nodes = voronoi_nodes[start_id].getAllAdjacent();
    passby_nodes.push_back(start_id);
    for (const auto& pair : adjacent_nodes) {
        voronoi_nodes[pair.first].updateProbability(start_id);
    }

    resetAllProbabilities();
    int next_id = voronoi_nodes[start_id].getAdjacent();
    while (next_id != end_id) {
        
    }
}