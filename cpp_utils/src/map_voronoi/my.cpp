#include "map_voronoi/mapvoronoi.h"
#include <iostream>
#include <fstream>
#include <string.h>
// #include <opencv2/opencv.hpp>

using namespace std;

void loadPGM( std::istream &is, int *sizeX, int *sizeY, bool ***map ) {
    std::string tag;
    is >> tag;
    if (tag!="P5") {
        std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
        exit(-1);
    }
    
    while (is.peek()==' ' || is.peek()=='\n') is.ignore();
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> *sizeX;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> *sizeY;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> tag;
    if (tag!="255") {
        std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
        exit(-1);
    }
    is.ignore(255, '\n');  
    
    *map = new bool*[*sizeX];

    for (int x=0; x<*sizeX; x++) {
        (*map)[x] = new bool[*sizeY];
    }
    for (int y=*sizeY-1; y>=0; y--) {
        for (int x=0; x<*sizeX; x++) {
            int c = is.get();
            if ((double)c<255-255*0.1) (*map)[x][y] = true; // cell is occupied
            else (*map)[x][y] = false; // cell is free
            if (!is.good()) {
                std::cerr << "Error reading pgm map.\n";
                exit(-1);
            }
        }
    }
}

int main( int argc, char *argv[] ) {
    if(argc<2 || argc>3 || (argc==3 && !(strcmp(argv[2],"prune")==0 || strcmp(argv[2],"pruneAlternative")==0))) {
        std::cerr<<"usage: "<<argv[0]<<" <pgm map> [prune|pruneAlternative]\n";
        exit(-1);
    }
    
    bool doPrune = false;
    bool doPruneAlternative = true;
    if (argc==3) doPrune = true;

    if(doPrune && strcmp(argv[2],"pruneAlternative")==0){
        doPrune = false;
        doPruneAlternative = true;
    }

    // LOAD PGM MAP AND INITIALIZE THE VORONOI
    std::ifstream is(argv[1]);  // 打开文件
    if (!is) {
        std::cerr << "Could not open map file for reading.\n";
        exit(-1);
    }
    
    bool **map=NULL;
    int sizeX, sizeY;
    loadPGM(is, &sizeX, &sizeY, &map);  // 传递文件流
    is.close();
    fprintf(stderr, "Map loaded (%dx%d).\n", sizeX, sizeY);

    // create the voronoi object and initialize it with the map
    MapVoronoi voronoi;
    voronoi.initializeMap(sizeX, sizeY, map);
    voronoi.update(); // update distance map and Voronoi diagram
    if (doPrune) {
        voronoi.prune();  // prune the Voronoi
    }
    if (doPruneAlternative) {
        voronoi.updateAlternativePrunedDiagram();  // prune the Voronoi
    }

    voronoi.iscenter();
    voronoi.getCenters();
    // voronoi.Cut();
    voronoi.visualize("initial.ppm");
    std::cout << "Generated initial frame.\n";
    voronoi.pathFind();

    cout<< voronoi.getDistance(1,1)<<endl;
    cout<<voronoi.getDistance(200,145)<<endl;
    cout<<voronoi.getDistance(145,200)<<endl;

    cout<<voronoi.getSizeX()<<endl;

    //complite code here:

    // 获取距离信息并生成图像
    // float** distanceMap = voronoi.getDistanceMap();
    // cv::Mat distanceImage(sizeY, sizeX, CV_8UC1, cv::Scalar(0));
    // for (int y = 0; y < sizeY; y++) {
    //     for (int x = 0; x < sizeX; x++) {
    //         float distance = distanceMap[x][y];
    //         unsigned char value = static_cast<unsigned char>(distance * 255.0f); // 将距离映射到 0-255 的灰度值
    //         distanceImage.at<uchar>(y, x) = value;
    //     }
    // }

    // // 将灰度图像转换为 BGR 格式
    // cv::Mat distanceImageBGR;
    // cv::cvtColor(distanceImage, distanceImageBGR, cv::COLOR_GRAY2BGR);

    // // 保存距离信息图像
    // cv::imwrite("distance_map.ppm", distanceImageBGR);
    // std::cout << "Distance map saved as 'distance_map.ppm'.\n";

    // // 释放距离信息内存
    // for (int x = 0; x < sizeX; x++) {
    //     delete[] distanceMap[x];
    // }
    // delete[] distanceMap;
    
    // // now perform some updates with random obstacles
    // char filename[20];
    // int numPoints = 10 + sizeX*sizeY*0.005;
    // for (int frame=1; frame<=10; frame++) {
    //     std::vector<IntPoint> newObstacles;
    //     for (int i=0; i<numPoints; i++) {
    //         double x = 2+rand()/(double)RAND_MAX*(sizeX-4);
    //         double y = 2+rand()/(double)RAND_MAX*(sizeY-4);
    //         newObstacles.push_back(IntPoint(x,y));
    //     }
    //     voronoi.exchangeObstacles(newObstacles); // register the new obstacles (old ones will be removed)
    //     voronoi.update();
    //     if (doPrune) {
    //         voronoi.prune();
    //     }
    //     sprintf(filename, "update_%03d.ppm", frame);
    //     voronoi.visualize(filename);
    //     std::cout << "Performed update with random obstacles.\n";
    // }

    // // now remove all random obstacles again.
    // // final.pgm should be very similar to initial.pgm, except for ambiguous spots
    // std::vector<IntPoint> empty;
    // voronoi.exchangeObstacles(empty);
    // voronoi.update();
    // if (doPrune) {
    //     voronoi.prune();
    // }
    // voronoi.visualize("final.ppm");
    // std::cout << "Done with final update (all random obstacles removed).\n";
    // std::cout << "Check initial.ppm, update_???.ppm and final.ppm.\n";

    // 释放地图内存
    for (int x=0; x<sizeX; x++) {
        delete[] map[x];
    }
    delete[] map;

    return 0;
}