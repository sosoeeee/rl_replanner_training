#include "map_voronoi/voronoi.h"
#include "map_voronoi/voronoinode.h"
#include "map_voronoi/voronoigraph.h"
#include <iostream>
#include <fstream>
#include <string.h>

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

    VoronoiGraph voronoigraph;
    voronoigraph.getVoronoiGraph(sizeX, sizeY, map);
    voronoigraph.visualizeVoronoi("initial.ppm");
    std::cout << "Generated initial frame.\n";

    // cout<< voronoigraph.voronoi.getDistance(1,1)<<endl;
    // cout<<voronoi.getDistance(200,145)<<endl;
    // cout<<voronoi.getDistance(145,200)<<endl;

    for (int x=0; x<sizeX; x++) {
        delete[] map[x];
    }
    delete[] map;

    return 0;
}