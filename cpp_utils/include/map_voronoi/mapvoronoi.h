#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <vector>
#include <algorithm>

#include "bucketedqueue.h"

//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class MapVoronoi {

private:

  //首先定义点的数据结构
  struct Point {
    int x;
    int y;
  };

  //每个center都存储一个path数组，path数组中首先存储一个id;
  struct Path {
    int id;
    std::vector<Point> path;  //路径：用点集表示
  };

  //
  struct Center {
    Point point;  //该点坐标
    int id;
    int friend_id=-1;
    std::vector<int> neighbors;  //用于记录该点的邻居节点(记录节点id)
    bool flag;  //用于标记该点是否已经被访问过
    std::vector<Path> path;
  };

  struct dataCell {
    float dist;
    char voronoi;   //State的枚举值
    char queueing;  //QueueingState的枚举值
    int obstX;
    int obstY;
    bool needsRaise;
    int sqdist;
  };

  typedef enum {voronoiKeep=-4, freeQueued = -3, voronoiRetry=-2, voronoiPrune=-1, free=0, occupied=1, cut=2} State;
  //下面这几个枚举状态没搞懂
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;



  // methods
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist=true);
  inline void reviveVoroNeighbors(int &x, int &y);

  inline bool isOccupied(int &x, int &y, dataCell &c);
  inline markerMatchResult markerMatch(int x, int y);
  inline bool markerMatchAlternative(int x, int y);
  inline int getVoronoiPruneValence(int x, int y);

  // queues

  BucketPrioQueue<INTPOINT> open_;
  std::queue<INTPOINT> pruneQueue_;
  BucketPrioQueue<INTPOINT> sortedPruneQueue_;

  std::vector<INTPOINT> removeList_;
  std::vector<INTPOINT> addList_;
  std::vector<INTPOINT> lastObstacles_;

  // maps
  int sizeY_;
  int sizeX_;
  dataCell** data_;
  bool** gridMap_;          //true是被占用，false是没有被占用
  bool allocatedGridMap_;   //是否为gridmap分配了内存的标志位

  // parameters
  int padding_;
  double doubleThreshold_;

  double sqrt2_;

  //  dataCell** getData(){ return data; }
  int** alternativeDiagram_;
  int** centers_;
  Center* centers;
  // std::vector<int> test;

public:

  MapVoronoi();
  ~MapVoronoi();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist=true);
  //! prune the Voronoi diagram
  void prune();
  //! prune the Voronoi diagram by globally revisiting all Voronoi nodes. Takes more time but gives a more sparsely pruned Voronoi graph. You need to call this after every call to udpate()
  void updateAlternativePrunedDiagram();
  //! retrieve the alternatively pruned diagram. see updateAlternativePrunedDiagram()
  int** alternativePrunedDiagram(){
    return alternativeDiagram_;
  };
  //! retrieve the number of neighbors that are Voronoi nodes (4-connected)
  int getNumVoronoiNeighborsAlternative(int x, int y);
  //! returns whether the specified cell is part of the alternatively pruned diagram. See updateAlternativePrunedDiagram.
  bool isVoronoiAlternative( int x, int y );
  //用于判断中心点的坐标：
  void iscenter();

  int getCenterNum();

  //获取未meirge之前的size
  void getCenters();

  //注意在获取节点之后使用：
  void pathFind();

  bool checkCenter(Point p);

  int checkCenterId(Point p);

  //在获取路径之后使用：
  // void MergeCenter();

  void fakeMerge();
  // void Cut();

  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi( int x, int y );
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX_;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY_;}

};


#endif

