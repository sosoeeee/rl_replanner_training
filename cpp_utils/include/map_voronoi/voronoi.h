#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <vector>
#include <algorithm>

#include "bucketedqueue.h"

class Voronoi {
private:
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


public:

  Voronoi();
  ~Voronoi();

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

  void mergeVoronoi();

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

