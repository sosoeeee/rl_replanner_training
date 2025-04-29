/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/optimal_planner.h>
#include <logger.h>

#include <signal.h>
#include <memory>


using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;

// Add global variable for program control
volatile sig_atomic_t g_shutdown_flag = 0;

// Signal handler function
void signalHandler(int signum) {
    g_shutdown_flag = 1;
}

// =============== Main function =================
int main( int argc, char** argv )
{
  // load ros parameters from node handle
  // config.loadRosParamFromNodeHandle(n);

  obst_vector.push_back(std::make_shared<PointObstacle>(-3.0,1.0) );
  obst_vector.push_back(std::make_shared<PointObstacle>(6.0,2.0) );
  obst_vector.push_back(std::make_shared<PointObstacle>(0.0,0.1) );
  
  // Setup robot shape model
  // config.robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n, config);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  // if (config.hcp.enable_homotopy_class_planning)
  //   planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, visual, &via_points));
  // else
  //   planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, visual, &via_points));

  planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, &via_points));
  
  LOGGER_INFO("teb_local_planner", "TebLocalPlannerROS initialized.");

  while (!g_shutdown_flag)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    try
    {
      if (planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0))) // check if planning was successful
      {
        std::vector<TrajectoryPointMsg> trajectory;
        planner->getFullTrajectory(trajectory);
        // print the length of the trajectory
        LOGGER_INFO("teb_local_planner", "Trajectory length: %zu", trajectory.size());
      }
      auto end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_time = end_time - start_time;
      LOGGER_INFO("teb_local_planner", "Planning time: %f ms", elapsed_time.count() * 1000.0);
    }
    catch (const std::exception& e)
    {
      LOGGER_ERROR("teb_local_planner", "Exception: %s", e.what());
    }
  }

  return 0;
}
