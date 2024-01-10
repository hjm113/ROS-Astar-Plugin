#ifndef ASTAR_PLANNER_CPP
#define ASTAR_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <vector>
#include <queue>
#include <cmath>

using std::string;
using std::vector;
using std::priority_queue;
using std::abs;

struct Node{
    double f_cost, g_cost;
    int idx, cell_cost;
};

namespace astar_planner {
    class AstarPlanner : public nav_core::BaseGlobalPlanner {
        public:
            bool initialized;
            costmap_2d::Costmap2DROS *m_costmap_ros;
            costmap_2d::Costmap2D *m_costmap;
            int cellsY;
            int cellsX;
            double resolution;
            int area;
            vector<int> OccupancyGridMap;
            string m_frame_id;
            ros::Publisher pub;
            AstarPlanner(); // default constructor
            AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            /** overridden classes from interface nav_core::BaseGlobalPlanner **/
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,vector<geometry_msgs::PoseStamped>& plan);
            
            //helper methods
            vector<geometry_msgs::PoseStamped> getPath(vector<int> parentNode, int start_idx, int goal_idx);
            vector<int> getAdjacent(int cur_idx);
            void outlineMap();
            double getGcost(int fstIdx, int sndIdx);
            int getHeuristic(int nIdx, int goalIdx);
            bool areaLimit(int x, int y);
    };
};

#endif
