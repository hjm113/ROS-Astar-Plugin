#include "AstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseAstarPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

// compare function for minimum heap
struct cmp {
    bool operator()(Node &fst_node, Node &snd_node) {
        return fst_node.f_cost > snd_node.f_cost;
    }
};

//finding the adjacent nodes in the clockwise direction
int dx[8] = {0,1,1,1,0,-1,-1,-1};
int dy[8] = {-1,-1,0,1,1,1,0,-1};
namespace astar_planner {

    //Constructor
    AstarPlanner::AstarPlanner(){}

    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    //initialized the atar planner
    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized) {
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            cellsY = m_costmap->getSizeInCellsY();
            cellsX = m_costmap->getSizeInCellsX();
            area = cellsY * cellsX;
            resolution = m_costmap->getResolution();
            OccupancyGridMap.clear();
            OccupancyGridMap.resize(area);
            for(unsigned int i = 0; i < cellsY; i++) {
                for(unsigned int j = 0; j < cellsX; j++) {
                    unsigned int cost = static_cast<int>(m_costmap->getCost(j,i));
                    if(cost == 0) {
                        OccupancyGridMap[i*cellsX+j] = 1;
                    }
                    else {
                        OccupancyGridMap[i*cellsX+j] = 0;
                    }
                    
                }
            }

            m_frame_id = m_costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/"+name);
            pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized = true;
        }
        else {
            ROS_WARN("Already Initialized");

        }
    }

    //making the path of the AMR
    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        if(!initialized) {
            ROS_WARN("Not Initialized");
            return false;
        }

        //get the start pose of the map from the world
        double world_sx = start.pose.position.x;
        double world_sy = start.pose.position.y;
        unsigned int start_x,start_y;
        m_costmap->worldToMap(world_sx,world_sy,start_x,start_y);

        //get the goal pose of the map from the world
        double world_gx = goal.pose.position.x;
        double world_gy = goal.pose.position.y;
        unsigned int goal_x,goal_y;
        m_costmap->worldToMap(world_gx,world_gy,goal_x,goal_y);

        //convert to the index from the coordinates
        int start_idx = m_costmap->getIndex(start_x,start_y);
        int goal_idx = m_costmap->getIndex(goal_x,goal_y);
        
        //check the start and goal located at the correct poistion
        if(OccupancyGridMap[start_idx] == 0 || !areaLimit(start_x,start_y)) {
            ROS_WARN("%d  %d       %d",start_x,start_y,OccupancyGridMap[start_idx]);
            ROS_WARN("Wrong start position");
            return false;
        }
        if(OccupancyGridMap[goal_idx] == 0 || !areaLimit(goal_x,goal_y)) {
            ROS_WARN("Wrong goal position");
            return false;
        }
        //pq to get the lowest value of the fuctions
        //make the open and cloosed checking vector
        //make the parent node
        priority_queue<Node,vector<Node>,cmp> pq_wait;
        vector<int> open(area,1000000);
        vector<bool> closed(area,false);
        vector<int> parentNode(area,-10);

        //make the start node
        Node start_node;
        start_node.f_cost = 0 + getHeuristic(start_idx,goal_idx);
        start_node.g_cost = 0;
        start_node.idx = start_idx;
        open[start_idx] = start_node.f_cost;
        parentNode[start_idx] = -1;
        pq_wait.push(start_node);
        
        //find the path and allocate it to the parentNode
        while(!pq_wait.empty()) {
            Node cur = pq_wait.top();
            pq_wait.pop();
            if(closed[cur.idx]== true) {
                continue;
            }
            if(cur.idx == goal_idx) {
                break;
            }
            closed[cur.idx] = true;
            vector<int> adj = getAdjacent(cur.idx);
            for(int i = 0; i < adj.size(); i++) {
                Node nxt;
                nxt.idx = adj[i];
                if(closed[nxt.idx]) {
                    continue;
                }
                nxt.g_cost = cur.g_cost + getGcost(cur.idx,nxt.idx);
                nxt.f_cost = nxt.g_cost + getHeuristic(nxt.idx,goal_idx);
                if(open[nxt.idx] < nxt.f_cost) {
                    continue;
                }
                open[nxt.idx] = nxt.f_cost;
                parentNode[nxt.idx] = cur.idx;
                pq_wait.push(nxt);
            }
        }

        if(parentNode[goal_idx] == -10) {
            ROS_WARN("Goal pose cannot be reached");
            return false;
        }
        
        //construct the path
        vector<int> path;
        int reverse_start = goal_idx;
        while(parentNode[reverse_start] != -1) {
            path.push_back(reverse_start);
            reverse_start = parentNode[reverse_start];
        }
        path.push_back(start_idx);
        reverse(path.begin(),path.end());

        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < path.size(); i++) {
            unsigned int tmp_x,tmp_y;
            m_costmap->indexToCells(path[i],tmp_x,tmp_y);
            double cur_x,cur_y;
            m_costmap->mapToWorld(tmp_x,tmp_y,cur_x,cur_y);

            m_costmap->indexToCells(path[i-1],tmp_x,tmp_y);
            double prev_x,prev_y;
            m_costmap->mapToWorld(tmp_x,tmp_y,prev_x,prev_y);
            double angle = atan2(cur_y-prev_y,cur_x-prev_x);

            geometry_msgs::PoseStamped coord;
            coord.header.stamp = plan_time;
            coord.header.frame_id = m_costmap_ros->getGlobalFrameID();
            coord.pose.position.x = cur_x;
            coord.pose.position.y = cur_y;
            coord.pose.position.z = 0.0;

            coord.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
            plan.push_back(coord);
        }
        return true;
    }

    //find the adjacent nodes
    vector<int> AstarPlanner::getAdjacent(int cur_idx) {
        vector<int> adj;
        unsigned int cur_x, cur_y;
        m_costmap->indexToCells(cur_idx,cur_x,cur_y);
        for(int dir = 0; dir < 8; dir++) {
            int nx = cur_x+dx[dir];
            int ny = cur_y+dy[dir];
            if(!areaLimit(nx,ny)) {
                continue;
            }
            int nidx = m_costmap->getIndex(nx,ny);
            if(OccupancyGridMap[nidx] == 0) {
                continue;
            }
            adj.push_back(nidx);
        }
        return adj;
    }

    // get the g function value of the adjacent nodes
    double AstarPlanner::getGcost(int fstIdx, int sndIdx) {
        unsigned int tmp_x,tmp_y;
        m_costmap->indexToCells(fstIdx,tmp_x,tmp_y);
        int fst_x = tmp_x;
        int fst_y = tmp_y;

        m_costmap->indexToCells(sndIdx,tmp_x,tmp_y);
        int snd_x = tmp_x;
        int snd_y = tmp_y;
        int cost = abs(fst_x-snd_x) + abs(fst_y-snd_y);
        if(cost == 2) {
            // pythagorean theorem
            return 1.414;
        }
        else {
            return 1.0;
        }
    }

    // get the heuristic function value
    int AstarPlanner::getHeuristic(int nIdx, int goalIdx) {
        unsigned int tmp_x,tmp_y;
        m_costmap->indexToCells(nIdx,tmp_x,tmp_y);
        int nx = tmp_x;
        int ny = tmp_y;

        m_costmap->indexToCells(goalIdx,tmp_x,tmp_y);
        int gx = tmp_x;
        int gy = tmp_y;

        return (abs(nx-gx)+abs(ny-gy));
    }

    //check the limitation of the map size
    bool AstarPlanner::areaLimit(int x, int y) {
        if(x < 0 || y < 0 || x >= cellsX || y >= cellsY) {
            return false;
        }
        return true;
    }
};