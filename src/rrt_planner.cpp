
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    return true;
                }
            }
        }

        return false;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {

        /**************************
         * Implement your code here
         **************************/
        float nearest_dist = computeDistance(nodes_[0].pos, point);
        float nearest_node_index = 0;
         for (int i = 1; i < nodes_.size(); i++) {
            float dist_point_node = computeDistance(nodes_[i].pos, point);
            if (dist_point_node < nearest_dist){
                nearest_dist = dist_point_node;
                nearest_node_index = i;
            }
         }
         return nearest_node_index; // or node_[nearest_node_index].node_id

    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {

        Node new_node(pos, nodes_.size(), parent_node_id); // not sure how to give the pos and the node_id

        /**************************
         * Implement your code here
         **************************/



        nodes_.emplace_back(new_node);
        
    }

    double* RRTPlanner::sampleRandomPoint() {

        /**************************
         * Implement your code here
         **************************/

        rand_point_[0] = random_double_x.generate();
        rand_point_[1] = random_double_y.generate();

        return rand_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        /**************************
         * Implement your code here
         **************************/


        // Not sure what is wanted here? 
        // Because the point is random so could be really far. So is the parameter step the longest distance allowed between nodes?
        // Or else candidate_point is just euqals to point random so what is the goal of having a function


        double distance_node_prand = computeDistance(point_nearest, point_rand);

        if(distance_node_prand < params_.step){
            candidate_point_[0] = point_nearest[0];
            candidate_point_[1] = point_nearest[0];
        }
        else { 
            double theta = atan2(point_rand[1]-point_nearest[1], point_rand[0] - point_nearest[0]);
            candidate_point_[0] = point_nearest[0] + params_.step*cos(theta);
            candidate_point_[1] = point_nearest[1] + params_.step*sin(theta);
        }

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};