
#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {

        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();

    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {

        /**************************
         * Implement your code here
         **************************/

        // Convert to map coordinates

        unsigned int map_x, map_y;
        costmap_->worldToMap(world_pos[0], world_pos[1], map_x, map_y); // supposed to be there #include <costmap_2d/costmap_2d.h>
        // http://docs.ros.org/en/electric/api/costmap_2d/html/costmap__2d_8h_source.html
        // http://docs.ros.org/en/electric/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html#ab7134df329c4d4c174297f5db070c5a5


        //is the point in the map ?
        if((map_x < 0) || (map_y < 0) || ((map_x >= costmap_->getSizeInCellsX()) || (map_y >= costmap_->getSizeInCellsY())))
            return false; // not in free space, out of bounds
        
        
        /*
        While each cell in the costmap can have one of 255 different cost values (see the inflation section), 
        the underlying structure that it uses is capable of representing only three. 
        Specifically, each cell in this structure can be either free, occupied, or unknown. Each status has a special cost value assigned to it upon projection into the costmap.
        Columns that have a certain number of occupied cells (see mark_threshold parameter) are assigned a costmap_2d::LETHAL_OBSTACLE cost, 
        columns that have a certain number of unknown cells (see unknown_threshold parameter) are assigned a costmap_2d::NO_INFORMATION cost, and other columns are assigned a costmap_2d::FREE_SPACE cost. 
        http://library.isr.ist.utl.pt/docs/roswiki/costmap_2d.html#Inflation

        */

        int cost = static_cast<int>(costmap_->getCost(map_x, map_y)); //returns a char must convert to int
        if (cost =! 0){ // =0 signifies free space
            return false;
        }
        else
        {
            return true;
        }


    }   

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {

        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return ( !inFreeSpace(point_b) ) ? true : false;

        } else {
            
            int num_steps = static_cast<int>(floor(dist/resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {

                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }

    }

};