#ifndef SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION
#define SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION

#include "ros/ros.h"
#include "motionModel.hpp"
#include "measurementModel.hpp"

class GridLocalisation
{
public:
    GridLocalisation(const ros::NodeHandle &nh,
                     const std::shared_ptr<MotionModel> &motion_model,
                     const std::shared_ptr<MeasurementModel> &measurement_model);
private:
    std::shared_ptr<MotionModel> motion_model_;
    std::shared_ptr<MeasurementModel> measurement_model_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION */
