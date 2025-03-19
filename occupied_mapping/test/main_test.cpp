#include "occupied_mapping/grid_map.h"

int main(){
    auto grid_map = std::make_shared<occupied_mapping::GridMap<occupied_mapping::PointType>>();
    occupied_mapping::BoxPointType box{-10, 10, -11, 11, -12, 12};

    debug_tools::Timer t_0;

    grid_map->setMapParams(0.01, box, Eigen::Vector4f(2, -2, -2, 2), 10);

    grid_map->checkFOV(pcl::PointXYZI(2, 2, -2));

    grid_map->clear();

    debug_tools::Timer t_1(t_0); t_1.log("time consumed", "ms");
    return 0;
}