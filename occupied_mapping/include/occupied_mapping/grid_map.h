#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "occupied_mapping/common.h"
#include "occupied_mapping/tools_kd_hash.hpp"
#include "debug_tools/debug_tools.h"

namespace occupied_mapping{
template <typename PointT>
class GridMap{
public:
    using PointVec = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
    typedef std::shared_ptr<GridMap> Ptr;
private:
    BoxPointType bounding_box_;
    Eigen::Vector4f FOV_range_; //[hor_min, hor_max, ver_min, ver_max]
    float resolution_, max_range_, max_range_sqrt_, sliding_threshold_;
    bool is_map_param_set_{false}, is_center_initialized_{false};
    Eigen::Vector3f map_center_;
    Hash_map_3d<float, PointT> map_;
    int len_low_[3], len_high_, size_low_, size_high_;
    int side_L_[3];  //x y z 索引长度

    float calDist(PointT p1, PointT p2);
    bool isInsideBox(PointT p);
    //返回p与边界最小点所构成的立方体区域
    BoxPointType acquireBox(PointT p);
    //返回p所在的栅格的中心点
	PointT acquireCenter(PointT p);

public:
    GridMap();
    GridMap(float resolution, BoxPointType box);
    ~GridMap();

    // @param FOV[up, down, left, right]
    void setMapParams(float resolution, BoxPointType box, Eigen::Vector4f FOV,  float range=INFINITY, float sliding_threshold=0.1);
    //遍历points内所有点p
	//查询点是否存在于哈希表内， 不存在则insert(p)
	//存在则计算原始点，新点与栅格中心的距离，挑选距离center较近的点，作为新值
    void addPoints(PointVec &points);
    //多一个参数odom -> 用于判断 (p - odom.pos).norm() 是否小于最大考虑范围，否则 continus
    void addPoints(PointVec &points, OdomType odom);
    PointVec getCenterNeibours(PointT& center, int range_index);
    void deletePoints(PointVec &points, OdomType odom);
    //将哈希表所有点装入 points返回
    void getPointsToVec(PointVec& vec, int down_rate, BoxPointType& box);
    //返回所有栅格中心坐标集合
    void getCentersToVec(PointVec& vec, int down_rate);

    void getPointsToCloud(PointCloud::Ptr& cloud);  //提取全部点

    //删除边界外的点, 边界赋值为new_bbx
    inline void slideMap(BoxPointType new_box);
    void slideMap(OdomType odom);  // 调用 inline void slideMap(BoxPointType new_box);
    bool isOccupied(PointT point);
    //清除所有元素  Hash_map_3d<float, PointType> map
    void clear();

    bool checkFOV(PointT point);

    inline size_t getMemory(){
        return map_.get_memory() + sizeof(*this);
    }

    inline double getMemoryKB(){
        return static_cast<double>(getMemory() / 1024.0);
    }

    inline double getMemoryMB(){
        return static_cast<double>(getMemory() / 1024.0 / 1024.0);
    }

    inline double getMemoryGB(){
        return static_cast<double>(getMemory() / 1024.0 / 1024.0 / 1024.0);
    }
};
}

#endif