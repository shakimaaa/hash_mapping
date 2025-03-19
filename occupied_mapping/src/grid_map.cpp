#include "occupied_mapping/grid_map.h"

namespace occupied_mapping{
template <typename PointT> float
GridMap<PointT>::calDist(PointT p1, PointT p2){
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

template <typename PointT> bool
GridMap<PointT>::isInsideBox(PointT p){
    if (p.x < bounding_box_.vertex_min[0] + EPSS || p.x > bounding_box_.vertex_max[0] - EPSS)
		return false;
	if (p.y < bounding_box_.vertex_min[1] + EPSS || p.y > bounding_box_.vertex_max[1] - EPSS)
		return false;
	if (p.z < bounding_box_.vertex_min[2] + EPSS || p.z > bounding_box_.vertex_max[2] - EPSS)
		return false;
	return true;
}

template <typename PointT> BoxPointType
GridMap<PointT>::acquireBox(PointT p){
    BoxPointType box;
	box.vertex_min[0] = std::floor((p.x - bounding_box_.vertex_min[0]) / resolution_) * resolution_ + bounding_box_.vertex_min[0];
	box.vertex_min[1] = std::floor((p.y - bounding_box_.vertex_min[1]) / resolution_) * resolution_ + bounding_box_.vertex_min[1];
	box.vertex_min[2] = std::floor((p.z - bounding_box_.vertex_min[2]) / resolution_) * resolution_ + bounding_box_.vertex_min[2];
	for (int i = 0; i < 3; i++)
		box.vertex_max[i] = box.vertex_min[i] + resolution_;
	return box;
}

template <typename PointT> PointT
GridMap<PointT>::acquireCenter(PointT p){
	PointType center;
	center.x = std::floor((p.x - bounding_box_.vertex_min[0]) / resolution_) * resolution_ + bounding_box_.vertex_min[0] + resolution_ / 2.0;
	center.y = std::floor((p.y - bounding_box_.vertex_min[1]) / resolution_) * resolution_ + bounding_box_.vertex_min[1] + resolution_ / 2.0;
	center.z = std::floor((p.z - bounding_box_.vertex_min[2]) / resolution_) * resolution_ + bounding_box_.vertex_min[2] + resolution_ / 2.0;
	return center;
}

template <typename PointT> 
GridMap<PointT>::GridMap(){
	is_map_param_set_ = true;
}

template <typename PointT> 
GridMap<PointT>::GridMap(float resolution, BoxPointType box){
	resolution_ = resolution;
	bounding_box_ = box;
	is_map_param_set_ = true;
}

template <typename PointT> 
GridMap<PointT>::~GridMap(){
}

template <typename PointT> void
GridMap<PointT>::setMapParams(float resolution, BoxPointType box, Eigen::Vector4f FOV, float range, float sliding_threshold){
	resolution_ = resolution;
	bounding_box_ = box;
	FOV_range_ = FOV;
	is_map_param_set_ = true;
	for (int i = 0; i < 3; i++) {
		side_L_[i] = ceil((box.vertex_max[i] - box.vertex_min[i]) / resolution_);
	}
	max_range_ = range;
	max_range_sqrt_ = max_range_ * max_range_;
	sliding_threshold_ = sliding_threshold;
}

template <typename PointT> void
GridMap<PointT>::addPoints(PointVec &points){
	assert(is_map_param_set_ && "[occupied_map] grid-map param unset");
	float cur_dist, new_dist;
	for (size_t i{0}; i < points.size(); i++) {
		if(!isInsideBox(points[i]))  continue;
		PointT center = acquireCenter(points[i]);
		if(map_.if_exist(center.x, center.y, center.z)){
			PointT *cur_p = map_.get_data(center.x, center.y, center.z);
			cur_dist = calDist(*cur_p, center);
			new_dist = calDist(points[i], center);
			if(new_dist < cur_dist) *cur_p = points[i];
		} 
		else
			map_.insert(center.x, center.y, center.z, points[i]);
	}
}

template <typename PointT> void
GridMap<PointT>::addPoints(PointVec &points, OdomType odom){
	assert(is_map_param_set_ && "[occupied_map] grid-map param unset");
	float cur_dist, new_dist;
	size_t num_add_1{0}, num_add_2{0}, num_add_3{0};
	for (size_t i{0}; i < points.size(); i++) {
		if(!isInsideBox(points[i]))  continue;
		Eigen::Vector3f p(points[i].x, points[i].y, points[i].z);
		num_add_1++;
		if((p - odom.pos).norm() > max_range_) continue;
		num_add_2++;
		PointT center = acquireCenter(points[i]);
		if(map_.if_exist(center.x, center.y, center.z)){
			PointT *cur_p = map_.get_data(center.x, center.y, center.z);
			cur_dist = calDist(*cur_p, center);
			new_dist = calDist(points[i], center);
			if(new_dist < cur_dist) *cur_p = points[i];
		} 
		else{
			map_.insert(center.x, center.y, center.z, points[i]);
			num_add_3++;
		}
	}
}

template <typename PointT> typename GridMap<PointT>::PointVec
GridMap<PointT>::getCenterNeibours(PointT& center, int range_index){
	if(range_index < 0) range_index = 1;
	PointVec neibours;
	PointT point;
	for(int x{-range_index}; x < range_index; x++){
		point.x = center.x + x * resolution_;
		for(int y{-range_index}; y  < range_index; y++){
			point.y = center.y + y * resolution_;
			for(int z{-range_index}; z < range_index; z++){
				point.z = center.z + z * resolution_;
				neibours.push_back(point);
			}
		}
	}
	return neibours;
}

template <typename PointT> void
GridMap<PointT>::deletePoints(PointVec &points, OdomType odom){
	assert(is_map_param_set_ && "[occupied_map] grid-map param unset");
	for(size_t i{0}; i < points.size(); i++){
		if(!isInsideBox(points[i]))  continue;
		Eigen::Vector3f p(points[i].x, points[i].y, points[i].z);
		if((p - odom.pos).norm() > max_range_) continue;

		PointT center = acquireCenter(points[i]);
		auto neibours = getCenterNeibours(center, 10);
		for(auto& p : neibours) map_.erase(p.x, p.y, p.z);
	}
}

template <typename PointT> void
GridMap<PointT>::getPointsToVec(PointVec& vec, int down_rate, BoxPointType& box){
	if(down_rate <= 0) down_rate = 1;
	std::vector<PointT> vec_temp;
	map_.all_data(vec_temp);
	vec.clear();
	for(size_t index{0}; index < vec_temp.size(); ++index){
		if(index % down_rate != 0) continue;
		PointT p = vec_temp[index];
		if(p.x < box.vertex_min[0] || p.x > box.vertex_max[0]) continue;
		if(p.y < box.vertex_min[1] || p.y > box.vertex_max[1]) continue;
		if(p.z < box.vertex_min[2] || p.z > box.vertex_max[2]) continue;
		vec.push_back(p);
	}
}

template <typename PointT> void
GridMap<PointT>::getCentersToVec(PointVec& vec, int down_rate){
	if(down_rate <= 0) down_rate = 1;
	std::vector<PointT> vec_temp;
	map_.all_data(vec_temp);
	vec.clear();
	for(size_t index{0}; index < vec_temp.size(); ++index)
		if(index % down_rate == 0) vec.push_back(acquireCenter(vec_temp[index]));  //返回中心
}

template <typename PointT> void
GridMap<PointT>::getPointsToCloud(PointCloud::Ptr& cloud){
	std::vector<PointT> vec_temp;
	map_.all_data(vec_temp);
	cloud->clear();
	for(auto& p : vec_temp) cloud->push_back(p);
}

template <typename PointT> inline void
GridMap<PointT>::slideMap(BoxPointType new_box){
	map_.erase_data_out_of_range(new_box.vertex_min[0], new_box.vertex_max[0],
		new_box.vertex_min[1], new_box.vertex_max[1], new_box.vertex_min[2], new_box.vertex_max[2]);
	bounding_box_ = new_box;
}

template <typename PointT> inline void
GridMap<PointT>::slideMap(OdomType odom){
	if(!is_center_initialized_){
        map_center_ = odom.pos;
        is_center_initialized_ = true;
		return;
    }
	Eigen::Vector3f diff = odom.pos - map_center_;
	for(int i{0}; i < 3; i++){
		if(std::fabs(diff(i)) <= sliding_threshold_) diff(i) = 0;
		else diff(i) = std::ceil(diff(i) / resolution_ + EPSS) * resolution_;	
	}
	if(diff.norm() > EPSS){
        BoxPointType new_bbx = bounding_box_;
        for(int i{0}; i < 3; i++){
            new_bbx.vertex_min[i] += diff(i);
            new_bbx.vertex_max[i] += diff(i);
        }
        map_center_ = map_center_ + diff;
        this->slideMap(new_bbx);
    }
}

template <typename PointT> bool
GridMap<PointT>::isOccupied(PointT point){
	if(!isInsideBox(point)) return false;
	PointT center = acquireCenter(point);
	return map_.if_exist(center.x, center.y, center.z);
}

template <typename PointT> void
GridMap<PointT>::clear(){
	map_.clear();
}

template <typename PointT> bool 
GridMap<PointT>::checkFOV(PointT point){
	if(point.x*point.x + point.y*point.y + point.z*point.z >= max_range_sqrt_) return false;
	float hor = std::atan2(point.y, point.x);
	float ver = std::atan2(point.z, std::sqrt((std::pow(point.x, 2) + std::pow(point.y, 2)))); 
	// debug::Debug().print("FOV_range_hor-ver", hor, ver, hor/M_PI*180.0, ver/M_PI*180.0);
	if(hor < FOV_range_(2) || hor > FOV_range_(3)) return false;
	if(ver < FOV_range_(0) || ver > FOV_range_(1)) return false;
	return true;
}

}

template class occupied_mapping::GridMap<occupied_mapping::PointType>;