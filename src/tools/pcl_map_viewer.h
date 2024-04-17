//
// Created by xiang on 2022/7/19.
//

#ifndef SLAM_IN_AUTO_DRIVING_PCL_MAP_VIEWER_H
#define SLAM_IN_AUTO_DRIVING_PCL_MAP_VIEWER_H

#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "common/point_types.h"
#include "common/point_cloud_utils.h"

namespace sad {

/// 基于pcl viewer的局部地图查看器
class PCLMapViewer {
   public:
    /// 构造函数，指定voxel size
    PCLMapViewer(const float& leaf_size, bool use_pcl_vis = true)
        : leaf_size_(leaf_size), tmp_cloud_(new PointCloudType), local_map_(new PointCloudType) {
        if (use_pcl_vis) {
            viewer_.reset(new pcl::visualization::PCLVisualizer());
            viewer_->addCoordinateSystem(10, "world");
        } else {
            viewer_ = nullptr;
        }
        scanvoxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
        mapvoxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    }

    /**
     * 增加一个Pose和它的点云（世界系）
     * @param pose
     * @param cloud_world
     */
    void SetPoseAndCloud(const SE3& pose, CloudPtr cloud_world) {
        std::cout<<"leaf size: "<<leaf_size_<<std::endl;
        tmp_cloud_->clear();
        scanvoxel_filter_.setInputCloud(cloud_world);
        scanvoxel_filter_.filter(*tmp_cloud_);
        tmp_cloud_->is_dense = false;
        std::cout<<"tmep cloud size: "<<tmp_cloud_->size()<<std::endl;
        *local_map_ += *tmp_cloud_;
        local_map_->is_dense = false;
        std::cout<<"local map before filter size: "<<local_map_->size()<<std::endl;
        mapvoxel_filter_.setInputCloud(local_map_);
        mapvoxel_filter_.filter(*local_map_);
        std::cout<<"local map after filter size: "<<local_map_->size()<<std::endl;

        if (viewer_ != nullptr) {
            
            viewer_->removePointCloud("local_map");
            viewer_->removeCoordinateSystem("vehicle");

            pcl::visualization::PointCloudColorHandlerGenericField<PointType> fieldColor(local_map_, "z");
            viewer_->addPointCloud<PointType>(local_map_, fieldColor, "local_map");

            Eigen::Affine3f T;
            T.matrix() = pose.matrix().cast<float>();
            viewer_->addCoordinateSystem(5, T, "vehicle");
            viewer_->spinOnce(1);
        }
        
        if (local_map_->size() > 600000) {
            leaf_size_ *= 1.26;
            mapvoxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            LOG(INFO) << "viewer set leaf size to " << leaf_size_;
        }
    }

    /// 存储地图至PCD文件
    void SaveMap(std::string path) {
        if (local_map_->size() > 0) {
            sad::SaveCloudToFile(path, *local_map_);
            LOG(INFO) << "save map to " << path;
        } else {
            LOG(INFO) << "map 是空的" << path;
        }
    }

    void Clean() {
        tmp_cloud_->clear();
        local_map_->clear();
    }

    void ClearAndResetLeafSize(const float leaf_size) {
        leaf_size_ = leaf_size;
        local_map_->clear();
        mapvoxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    }

   private:
    pcl::VoxelGrid<PointType> scanvoxel_filter_;
    pcl::VoxelGrid<PointType> mapvoxel_filter_;
    pcl::visualization::PCLVisualizer::Ptr viewer_ = nullptr;  // pcl viewer
    float leaf_size_ = 1.0;
    CloudPtr tmp_cloud_;  //
    CloudPtr local_map_;
};
}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_PCL_MAP_VIEWER_H
