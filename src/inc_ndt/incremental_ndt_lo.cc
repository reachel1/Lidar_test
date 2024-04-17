//
// Created by xiang on 2022/7/20.
//

#include "inc_ndt/incremental_ndt_lo.h"
#include "common/math_utils.h"
#include "common/timer/timer.h"

namespace sad {

void IncrementalNDTLO::AddCloud(CloudPtr scan, SE3& pose, bool use_guess) {
    if (first_frame_) {
        // 第一个帧，直接加入local map
        pose = SE3();
        last_kf_pose_ = pose;
        ndt_.AddCloud(scan);
        first_frame_ = false;
        estimated_poses_.emplace_back(pose);
        pose_save.emplace_back(pose);
        return;
    }

    // 此时local map位于NDT内部，直接配准即可
    SE3 guess;
    ndt_.SetSource(scan);
    if (estimated_poses_.size() < 3) {
        ndt_.AlignNdt(guess);
    } else {
        if (!use_guess) {
            // 从最近两个pose来推断
            SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
            SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
            guess = T1 * (T2.inverse() * T1);
        } else {
            guess = pose;
        }

        ndt_.AlignNdt(guess);
    }

    pose = guess;
    estimated_poses_.emplace_back(pose);

    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, guess.matrix().cast<float>());
    std::cout<<"TWL: "<<std::endl;
    std::cout<<guess.matrix().cast<float>()<<std::endl;
    
    if (IsKeyframe(pose)) {
        std::cout<<"Key frame!!!"<<std::endl;
        last_kf_pose_ = pose;
        cnt_frame_ = 0;
        // 放入ndt内部的local map
        ndt_.AddCloud(scan_world);
    }
    
    if(!use_guess){
        if (viewer_ != nullptr) {
            viewer_->SetPoseAndCloud(pose, scan_world);
        }
    }
    cnt_frame_++;
}

bool IncrementalNDTLO::IsKeyframe(const SE3& current_pose) {
    // std::cout<<"cnt_frame: "<<cnt_frame_<<std::endl;
    if (cnt_frame_ > 10) {
        return true;
    }

    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void IncrementalNDTLO::SaveMap(const std::string& map_path) {
    if (viewer_) {
        viewer_->SaveMap(map_path);
    }
}

void IncrementalNDTLO::SavePath(const std::string& pose_path) {
    if (pose_save.size() > 2 && pose_save.size() == time_queue.size()) {
        
        std::ofstream out_lidarodom;
        out_lidarodom.open(pose_path.c_str(), std::ios::out);
        
        out_lidarodom.precision(6);
        out_lidarodom.setf(std::ios::fixed, std::ios::floatfield);
    
        for(int i = 0;i < pose_save.size();i++){
            SE3 pose = pose_save.at(i);
            double timestamp = time_queue.at(i);
            Eigen::Quaterniond q_lidarodom;
            q_lidarodom = Eigen::Quaterniond(pose.rotationMatrix());
            q_lidarodom.normalize();
            out_lidarodom<<timestamp<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)<<" "
              <<q_lidarodom.x()<<" "<<q_lidarodom.y()<<" "<<q_lidarodom.z()<<" "<<q_lidarodom.w()<<std::endl;
        }
        LOG(INFO) << "save pose success1 !!!";
        pose_save.clear();
    }
    else if (estimated_poses_.size() > 2 && estimated_poses_.size() == time_queue.size()) {
        
        std::ofstream out_lidarodom;
        out_lidarodom.open(pose_path.c_str(), std::ios::out);
        
        out_lidarodom.precision(6);
        out_lidarodom.setf(std::ios::fixed, std::ios::floatfield);
    
        for(int i = 0;i < estimated_poses_.size();i++){
            SE3 pose = estimated_poses_.at(i);
            double timestamp = time_queue.at(i);
            Eigen::Quaterniond q_lidarodom;
            q_lidarodom = Eigen::Quaterniond(pose.rotationMatrix());
            q_lidarodom.normalize();
            out_lidarodom<<timestamp<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)<<" "
              <<q_lidarodom.x()<<" "<<q_lidarodom.y()<<" "<<q_lidarodom.z()<<" "<<q_lidarodom.w()<<std::endl;
        }
        LOG(INFO) << "save pose success2 !!!";
        estimated_poses_.clear();
    }
    else{
        LOG(INFO) << "error: pose size not equal to time size !!!"<<estimated_poses_.size()<<" "<<time_queue.size();
    }
}

}  // namespace sad