#ifndef SAD_CH8_LASER_MAPPING_H
#define SAD_CH8_LASER_MAPPING_H

#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

/// 部分类直接使用ch7的结果
#include "imu_process/static_imu_init.h"
#include "loosely_lio/cloud_convert.h"
#include "loosely_lio/measure_sync.h"
#include "inc_ndt/ndt_inc.h"
#include "kf/iekf.hpp"
#include "tools/pcl_map_viewer.h"

#include "tools/ui/pangolin_window.h"

namespace sad {

class LioIEKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct Options {
        Options() {}
        bool save_motion_undistortion_pcd_ = false;  // 是否保存去畸变前后的点云
        bool with_ui_ = false;                        // 是否带着UI
        bool display_realtime_cloud_ = true;
    };

    LioIEKF(Options options = Options());
    ~LioIEKF() = default;

    /// init without ros
    bool Init(const std::string& config_yaml);

    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);

    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);

    /// 结束程序，退出UI
    void Finish();

    /// 获取当前姿态
    NavStated GetCurrentState() const { return ieskf_.GetNominalState(); }

    /// 获取当前扫描
    CloudPtr GetCurrentScan() const { return current_scan_; }

    void SaveMap(const std::string& map_path) {
        if (viewer_) {
            viewer_->SaveMap(map_path);
        }
    }

    void SavePath(const std::string& pose_path) {
        if (estimated_poses_.size() > 2 && estimated_poses_.size() == time_queue.size()) {
            
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

   private:
    bool LoadFromYAML(const std::string& yaml_file);

    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup& meas);

    /// 尝试让IMU初始化
    void TryInitIMU();

    /// 利用IMU预测状态信息
    /// 这段时间的预测数据会放入imu_states_里
    void Predict();

    /// 对measures_中的点云去畸变
    void Undistort();

    /// 执行一次配准和观测
    void Align();

    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;
    StaticIMUInit imu_init_;

    /// point clouds data
    FullCloudPtr scan_undistort_{new FullPointCloudType()};  // scan after undistortion
    CloudPtr current_scan_ = nullptr;

    /// NDT数据
    IncNdt3d ndt_;
    SE3 last_pose_;

    /// 可视化工具
    std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
    std::vector<SE3> estimated_poses_;  // 所有估计出来的pose，用于记录轨迹
    std::vector<double> time_queue;

    // flags
    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan
    std::vector<NavStated> imu_states_;
    IESKFD ieskf_;  // IESKF
    SE3 TIL_;       // Lidar与IMU之间外参

    Options options_;
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace sad

#endif  // FASTER_LIO_LASER_MAPPING_H