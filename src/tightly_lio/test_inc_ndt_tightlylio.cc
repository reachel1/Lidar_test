#include <gflags/gflags.h>
#include <glog/logging.h>

#include "tightly_lio/lio_iekf.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/home/crq/dataset/1207_test09.bag", "path to rosbag");
DEFINE_string(dataset_type, "HY", "hy");                   // 数据集类型
DEFINE_string(config, "/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/config/hy_looselylio.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, false, "display map?");
DEFINE_bool(save_path, true, "save_path?");
DEFINE_bool(save_map, true, "save_map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    sad::LioIEKF lio;
    lio.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            sad::common::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "tightly lio");
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) {
            lio.IMUCallBack(imu);
            return true;
        })
        .Go();

    // if (FLAGS_save_map) {
    //     // 把地图存下来
    //     lio.inc_ndt_lo_->SaveMap("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_map3.pcd");
    // }
    // if (FLAGS_save_path) {
    //     // 把轨迹存下来
    //     lio.inc_ndt_lo_->SavePath("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_path4.txt");
    //     // lm.inc_ndt_lo_->SavePath("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_path5.txt");
    // }

    lio.SaveMap("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_map4.pcd");
    lio.SavePath("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_path5.txt");

    lio.Finish();
    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}