//
// Created by xiang on 2022/7/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "inc_ndt/incremental_ndt_lo.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/home/crq/dataset/1207_test09.bag", "path to rosbag");
DEFINE_string(dataset_type, "HY", "hy");  // 数据集类型
DEFINE_bool(use_ndt_nearby_6, true, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");
DEFINE_bool(save_path, true, "save_path?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    sad::IncrementalNDTLO::Options options;
    options.ndt3d_options_.nearby_type_ =
        FLAGS_use_ndt_nearby_6 ? sad::IncNdt3d::NearbyType::NEARBY6 : sad::IncNdt3d::NearbyType::CENTER;
    options.display_realtime_cloud_ = FLAGS_display_map;
    sad::IncrementalNDTLO ndt_lo(options);

    rosbag_io
        .AddAutoPointCloudHandle([&ndt_lo](sensor_msgs::PointCloud2::Ptr msg) -> bool {
            sad::common::Timer::Evaluate(
                [&]() {
                    SE3 pose;
                    ndt_lo.AddCloud(sad::VoxelCloud(sad::PointCloud2ToCloudPtr(msg)), pose);
                    ndt_lo.Savetime(msg->header.stamp.toSec());
                },
                "NDT registration");
            return true;
        })
        .Go();

    if (FLAGS_display_map) {
        // 把地图存下来
        ndt_lo.SaveMap("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_map.pcd");
    }
    if (FLAGS_save_path) {
        // 把地图存下来
        ndt_lo.SavePath("/home/crq/workspace_HY/catkin_openhy_lidar/openhy_lidar/data/1207test09_path.txt");
    }

    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
