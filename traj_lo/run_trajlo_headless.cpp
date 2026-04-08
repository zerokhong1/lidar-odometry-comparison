/**
 * Headless runner for Traj-LO (no GUI/GLFW required)
 * Connects DataLoader -> odometry queue, runs without visualization.
 */
#include <trajlo/core/odometry.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {
  std::cout << "=================================================\n";
  std::cout << "  Traj-LO Headless Mode (no GUI)\n";
  std::cout << "=================================================\n";

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
    return 1;
  }

  traj::TrajConfig config;
  config.load(argv[1]);
  config.save_pose = true;

  traj::TrajLOdometry::Ptr odometry(new traj::TrajLOdometry(config));
  traj::DataLoader::Ptr dataLoader = traj::DatasetFactory::GetDatasetIo(config.type);

  // Wire the loader to the odometry queue (same as visualizer does)
  dataLoader->laser_queue = &odometry->laser_data_queue;

  // Start processing thread
  odometry->Start();

  // Feed data in a separate thread
  std::thread t_io(&traj::DataLoader::publish, dataLoader,
                   config.dataset_path, config.topic);

  std::cout << "Processing bag... (this may take a while)\n";
  t_io.join();

  // Wait for odometry to finish
  while (!odometry->isFinish) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "Done! Poses saved to: " << config.pose_file_path << "\n";
  return 0;
}
