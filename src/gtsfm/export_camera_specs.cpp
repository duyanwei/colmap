/**
 * @file export_camera_spec.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 03-22-2023
 * @copyright Copyright (c) 2023
 */

#include <fstream>
#include <iostream>

#include "util/camera_specs.h"

using namespace colmap;

int main() {
  const camera_specs_t camera_specs = InitializeCameraSpecs();

  const std::string filename = "./sensor_database_colmap.csv";
  std::ofstream spec_file(filename);

  if (!spec_file.is_open()) {
    std::cerr << "Failed to open file: " << filename << "\n";
    return -1;
  }

  // Write header.
  spec_file << "CameraMaker,CameraModel,SensorWidth(mm)"
            << "\n";

  int count = 0;
  for (const auto& c : camera_specs) {
    for (const auto& v : c.second) {
      spec_file << c.first << "," << v.first << "," << v.second << "\n";
    }
    count++;

    // if (count > 0) {
    //   break;
    // }
  }

  spec_file.close();

  return 0;
}