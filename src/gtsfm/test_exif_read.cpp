/**
 * @file test_exif_read.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 02-26-2023
 * @copyright Copyright (c) 2023
 */

#include <fstream>
#include <iostream>
#include <map>

#include "base/image_reader.h"

using namespace colmap;

std::string ExtractImageName(const std::string& path) {
  auto s = path.find_last_of("/");
  auto e = path.find_last_of(".");
  return path.substr(s + 1, e - s - 1);
}

std::string ExtractFileName(const std::string& path) {
  auto s = path.find_last_of("/");
  return path.substr(s + 1, path.size() - s - 1);
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: ./test_exif_read image_path" << std::endl;
    return -1;
  }

  // Get image path.
  const std::string image_path(argv[1]);
  std::cout << "Image Path: " << image_path << std::endl;

  // Construct reader.
  ImageReaderOptions init_options;
  init_options.image_path = image_path;
  ImageReader im_reader(init_options, nullptr);
  const auto& options = im_reader.GetOptions();

  // std::ofstream fs;
  // fs.open("colmap_focal_length_" + ExtractFileName(image_path) + ".txt");

  // Read image.
  std::map<int, int> indexed_count;
  Bitmap* bitmap(new Bitmap());
  for (size_t image_index = 0; image_index < im_reader.NumImages();
       ++image_index) {
    const std::string image_path = options.image_list.at(image_index);
    bitmap->Read(image_path, false);
    // Extract focal length.
    double focal_length = 0.0;
    const int status = bitmap->ExifFocalLengthGTSFM(&focal_length);
    if (status == 0) {
      focal_length = options.default_focal_length_factor *
                     std::max(bitmap->Width(), bitmap->Height());
    }

    // Record status and count.
    if (indexed_count.count(status)) {
      ++indexed_count[status];
    } else {
      indexed_count[status] = 1;
    }

    // fs << ExtractImageName(image_path) << " " << focal_length << " " <<
    // status
    //  << "\n";
  }
  // fs.close();

  // Logging.

  std::cout << "(Status, Count)\n";
  int count = 0;
  for (const auto& m : indexed_count) {
    count += m.second;
    std::cout << "(" << m.first << ", " << m.second << ")\n";
  }
  std::cout << "(Total, Loaded) = (" << im_reader.NumImages() << ", " << count
            << ")\n";

  delete bitmap;
  return 0;
}