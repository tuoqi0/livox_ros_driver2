
#include "parse_cfg_file.h"

#include <iostream>
#include <cstdio>
#include <arpa/inet.h>

namespace livox_ros {

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}

bool ParseCfgFile::ParseSummaryInfo(LidarSummaryInfo& lidar_summary_info) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout << "parse summary info failed, can not open file: " << path_ << std::endl;
    return false;
  }

  char read_buffer[kMaxBufferSize];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  rapidjson::Document doc;
  do {
    if (doc.ParseStream(config_file).HasParseError()) {
      break;
    }
    if (!doc.HasMember("lidar_summary_info") || !doc["lidar_summary_info"].IsObject()) {
      break;
    }
    const rapidjson::Value &object = doc["lidar_summary_info"];
    if (!object.HasMember("lidar_type") || !object["lidar_type"].IsUint()) {
      break;
    }
    lidar_summary_info.lidar_type = static_cast<uint8_t>(object["lidar_type"].GetUint());
    std::fclose(raw_file);
    return true;
  } while (false);

  std::cout << "parse lidar type failed." << std::endl;
  std::fclose(raw_file);
  return false;
}

} // namespace livox_ros

