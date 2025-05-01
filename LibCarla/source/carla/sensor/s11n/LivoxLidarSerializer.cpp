// LivoxLidarSerializer.cpp
#include "carla/sensor/data/LivoxLidarMeasurement.h"
#include "carla/sensor/s11n/LivoxLidarSerializer.h"
#include <iostream>  // For debug logging

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> LivoxLidarSerializer::Deserialize(RawData &&data) {
    auto view = DeserializeHeader(data);
    auto channel_count = view.GetChannelCount();
    auto total_bytes = data.size();

    //std::cout << "[LivoxDeserializer] ChannelCount = " << channel_count << std::endl;
    //std::cout << "[LivoxDeserializer] RawData size = " << total_bytes << " bytes" << std::endl;

    // 방어 코드 추가
    size_t expected_min_header = sizeof(uint32_t) * (data::LivoxLidarData::Index::SIZE + channel_count);
    if (channel_count == 0 || total_bytes < expected_min_header) {
      std::cerr << "[LivoxDeserializer] Invalid header or raw_data too small. Rejecting." << std::endl;
      return nullptr;
    }

    auto ptr = new data::LivoxLidarMeasurement{std::move(data)};
    //std::cout << "[LivoxDeserializer] Point count = " << ptr->size() << std::endl;

    if (ptr->size() == 0) {
      std::cerr << "[LivoxDeserializer] Empty point cloud. Skipping." << std::endl;
      return nullptr;
    }

    return SharedPtr<data::LivoxLidarMeasurement>(ptr);
  }

} // namespace s11n
} // namespace sensor
} // namespace carla