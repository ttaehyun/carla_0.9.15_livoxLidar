// LivoxLidarMeasurement.h
#pragma once

#include "carla/Debug.h"
#include "carla/rpc/Location.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/s11n/LivoxLidarSerializer.h"

namespace carla {
namespace sensor {
namespace data {

  class LivoxLidarMeasurement : public Array<data::LivoxLidarDetection> {
    static_assert(sizeof(data::LivoxLidarDetection) == 4u * sizeof(float), "Location size mismatch");
    using Super = Array<data::LivoxLidarDetection>;

  protected:
    using Serializer = s11n::LivoxLidarSerializer;
    friend Serializer;

    explicit LivoxLidarMeasurement(RawData &&data)
        : Super(std::move(data), [](const RawData &d) {
            return Serializer::GetHeaderOffset(d);
          }) {}

  private:
    auto GetHeader() const {
      return Serializer::DeserializeHeader(Super::GetRawData());
    }

  public:
    auto GetHorizontalAngle() const {
      return GetHeader().GetHorizontalAngle();
    }

    auto GetChannelCount() const {
      return GetHeader().GetChannelCount();
    }

    auto GetPointCount(size_t channel) const {
      return GetHeader().GetPointCount(channel);
    }
  };

} // namespace data
} // namespace sensor
} // namespace carla
