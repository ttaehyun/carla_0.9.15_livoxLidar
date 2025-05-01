// LivoxLidarSerializer.h
#pragma once

#include "carla/Debug.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/data/LivoxLidarData.h"

namespace carla {
namespace sensor {

  class SensorData;

  namespace s11n {

    class LivoxLidarHeaderView {
      using Index = data::LivoxLidarData::Index;

    public:
      float GetHorizontalAngle() const {
        return reinterpret_cast<const float &>(_begin[Index::HorizontalAngle]);
      }

      uint32_t GetChannelCount() const {
        return _begin[Index::ChannelCount];
      }

      uint32_t GetPointCount(size_t channel) const {
        DEBUG_ASSERT(channel < GetChannelCount());
        return _begin[Index::SIZE + channel];
      }

    private:
      friend class LivoxLidarSerializer;

      explicit LivoxLidarHeaderView(const uint32_t *begin) : _begin(begin) {
        DEBUG_ASSERT(_begin != nullptr);
      }

      const uint32_t *_begin;
    };

    class LivoxLidarSerializer {
    public:
      static LivoxLidarHeaderView DeserializeHeader(const RawData &data) {
        return LivoxLidarHeaderView{reinterpret_cast<const uint32_t *>(data.begin())};
      }

      static size_t GetHeaderOffset(const RawData &data) {
        auto View = DeserializeHeader(data);
        return sizeof(uint32_t) * (View.GetChannelCount() + data::LivoxLidarData::Index::SIZE);
      }

      template <typename Sensor>
      static Buffer Serialize(
          const Sensor &sensor,
          const data::LivoxLidarData &data,
          Buffer &&output);

      static SharedPtr<SensorData> Deserialize(RawData &&data);
    };

    // template <typename Sensor>
    // inline Buffer LivoxLidarSerializer::Serialize(
    //     const Sensor &,
    //     const data::LivoxLidarData &data,
    //     Buffer &&output) {
    //   std::array<boost::asio::const_buffer, 2u> seq = {
    //       boost::asio::buffer(data._header),
    //       boost::asio::buffer(data._points)};
    //   output.copy_from(seq);
    //   return std::move(output);
    // }
    template <typename Sensor>
    inline Buffer LivoxLidarSerializer::Serialize(
        const Sensor &,
        const data::LivoxLidarData &data,
        Buffer &&output) {
      
      //std::cout << "[Serialize] START ------------------------" << std::endl;
      //std::cout << "[Serialize] _header.size() = " << data._header.size() << std::endl;
      //std::cout << "[Serialize] _points.size() = " << data._points.size() << std::endl;
      //std::cout << "[Serialize] _points capacity = " << data._points.capacity() << std::endl;

      // for (size_t i = 0; i < data._header.size(); ++i) {
      //   std::cout << "[Serialize] _header[" << i << "] = " << data._header[i] << std::endl;
      // }

      //std::cout << "[Serialize] calling asio::buffer..." << std::endl;

      std::array<boost::asio::const_buffer, 2u> seq = {
          boost::asio::buffer(data._header),
          boost::asio::buffer(data._points)};
      
      //std::cout << "[Serialize] calling output.copy_from..." << std::endl;

      output.copy_from(seq);
      
      //std::cout << "[Serialize] DONE ------------------------" << std::endl;
      return std::move(output);
    }

  } // namespace s11n
} // namespace sensor
} // namespace carla
