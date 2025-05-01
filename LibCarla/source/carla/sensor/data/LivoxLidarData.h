// LivoxLidarData.h
#pragma once

#include "carla/rpc/Location.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/LidarData.h"

#include <cstdint>
#include <vector>

namespace carla {
namespace sensor {

  namespace s11n {
    class LivoxLidarSerializer;
    class LivoxLidarHeaderView;
  }

  namespace data {

    class LivoxLidarDetection {
    public:
      geom::Location point;
      float intensity;

      LivoxLidarDetection() : point(0.0f, 0.0f, 0.0f), intensity{0.0f} {}
      LivoxLidarDetection(float x, float y, float z, float intensity)
          : point(x, y, z), intensity{intensity} {}
      LivoxLidarDetection(geom::Location p, float intensity)
          : point(p), intensity{intensity} {}

      void WritePlyHeaderInfo(std::ostream &out) const {
        out << "property float32 x\n"
               "property float32 y\n"
               "property float32 z\n"
               "property float32 I";
      }

      void WriteDetection(std::ostream &out) const {
        out << point.x << ' ' << point.y << ' ' << point.z << ' ' << intensity;
      }
    };

    class LivoxLidarData : public SemanticLidarData {
    public:
      explicit LivoxLidarData(uint32_t ChannelCount = 0u)
          : SemanticLidarData(ChannelCount) {}

      LivoxLidarData &operator=(LivoxLidarData &&) = default;
      ~LivoxLidarData() = default;

      virtual void ResetMemory(std::vector<uint32_t> points_per_channel) {
        //DEBUG_ASSERT(GetChannelCount() > points_per_channel.size());
        auto ch = GetChannelCount();
        //std::cout << "[LivoxLidarData] ResetMemory: ChannelCount = " << ch << ", VectorSize = " << points_per_channel.size() << std::endl;
      
        if (ch == 0 || points_per_channel.empty()) {
          std::cerr << "[ERROR] LivoxLidarData: Invalid ChannelCount or points_per_channel empty!" << std::endl;
        }
        _header.resize(Index::SIZE + ch);
        std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());
        
        uint32_t total_points = std::accumulate(
          points_per_channel.begin(), points_per_channel.end(), 0u);
        // uint32_t total_points = static_cast<uint32_t>(
        //     std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));

        // After reserve()
        _points.clear();
        _points.reserve(total_points * 4);

        // Dummy point if empty
        if (total_points == 0) {
          std::cerr << "[WARN] LivoxLidarData: total_points == 0, inserting dummy point to avoid crash" << std::endl;
          _points.emplace_back(0.0f);
          _points.emplace_back(0.0f);
          _points.emplace_back(0.0f);
          _points.emplace_back(0.0f);
        }

        // ✅ 반드시 헤더 초기화 추가!
        float horiz_angle = 0.0f;
        std::memcpy(&_header[0], &horiz_angle, sizeof(uint32_t));  // HorizontalAngle
        _header[1] = ch;                                            // ChannelCount
      }

      void WritePointSync(LivoxLidarDetection &detection) {
        _points.emplace_back(detection.point.x);
        _points.emplace_back(detection.point.y);
        _points.emplace_back(detection.point.z);
        _points.emplace_back(detection.intensity);
      }

      virtual void WritePointSync(SemanticLidarDetection &detection) {
        (void)detection;
        DEBUG_ASSERT(false);
      }

    private:
      std::vector<float> _points;

      friend class s11n::LivoxLidarSerializer;
      friend class s11n::LivoxLidarHeaderView;
    };

  } // namespace data
}   // namespace sensor
} // namespace carla
