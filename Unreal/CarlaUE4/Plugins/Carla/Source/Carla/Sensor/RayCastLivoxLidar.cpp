// RayCastLivoxLidar.cpp
#include "Carla/Sensor/RayCastLivoxLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include <PxScene.h>
#include <cmath>
#include <fstream>
#include <sstream>

FActorDefinition ARayCastLivoxLidar::GetSensorDefinition()
{
  // FActorDefinition Definition;
  // Definition.Id = TEXT("sensor.lidar.ray_cast_livox");
  // Definition.Tags = TEXT("Lidar,Livox");

  // FActorVariation Channels;
  // Channels.Id = TEXT("channels");
  // Channels.Type = EActorAttributeType::Int;
  // Channels.RecommendedValues = {TEXT("1")};

  // FActorVariation Range;
  // Range.Id = TEXT("range");
  // Range.Type = EActorAttributeType::Float;
  // Range.RecommendedValues = {TEXT("260")};

  // FActorVariation PointsPerSecond;
  // PointsPerSecond.Id = TEXT("points_per_second");
  // PointsPerSecond.Type = EActorAttributeType::Int;
  // PointsPerSecond.RecommendedValues = {TEXT("200000")};

  // FActorVariation DropOffIntensityLimit;
  // DropOffIntensityLimit.Id = TEXT("dropoff_intensity_limit");
  // DropOffIntensityLimit.Type = EActorAttributeType::Float;
  // DropOffIntensityLimit.RecommendedValues = {TEXT("0")};

  // FActorVariation DropOffAtZeroIntensity;
  // DropOffAtZeroIntensity.Id = TEXT("dropoff_zero_intensity");
  // DropOffAtZeroIntensity.Type = EActorAttributeType::Float;
  // DropOffAtZeroIntensity.RecommendedValues = {TEXT("0")};

  // FActorVariation DropOffGenRate;
  // DropOffGenRate.Id = TEXT("dropoff_general_rate");
  // DropOffGenRate.Type = EActorAttributeType::Float;
  // DropOffGenRate.RecommendedValues = {TEXT("0.3")};

  // FActorVariation StdDevLidar;
  // StdDevLidar.Id = TEXT("noise_stddev");
  // StdDevLidar.Type = EActorAttributeType::Float;
  // StdDevLidar.RecommendedValues = {TEXT("0")};

  // FActorVariation Decay;
  // Decay.Id = TEXT("decay_time");
  // Decay.Type = EActorAttributeType::Float;
  // Decay.RecommendedValues = {TEXT("1")};

  // FActorVariation LidarType;
  // LidarType.Id = TEXT("lidar_type");
  // LidarType.Type = EActorAttributeType::Float;
  // LidarType.RecommendedValues = {TEXT("0")};

  // Definition.Variations.Append({
  //   Channels,
  //   Range,
  //   PointsPerSecond,
  //   DropOffIntensityLimit,
  //   DropOffAtZeroIntensity,
  //   DropOffGenRate,
  //   StdDevLidar,
  //   Decay,
  //   LidarType
  // });

  // return Definition;
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_livox"));
}

ARayCastLivoxLidar::ARayCastLivoxLidar(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  //RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);
}

void ARayCastLivoxLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLivoxLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LivoxLidarData = FLivoxLidarData(Description.Channels);
  LivoxCsvInfo = LoadLivoxCSV();
  LivoxSize = LivoxCsvInfo.size();
  LivoxCount = 0;

  PointsPerChannel.resize(Description.Channels);

  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();

  CreateLasers(); // optional depending on pattern use
}

void ARayCastLivoxLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  auto DataStream = GetDataStream(*this);
  auto SensorTransform = DataStream.GetSensorTransform();

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    DataStream.SerializeAndSend(*this, LivoxLidarData, DataStream.PopBufferFromPool());
    
  }
}

void ARayCastLivoxLidar::CreateLasers()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) / static_cast<float>(NumberOfLasers - 1);

  LaserAngles.Empty(NumberOfLasers);
  for (auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle = Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    LaserAngles.Emplace(VerticalAngle);
  }

  clock_t start = clock();
  LivoxCsvInfo = LoadLivoxCSV();
  clock_t finish = clock();
  double TIME = finish - start;
  LivoxSize = LivoxCsvInfo.size();
}

void ARayCastLivoxLidar::SimulateLidar(float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLivoxLidar::SimulateLidar);
  const uint32 ChannelCount = Description.Channels;

  float decayTime = Description.Decay;
  const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(float(LivoxSize) * DeltaTime * decayTime);

  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
      LogCarla,
      Warning,
      TEXT("%s: no points requested this frame, try increasing the number of points per second."),
      *GetName());
    return;
  }

  check(ChannelCount == LaserAngles.Num());

  const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(
    LivoxLidarData.GetHorizontalAngle());
  const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov * DeltaTime;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel)
    {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
      FCollisionQueryParams TraceParams(FName(TEXT("Laser_Trace")), true, this);
      TraceParams.bTraceComplex = true;
      TraceParams.bReturnPhysicalMaterial = false;

      int i_count = LivoxCount;
      int i_limit = i_count + PointsToScanWithOneLaser;
      int RayCheck = 0;

      for (int i = i_count; i < i_limit; i++)
      {
        if (i >= LivoxSize)
        {
          i_count = 0;
          i = 0;
          i_limit = i_limit - LivoxSize;
          LivoxCount = i;
          continue;
        }

        FHitResult HitResult;
        float simTime = LivoxCsvInfo[i][0];
        float livoxTimeStamp = (simTime - std::floor(simTime)) * 0.1f;
        float Azimuth = LivoxCsvInfo[i][1];
        float Height = LivoxCsvInfo[i][2] - 90;
        int LineIndex = LivoxCsvInfo[i][3];

        const float VertAngle = Height;
        const float HorizAngle = Azimuth;

        const bool PreprocessResult = RayPreprocessCondition[idxChannel][RayCheck];
        ++RayCheck;

        if (PreprocessResult && ShootLaserLivox(VertAngle, HorizAngle, HitResult, TraceParams, LineIndex, livoxTimeStamp))
        {
          RecordedHits[idxChannel].emplace_back(HitResult);
        }
        LivoxCount = i;
      }
    });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf);

  const float HorizontalAngle = carla::geom::Math::ToRadians(
    std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  LivoxLidarData.SetHorizontalAngle(HorizontalAngle);
}

bool ARayCastLivoxLidar::ShootLaserLivox(float VerticalAngle, float HorizontalAngle, FHitResult &HitResult, FCollisionQueryParams &TraceParams, int LineIndex, float livoxTimeStamp) const
{
  FTransform ActorTransf = GetTransform();
  FVector LidarLoc = ActorTransf.GetLocation();
  FRotator LidarRot = ActorTransf.Rotator();
  FRotator LaserRot(VerticalAngle, HorizontalAngle, 0);
  FRotator FinalRot = UKismetMathLibrary::ComposeRotators(LaserRot, LidarRot);

  FVector EndTrace = Description.Range * UKismetMathLibrary::GetForwardVector(FinalRot) + LidarLoc;

  GetWorld()->ParallelLineTraceSingleByChannel(
    HitResult,
    LidarLoc,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam);

  HitResult.ElementIndex = LineIndex;
  HitResult.Time = livoxTimeStamp;
  return HitResult.bBlockingHit;
}

float ARayCastLivoxLidar::ComputeIntensity(const FHitResult &HitResult) const
{
  const float distance = HitResult.Distance;
  const float atten = exp(-Description.AtmospAttenRate * distance);
  return atten;
}

ARayCastLivoxLidar::FLivoxDetection ARayCastLivoxLidar::ComputeDetection(const FHitResult &HitInfo, const FTransform &SensorTransf) const
{
  FLivoxDetection Detection;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitInfo.ImpactPoint);
  Detection.intensity = HitInfo.ElementIndex + HitInfo.Time;
  return Detection;
}

bool ARayCastLivoxLidar::PostprocessDetection(FLivoxDetection &Detection) const
{
  if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
    auto noise = Detection.point.MakeUnitVector() * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
    Detection.point += noise;
  }
  float intensity = Detection.intensity;
  return (intensity > Description.DropOffIntensityLimit) ||
         (RandomEngine->GetUniformFloat() < DropOffAlpha * intensity + DropOffBeta);
}

void ARayCastLivoxLidar::ComputeAndSaveDetections(const FTransform &SensorTransform)
{
  for (uint32 i = 0; i < Description.Channels; ++i)
    PointsPerChannel[i] = RecordedHits[i].size();

  LivoxLidarData.ResetMemory(PointsPerChannel);
  for (uint32 i = 0; i < Description.Channels; ++i)
  {
    for (auto &hit : RecordedHits[i])
    {
      auto det = ComputeDetection(hit, SensorTransform);
      if (PostprocessDetection(det))
        LivoxLidarData.WritePointSync(det);
      else
        PointsPerChannel[i]--;
    }
  }
  LivoxLidarData.WriteChannelCount(PointsPerChannel);
}

std::vector<std::vector<float>> ARayCastLivoxLidar::LoadLivoxCSV()
{
    std::vector<std::vector<float>> info;
    std::ifstream file;
    switch (int(Description.LidarType)) {
    case 0: file.open("/home/a/LivoxCsv/horizon.csv"); break;
    case 1: file.open("/home/a/LivoxCsv/mid40.csv"); break;
    case 2: file.open("/home/a/LivoxCsv/avia.csv"); break;
    case 3: file.open("/home/a/LivoxCsv/tele.csv"); break;
    case 4: file.open("/home/a/LivoxCsv/mid360.csv"); break;
    case 5: file.open("/home/a/LivoxCsv/hap.csv"); break;
    default: file.open("/home/a/LivoxCsv/horizon.csv"); break;
    }

    std::string line;
    while (getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::vector<float> row;
        while (getline(ss, val, ',')) row.push_back(std::stof(val));
        info.push_back(row);
    }
    return info;
}

void ARayCastLivoxLidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
  RecordedHits.resize(Channels);
  RayPreprocessCondition.resize(Channels);
  for (uint32_t i = 0; i < Channels; ++i)
  {
    RecordedHits[i].clear();
    RecordedHits[i].reserve(MaxPointsPerChannel);
    RayPreprocessCondition[i].assign(MaxPointsPerChannel, true);
  }
}