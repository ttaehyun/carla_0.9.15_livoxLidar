//LivoxLidar.h
#pragma once

#include "Carla/Sensor/RayCastLidar.h"
#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/LivoxLidarData.h>
#include <compiler/enable-ue4-macros.h>

#include "RayCastLivoxLidar.generated.h"

UCLASS()
class CARLA_API ARayCastLivoxLidar : public ARayCastLidar
{
    GENERATED_BODY()

    using FLivoxLidarData = carla::sensor::data::LivoxLidarData;
    using FLivoxDetection = carla::sensor::data::LivoxLidarDetection;

public:
    static FActorDefinition GetSensorDefinition();

    ARayCastLivoxLidar(const FObjectInitializer& ObjectInitializer);
    virtual void Set(const FActorDescription &ActorDescription) override;
    virtual void Set(const FLidarDescription &LidarDescription) override;
    virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

private:
    virtual void CreateLasers();
    virtual void SimulateLidar(float DeltaTime);

    bool ShootLaserLivox(float VerticalAngle, float HorizontalAngle, FHitResult &HitResult, FCollisionQueryParams &TraceParams, int LineIndex, float livoxTimeStamp) const;

    carla::sensor::data::LivoxLidarData LivoxLidarData;
    std::vector<std::vector<float>> LivoxCsvInfo;
    int LivoxCount = 0;
    int LivoxSize = 0;

    bool DropOffGenActive;
    float DropOffAlpha;
    float DropOffBeta;

    float ComputeIntensity(const FHitResult &HitResult) const;
    FLivoxDetection ComputeDetection(const FHitResult &HitInfo, const FTransform &SensorTransf) const;
    bool PostprocessDetection(FLivoxDetection &Detection) const;
    void ComputeAndSaveDetections(const FTransform &SensorTransform);
    
    void ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel);

    std::vector<std::vector<FHitResult>> RecordedHits;
    std::vector<std::vector<bool>> RayPreprocessCondition;
    std::vector<uint32_t> PointsPerChannel;

    std::vector<std::vector<float>> LoadLivoxCSV();
};