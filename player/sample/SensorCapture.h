
#pragma once

#include <ResearchModeApi.h>
#include <vector>

#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Web.Http.h>
#include <winrt/Windows.Data.Json.h>
#include <iostream>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>


using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

class SensorCapture
{
public:

    SensorCapture();
    winrt::Windows::Foundation::Numerics::float4x4 hololens_location;
    int OverrideWorldCoordinateSystem(void* scs_ptr);
    void Locator_Initialize();
    static SensorCapture* instance;
    uint32_t g_sensor_count;
    //std::optional<winrt::Windows::Perception::Spatial::SpatialLocator> g_locator;
    //winrt::Windows::Perception::Spatial::SpatialLocator g_locator;
    HMODULE g_hrResearchMode;
    IResearchModeSensorDevice* g_pSensorDevice;
    IResearchModeSensorDeviceConsent* g_pSensorDeviceConsent;
    std::wstring senderIp;
    HANDLE g_imu_consent_event;
    
    ResearchModeSensorConsent g_imu_consent_value;
    //IResearchModeSensor* g_sensors = nullptr;
    IResearchModeSensor* depthSensor;

    winrt::Windows::Foundation::Numerics::float4x4 Locator_Locate(winrt::Windows::Perception::PerceptionTimestamp const& timestamp, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
    winrt::Windows::Foundation::Numerics::float4x4 Locator_GetTransformTo(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& src, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& dst);
    winrt::Windows::Perception::Spatial::SpatialCoordinateSystem Locator_GetWorldCoordinateSystem();
    void Locator_OverrideWorldCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& scs);
    winrt::Windows::Perception::Spatial::SpatialCoordinateSystem Locator_SanitizeSpatialCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& scs);

    bool g_ready;

    int const RM_ZHT_WIDTH = 512;
    int const RM_ZHT_HEIGHT = 512;
    int const RM_ZHT_FPS = 45;
    int const RM_ZHT_MASK = 4090;
    int const RM_ZHT_PIXELS = RM_ZHT_WIDTH * RM_ZHT_HEIGHT;
    int const RM_ZHT_ZSIZE = RM_ZHT_PIXELS * sizeof(uint16_t);
    int const RM_ZHT_ABSIZE = RM_ZHT_PIXELS * sizeof(uint16_t);

    typedef void (*HOOK_RM_PROC)(IResearchModeSensorFrame*, void*);
    typedef void (*HOOK_RM_VLC_PROC)(BYTE const*, UINT64, UINT64, UINT64, UINT32, void*);
    typedef void (*HOOK_RM_ZHT_PROC)(UINT16 const*, UINT16 const*, UINT64, UINT64, void*);
    typedef void (*HOOK_RM_ZLT_PROC)(BYTE const*, UINT16 const*, UINT16 const*, UINT64, UINT64, void*);
    typedef void (*HOOK_RM_ACC_PROC)(AccelDataStruct const*, size_t, UINT64, UINT64, void*);
    typedef void (*HOOK_RM_GYR_PROC)(GyroDataStruct const*, size_t, UINT64, UINT64, void*);
    typedef void (*HOOK_RM_MAG_PROC)(MagDataStruct const*, size_t, UINT64, UINT64, void*);


    void ResearchMode_Startup();
    void StartStreaming();
    //void ResearchMode_Cleanup();
    std::tuple<UINT16 const*, UINT16 const*, float4x4> GetDepth();
    void ReleaseSensor();
    void SendUInt16Array(UINT16 const* array, winrt::Windows::Foundation::Uri uri);
    winrt::Windows::Foundation::Numerics::float4x4 GetLocation();
    void SendFloat4x4Matrix(winrt::Windows::Foundation::Numerics::float4x4 matrix, winrt::Windows::Foundation::Uri uri);

    IResearchModeSensorFrame* pSensorFrame; // Release
    void ResearchMode_CameraAccessCallback(ResearchModeSensorConsent consent);


    HANDLE g_camera_consent_event;
    ResearchModeSensorConsent g_camera_consent_value;
};

