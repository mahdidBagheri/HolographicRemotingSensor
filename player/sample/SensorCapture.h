
#pragma once

#include <ResearchModeApi.h>
#include <vector>

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Web.Http.h>
#include <winrt/Windows.Data.Json.h>
#include <iostream>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

class SensorCapture
{
public:

    SensorCapture();
    static SensorCapture* instance;
    uint32_t g_sensor_count;
    std::optional<winrt::Windows::Perception::Spatial::SpatialLocator> g_locator;
    HMODULE g_hrResearchMode;
    IResearchModeSensorDevice* g_pSensorDevice;
    IResearchModeSensorDeviceConsent* g_pSensorDeviceConsent;
    std::wstring senderIp;
    HANDLE g_imu_consent_event;
    
    ResearchModeSensorConsent g_imu_consent_value;
    //IResearchModeSensor* g_sensors = nullptr;
    IResearchModeSensor* depthSensor;

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
    std::tuple<UINT16 const*, UINT16 const*> GetDepth();
    void ReleaseSensor();
    void SendUInt16Array(UINT16 const* array, winrt::Windows::Foundation::Uri uri);
    IResearchModeSensorFrame* pSensorFrame; // Release
    void ResearchMode_CameraAccessCallback(ResearchModeSensorConsent consent);

    //IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type);
    //winrt::Windows::Foundation::Numerics::float4x4 ResearchMode_GetRigNodeWorldPose(UINT64 host_ticks);
    //bool ResearchMode_WaitForConsent(IResearchModeSensor* sensor);
    //bool ResearchMode_GetIntrinsics(IResearchModeSensor* sensor, std::vector<float>& uv2x, std::vector<float>& uv2y, std::vector<float>& mapx, std::vector<float>& mapy, float K[4]);
    //bool ResearchMode_GetExtrinsics(IResearchModeSensor* sensor, DirectX::XMFLOAT4X4& extrinsics);
    //void ResearchMode_ExecuteSensorLoop(IResearchModeSensor* sensor, HOOK_RM_PROC hook, void* param, HANDLE event_stop);
    //void ResearchMode_ProcessSample_VLC(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_VLC_PROC hook, void* param);
    //void ResearchMode_ProcessSample_ZHT(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ZHT_PROC hook, void* param);
    //void ResearchMode_ProcessSample_ZLT(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ZLT_PROC hook, void* param);
    //void ResearchMode_ProcessSample_ACC(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ACC_PROC hook, void* param);
    //void ResearchMode_ProcessSample_GYR(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_GYR_PROC hook, void* param);
    //void ResearchMode_ProcessSample_MAG(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_MAG_PROC hook, void* param);

    //void ResearchMode_SetEyeSelection(bool enable);

    HANDLE g_camera_consent_event;
    ResearchModeSensorConsent g_camera_consent_value;
};

