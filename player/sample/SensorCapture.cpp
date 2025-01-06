#pragma once

#include "SensorCapture.h"
#include "lock.h"

extern "C" { HMODULE LoadLibraryA(LPCSTR lpLibFileName); }

typedef HRESULT(__cdecl* PFN_CREATEPROVIDER)(IResearchModeSensorDevice**);

SensorCapture* SensorCapture::instance = nullptr;

static ResearchModeSensorType const g_sensor_lut[] =
{
    LEFT_FRONT,
    LEFT_LEFT,
    RIGHT_FRONT,
    RIGHT_RIGHT,
    DEPTH_AHAT,
    DEPTH_LONG_THROW,
    IMU_ACCEL,
    IMU_GYRO,
    IMU_MAG
};

static void GlobalSensorCallback(ResearchModeSensorConsent consent)
{
    SensorCapture* scInstance = SensorCapture::instance;
    ResearchModeSensorConsent g_camera_consent_value = consent;
    HANDLE g_camera_consent_event = NULL;
    scInstance->g_camera_consent_value = consent;
    SetEvent(g_camera_consent_event);
    scInstance->g_camera_consent_event = g_camera_consent_event;
}

SensorCapture::SensorCapture()
{
    instance = this;
    int a = 0;
    this->g_sensor_count = sizeof(g_sensor_lut) / sizeof(ResearchModeSensorType);
    this->g_hrResearchMode = NULL; // FreeLibrary
    this->g_pSensorDevice = NULL; // Release
    this->g_pSensorDeviceConsent = NULL; // Release
    this->g_camera_consent_event = NULL; // CloseHandle
    this->g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
    this->g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;
    //this->g_sensors[g_sensor_count]; // Release

    this->g_ready = false;
}

void SensorCapture::ResearchMode_CameraAccessCallback(ResearchModeSensorConsent consent)
{
    
    this->g_camera_consent_value = consent;
    SetEvent(g_camera_consent_event);
}

void SensorCapture::ResearchMode_Startup()
{
    OutputDebugString(L"ResearchMode_Startup\n");
    IResearchModeSensorDevicePerception* pSensorDevicePerception; // Release
    GUID rigNodeId;
    HRESULT hr;
    OutputDebugString(L"ego1");
    g_hrResearchMode = LoadLibraryA("ResearchModeAPI");

    PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(g_hrResearchMode, "CreateResearchModeSensorDevice"));
    hr = pfnCreate(&g_pSensorDevice);
    OutputDebugString(L"ego2");
    g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&g_pSensorDeviceConsent));

    g_camera_consent_event = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_imu_consent_event = CreateEvent(NULL, TRUE, FALSE, NULL);
    OutputDebugString(L"ego3");

    g_pSensorDeviceConsent->RequestCamAccessAsync(GlobalSensorCallback);

    //for (uint32_t sensor_index = 0; sensor_index < g_sensor_count; ++sensor_index)
    //{
    //    g_pSensorDevice->GetSensor(g_sensor_lut[sensor_index], &g_sensors[sensor_index]);
    //}
    g_pSensorDevice->GetSensor(g_sensor_lut[4], &depthSensor);

    g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception));
    OutputDebugString(L"ego4");
    pSensorDevicePerception->GetRigNodeId(&rigNodeId);
    pSensorDevicePerception->Release();

    this->g_locator = SpatialGraphInteropPreview::CreateLocatorForNode(rigNodeId);
    g_ready = true;
    OutputDebugString(L"ego5");
    return;
}
void SensorCapture::StartStreaming()
{
    this->depthSensor->OpenStream();
}

std::tuple<UINT16 const*, UINT16 const*> SensorCapture::GetDepth()
{
    OutputDebugString(L"SEN0");

    OutputDebugString(L"SEN1");

    OutputDebugString(L"SEN2");

    this->depthSensor->GetNextBuffer(&pSensorFrame); // block

    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    ResearchModeSensorTimestamp timestamp;

    UINT16 const* pDepth;
    UINT16 const* pAbImage;

    size_t nDepthCount;
    size_t nAbCount;

    this->pSensorFrame->GetTimeStamp(&timestamp);
    this->pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    pDepthFrame->Release();

    OutputDebugString(L"SEN3");
    std::tuple<UINT16 const*, UINT16 const*> IRsensors = std::make_tuple(pDepth, pAbImage);
    return IRsensors;
}

void SensorCapture::ReleaseSensor()
{
    this->pSensorFrame->Release();
    this->depthSensor->CloseStream();
}




