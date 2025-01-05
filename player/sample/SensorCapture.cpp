#pragma once

#include "SensorCapture.h"
#include "lock.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

extern "C" { HMODULE LoadLibraryA(LPCSTR lpLibFileName); }

typedef HRESULT(__cdecl* PFN_CREATEPROVIDER)(IResearchModeSensorDevice**);


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

static uint32_t const g_sensor_count = sizeof(g_sensor_lut) / sizeof(ResearchModeSensorType);

static HMODULE g_hrResearchMode = NULL; // FreeLibrary
static IResearchModeSensorDevice* g_pSensorDevice = NULL; // Release
static IResearchModeSensorDeviceConsent* g_pSensorDeviceConsent = NULL; // Release
static HANDLE g_camera_consent_event = NULL; // CloseHandle
static HANDLE g_imu_consent_event = NULL; // CloseHandle
static ResearchModeSensorConsent g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
static ResearchModeSensorConsent g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;
static IResearchModeSensor* g_sensors[g_sensor_count]; // Release
static SpatialLocator g_locator = nullptr;
static bool g_ready = false;

static void ResearchMode_CameraAccessCallback(ResearchModeSensorConsent consent)
{
	g_camera_consent_value = consent;
	SetEvent(g_camera_consent_event);
}

void ResearchMode_Startup()
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
	g_imu_consent_event    = CreateEvent(NULL, TRUE, FALSE, NULL);
    OutputDebugString(L"ego3");
	g_pSensorDeviceConsent->RequestCamAccessAsync(ResearchMode_CameraAccessCallback);

	for (uint32_t sensor_index = 0; sensor_index < g_sensor_count; ++sensor_index) { g_pSensorDevice->GetSensor(g_sensor_lut[sensor_index], &(g_sensors[sensor_index])); }

	g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception));
    OutputDebugString(L"ego4");
	pSensorDevicePerception->GetRigNodeId(&rigNodeId);
	pSensorDevicePerception->Release();

	g_locator = SpatialGraphInteropPreview::CreateLocatorForNode(rigNodeId);
	g_ready   = true;
    OutputDebugString(L"ego5");
	return;
    }

std::tuple<UINT16 const*, UINT16 const*> GetDepth()
{
    OutputDebugString(L"SEN0");
    IResearchModeSensor* sensor = ResearchMode_GetSensor(ResearchModeSensorType::DEPTH_AHAT);
    IResearchModeSensorFrame* pSensorFrame; // Release
    OutputDebugString(L"SEN1");
    sensor->OpenStream();
    OutputDebugString(L"SEN2");
    sensor->GetNextBuffer(&pSensorFrame); // block

    IResearchModeSensorDepthFrame* pDepthFrame; // Release
    ResearchModeSensorTimestamp timestamp;

    UINT16 const* pDepth;
    UINT16 const* pAbImage;

    size_t nDepthCount;
    size_t nAbCount;

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    pDepthFrame->Release();

    int myVariable = 42;


    pSensorFrame->Release();
    sensor->CloseStream();
    OutputDebugString(L"SEN3");
    std::tuple<UINT16 const*, UINT16 const*> IRsensors = std::make_tuple(pDepth, pAbImage);
    return IRsensors;
}

IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type)
{
    return g_sensors[type];
}


