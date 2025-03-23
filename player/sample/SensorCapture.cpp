#pragma once

#include "SensorCapture.h"
#include "lock.h"
#include "Locator.h"
#include "timestamp.h"

// using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

extern "C" { HMODULE LoadLibraryA(LPCSTR lpLibFileName); }

typedef HRESULT(__cdecl* PFN_CREATEPROVIDER)(IResearchModeSensorDevice**);

SensorCapture* SensorCapture::instance = nullptr;

static SRWLOCK g_lock;
static SpatialCoordinateSystem g_world_override = nullptr;
static SpatialLocator g_locator = nullptr;
static SpatialStationaryFrameOfReference g_referenceFrame = nullptr;
static SpatialCoordinateSystem g_world = nullptr;

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

//winrt::Windows::Foundation::Numerics::float4x4 Locator_Locate(winrt::Windows::Perception::PerceptionTimestamp const& timestamp, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world)
//{
//    auto location = locator.TryLocateAtTimestamp(timestamp, world);
//    return location ? (make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position())) : winrt::Windows::Foundation::Numerics::float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//}

winrt::Windows::Foundation::Numerics::float4x4 Locator_GetTransformTo(SpatialCoordinateSystem const& src, SpatialCoordinateSystem const& dst)
{
    auto location = src.TryGetTransformTo(dst);
    return location ? location.Value() : winrt::Windows::Foundation::Numerics::float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void SensorCapture::Locator_Initialize()
{
    InitializeSRWLock(&g_lock);

    g_locator = SpatialLocator::GetDefault();
    g_referenceFrame = g_locator.CreateStationaryFrameOfReferenceAtCurrentLocation();
    g_world = g_referenceFrame.CoordinateSystem();
    int a = 0;
    OutputDebugString(L"AAA");

}

winrt::Windows::Foundation::Numerics::float4x4 SensorCapture::Locator_Locate(winrt::Windows::Perception::PerceptionTimestamp const& timestamp, winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world)
{
    auto location = locator.TryLocateAtTimestamp(timestamp, world);
    return location ? (make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position())) : winrt::Windows::Foundation::Numerics::float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

winrt::Windows::Foundation::Numerics::float4x4 SensorCapture::Locator_GetTransformTo(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& src, winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& dst)
{
    auto location = src.TryGetTransformTo(dst);
    return location ? location.Value() : winrt::Windows::Foundation::Numerics::float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

winrt::Windows::Perception::Spatial::SpatialCoordinateSystem SensorCapture::Locator_GetWorldCoordinateSystem()
{
    SRWLock srw(&g_lock, false);
    return g_world_override ? g_world_override : g_world;
}

void SensorCapture::Locator_OverrideWorldCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& scs)
{
    SRWLock srw(&g_lock, true);
    g_world_override = scs;
}

winrt::Windows::Perception::Spatial::SpatialCoordinateSystem SensorCapture::Locator_SanitizeSpatialCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& scs)
{
    winrt::Windows::Perception::Spatial::Preview::SpatialGraphInteropFrameOfReferencePreview sgiforp = winrt::Windows::Perception::Spatial::Preview::SpatialGraphInteropPreview::TryCreateFrameOfReference(scs);
    return sgiforp ? sgiforp.CoordinateSystem() : nullptr;
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
    g_locator = SpatialGraphInteropPreview::CreateLocatorForNode(rigNodeId);
    //g_world = Locator_GetWorldCoordinateSystem();

    //this->g_locator = SpatialGraphInteropPreview::CreateLocatorForNode(rigNodeId);
    //g_ready = true;
    OutputDebugString(L"ego5");
    return;
}
void SensorCapture::StartStreaming()
{
    this->depthSensor->OpenStream();
    OutputDebugString(L"STR0");
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

    //winrt::Windows::Perception::Spatial::SpatialCoordinateSystem world = Locator_GetWorldCoordinateSystem();
    this->hololens_location = Locator_Locate(Timestamp_QPCToPerception(timestamp.SensorTicks), g_locator, g_world);

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

void SensorCapture::SendUInt16Array(UINT16 const* array, winrt::Windows::Foundation::Uri uri) {

    try {
        // Calculate the size of the array
        size_t size = 512*512;
        std::wcerr << L"arraysize: " << size << std::endl;
        // Create an HttpClient
        winrt::Windows::Web::Http::HttpClient client;

        //size = 125;
        std::wstring jsonString = L"{\"data\": [";
        for (size_t i = 0; i < size; ++i) {
            jsonString += std::to_wstring(array[i]);
            if (i < size - 1) {
                jsonString += L",";
            }
        }
        jsonString += L"]}";

        OutputDebugString(jsonString.c_str());

        // Create an HTTP StringContent with the JSON string
        winrt::Windows::Web::Http::HttpStringContent content(
            jsonString,
            winrt::Windows::Storage::Streams::UnicodeEncoding::Utf8,
            L"application/json"
        );

        // Create the URI
        

        // Send the POST request asynchronously
        client.PostAsync(uri, content);

        OutputDebugString(L"sent array");
    }
    catch (const winrt::hresult_error& ex) {
        std::wcerr << L"Error: " << ex.message().c_str() << std::endl;
    }
}

winrt::Windows::Foundation::Numerics::float4x4 SensorCapture::GetLocation()
{
    return this->hololens_location;
}

void SensorCapture::SendFloat4x4Matrix(winrt::Windows::Foundation::Numerics::float4x4 matrix,
    winrt::Windows::Foundation::Uri uri) {
    try {
        // Create an HttpClient
        winrt::Windows::Web::Http::HttpClient client;

        // Create JSON structure for the matrix
        std::wstring jsonString = L"{\"matrix\": [";

        // float4x4 has m11 through m44 members representing the 4x4 matrix elements
        // First row
        jsonString += L"[" + std::to_wstring(matrix.m11) + L","
            + std::to_wstring(matrix.m12) + L","
            + std::to_wstring(matrix.m13) + L","
            + std::to_wstring(matrix.m14) + L"],";

        // Second row
        jsonString += L"[" + std::to_wstring(matrix.m21) + L","
            + std::to_wstring(matrix.m22) + L","
            + std::to_wstring(matrix.m23) + L","
            + std::to_wstring(matrix.m24) + L"],";

        // Third row
        jsonString += L"[" + std::to_wstring(matrix.m31) + L","
            + std::to_wstring(matrix.m32) + L","
            + std::to_wstring(matrix.m33) + L","
            + std::to_wstring(matrix.m34) + L"],";

        // Fourth row
        jsonString += L"[" + std::to_wstring(matrix.m41) + L","
            + std::to_wstring(matrix.m42) + L","
            + std::to_wstring(matrix.m43) + L","
            + std::to_wstring(matrix.m44) + L"]";

        jsonString += L"]}";

        OutputDebugString(jsonString.c_str());

        // Create an HTTP StringContent with the JSON string
        winrt::Windows::Web::Http::HttpStringContent content(
            jsonString,
            winrt::Windows::Storage::Streams::UnicodeEncoding::Utf8,
            L"application/json"
        );

        // Send the POST request asynchronously
        client.PostAsync(uri, content);

        OutputDebugString(L"sent matrix");
    }
    catch (const winrt::hresult_error& ex) {
        std::wcerr << L"Error: " << ex.message().c_str() << std::endl;
    }
}


