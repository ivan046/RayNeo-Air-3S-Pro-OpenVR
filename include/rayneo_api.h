// RayNeo C API
// Public low-level interface for building and sending 64-byte protocol frames,
// managing a background service thread, and receiving events via callback or polling.
// Versioning: MAJOR changes break ABI, MINOR are backward compatible additions.
// Packed version format: (MAJOR << 16) | (MINOR & 0xFFFF)

#pragma once

#ifdef _WIN32
 #ifdef RAYNEO_BUILD
  #define RAYNEO_API __declspec(dllexport)
 #else
  #define RAYNEO_API __declspec(dllimport)
 #endif
#else
 #define RAYNEO_API
#endif

#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
#include <array>
#endif

#define RAYNEO_AIR_3S_PRO_VID 0x1BBB
#define RAYNEO_AIR_3S_PRO_PID 0xAF50
#define RAYNEO_GT_VID 0x3941
#define RAYNEO_GT_PID 0xAF50

// GT/Gemini runtime HID topology used by automatic selection. Explicit
// interface selection takes precedence.
#define RAYNEO_GT_HID_INTERFACE 5
#define RAYNEO_GT_HID_EP_OUT 0x04
#define RAYNEO_GT_HID_EP_IN 0x85
#define RAYNEO_GT_SENSOR_TICK_HZ 10000u
#define RAYNEO_GT_SENSOR_TICK_US 100u

// Add supported models here; discovery and the constexpr list share this table.
#define RAYNEO_SUPPORTED_DEVICES(X) \
    X(AIR_3S_PRO, RAYNEO_AIR_3S_PRO_VID, RAYNEO_AIR_3S_PRO_PID) \
    X(GT, RAYNEO_GT_VID, RAYNEO_GT_PID)

typedef struct RAYNEO_VidPid {
    uint16_t vid;
    uint16_t pid;
} RAYNEO_VidPid;

#define RAYNEO_COUNT_DEVICE(model, vid, pid) + 1
enum { RAYNEO_SUPPORTED_DEVICE_COUNT = 0 RAYNEO_SUPPORTED_DEVICES(RAYNEO_COUNT_DEVICE) };
#undef RAYNEO_COUNT_DEVICE

#ifdef __cplusplus
constexpr std::array<RAYNEO_VidPid, RAYNEO_SUPPORTED_DEVICE_COUNT> Rayneo_GetSupportedDevices()
{
#define RAYNEO_DEVICE_PAIR(model, vid, pid) {vid, pid},
    return {{ RAYNEO_SUPPORTED_DEVICES(RAYNEO_DEVICE_PAIR) }};
#undef RAYNEO_DEVICE_PAIR
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ---- Versioning ----
#define RAYNEO_API_VERSION_MAJOR 1
// Minor 1: added structured fields to RAYNEO_DeviceInfoMini (backward compatible: raw[] still first)
// Minor 2: added RAYNEO_EVENT_NOTIFY (sleep/wake notifications) and notify union member
// Minor 3: added Rayneo_SetTargetInterface and RAYNEO_NOTIFY_BUTTON_SPATIAL_MODE
// Minor 4: supported device table and Rayneo_Discovery.
// Minor 5: backward-compatible GT/Gemini calibration and fused-orientation API.
#define RAYNEO_API_VERSION_MINOR 5
#define RAYNEO_API_VERSION_PATCH 0

#define RAYNEO_STRINGIFY_IMPL(value) #value
#define RAYNEO_STRINGIFY(value) RAYNEO_STRINGIFY_IMPL(value)
#define RAYNEO_VERSION_STRING RAYNEO_STRINGIFY(RAYNEO_API_VERSION_MAJOR) \
    "." RAYNEO_STRINGIFY(RAYNEO_API_VERSION_MINOR) \
    "." RAYNEO_STRINGIFY(RAYNEO_API_VERSION_PATCH)

// Preserve the packed ABI version format; patch releases do not change it.
#define RAYNEO_API_VERSION ((RAYNEO_API_VERSION_MAJOR << 16) | (RAYNEO_API_VERSION_MINOR & 0xFFFF))

RAYNEO_API unsigned int Rayneo_GetApiVersion(void); // returns packed version

typedef struct RayneoContext__* RAYNEO_Context; // opaque handle

typedef enum RAYNEO_Result {
    RAYNEO_OK = 0,
    RAYNEO_ERR_GENERAL      = -1,
    RAYNEO_ERR_NO_DEVICE    = -2,
    RAYNEO_ERR_TIMEOUT      = -3,
    RAYNEO_ERR_IO           = -4,
    RAYNEO_ERR_BUSY         = -5,
    RAYNEO_ERR_UNSUPPORTED  = -6,
    RAYNEO_ERR_INVALID_ARG  = -7,
    RAYNEO_ERR_QUEUE_FULL   = -8
} RAYNEO_Result;

// Service flags (bitmask)
typedef enum RAYNEO_ServiceFlags {
    RAYNEO_SERVICE_FLAG_AUTODETECT = 1u << 0,
    RAYNEO_SERVICE_FLAG_ASYNC_IMU  = 1u << 1
} RAYNEO_ServiceFlags;

// Event types produced by the service thread.
typedef enum RAYNEO_EventType {
    RAYNEO_EVENT_DEVICE_ATTACHED = 0,
    RAYNEO_EVENT_DEVICE_DETACHED = 1,
    RAYNEO_EVENT_IMU_SAMPLE      = 2,
    RAYNEO_EVENT_DEVICE_INFO     = 3,
    RAYNEO_EVENT_ERROR           = 4,
    RAYNEO_EVENT_LOG             = 5,
    RAYNEO_EVENT_NOTIFY          = 6,  // new in 1.2 (e.g. sleep/wake)
    RAYNEO_NOTIFY_SLEEP          = 7,
    RAYNEO_NOTIFY_WAKE           = 8,
    RAYNEO_NOTIFY_BUTTON         = 9,
    RAYNEO_NOTIFY_BUTTON_VOLUME_UP      = 10,
    RAYNEO_NOTIFY_BUTTON_VOLUME_DOWN    = 11,
    RAYNEO_NOTIFY_BUTTON_BRIGHTNESS     = 12,
    RAYNEO_NOTIFY_IMU_ON          = 13,
    RAYNEO_NOTIFY_IMU_OFF         = 14,
    RAYNEO_NOTIFY_BUTTON_SPATIAL_MODE = 15,
} RAYNEO_EventType;

typedef struct RAYNEO_ImuSample {
    float    acc[3];
    float    gyroDps[3];
    float    gyroRad[3];
    float    magnet[3];
    float    temperature;
    float    psensor;
    float    lsensor;
    uint32_t tick;
    uint32_t count;
    uint8_t  flag;
    uint8_t  checksum;
    uint8_t  valid;      // 1 if filled
    uint8_t  reserved;   // future use
} RAYNEO_ImuSample;

// ---- GT/Gemini calibration and fused orientation (added in 1.5) ----
// Kept separate from RAYNEO_Event to preserve its ABI.

typedef struct RAYNEO_GtFactoryCalibration {
    float   transform[9];     // row-major 3x3 Tsb matrix from command 0x3C
    float   accelOffset[3];   // accelerometer offset Ta from command 0x3C
    uint8_t raw[48];
    uint8_t valid;
    uint8_t reserved[3];
} RAYNEO_GtFactoryCalibration;

#define RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C (-20)
#define RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C (60)
#define RAYNEO_GT_GYRO_BIAS_TABLE_COUNT (81)

typedef struct RAYNEO_GtGyroBiasTable {
    float   biasRadPerSec[RAYNEO_GT_GYRO_BIAS_TABLE_COUNT][3];
    uint8_t validEntry[RAYNEO_GT_GYRO_BIAS_TABLE_COUNT];
    uint8_t complete;
    uint8_t validCount;
    uint8_t reserved[2];
} RAYNEO_GtGyroBiasTable;

// "6D"/"9D" here refer to six/nine sensor axes, not positional DoF. All
// modes below output 3DoF orientation only.
typedef enum RAYNEO_GtFusionMode {
    RAYNEO_GT_FUSION_6D = 0, // gyro + accelerometer
    RAYNEO_GT_FUSION_AUTO_9D   = 1, // +magnetometer with disturbance rejection
    RAYNEO_GT_FUSION_FORCE_9D  = 2  // diagnostic: +magnetometer without rejection
} RAYNEO_GtFusionMode;

// VQF tuning remains internal to the SDK.
typedef struct RAYNEO_GtTrackingConfig {
    uint32_t structSize;
    int32_t  fusionMode;
    uint8_t  requireFactoryCalibration;
    uint8_t  requireTemperatureBias;
    uint8_t  reserved[6];
} RAYNEO_GtTrackingConfig;

typedef struct RAYNEO_GtMagCalibration {
    float hardIron[3]; // subtract before per-axis scale
    float scale[3];    // normally near 1.0
    uint8_t valid;
    uint8_t reserved[3];
} RAYNEO_GtMagCalibration;

// GT orientation channel. Quaternion order is [w,x,y,z]; translation is not estimated.
typedef struct RAYNEO_GtOrientation {
    float    quat[4];
    uint64_t timestampNs; // relative sensor time since the first fused sample
    uint32_t tick;
    uint32_t count;
    float    temperature;
    float    appliedTemperatureBiasRadPerSec[3];
    float    residualBiasRadPerSec[3];
    float    magnetRuntime[3];
    float    magnetFieldStrength; // norm in the raw packet's magnetic units
    int32_t  fusionMode;
    uint8_t  valid;
    uint8_t  usingMagnetometer;
    uint8_t  magneticDisturbance;
    uint8_t  restDetected;
    uint8_t  factoryCalibrationValid;
    uint8_t  temperatureBiasValid;
    uint8_t  magnetometerCalibrationValid;
    uint8_t  fusionReset;
} RAYNEO_GtOrientation;

// Device info (mini) returned from command 0x00 (type 0xC8 ack). Originally only raw[60] was exposed.
// Minor version 1 adds decoded fields after the original prefix for convenience.
typedef struct vyt  {
    uint8_t  raw[60];      // raw payload bytes (unchanged for backward compatibility)
    uint8_t  valid;        // 1 if filled
    uint8_t  reserved[3];  // kept for alignment with original 1.0 layout
    // --- Decoded fields (added in 1.1). If parsing failed they remain zeroed. ---
    uint32_t tick;
    uint8_t  value;
    uint8_t  cpuid[12];
    uint8_t  board_id;
    uint8_t  sensor_on;
    uint8_t  support_fov;
    char     date[13];     // null-terminated
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  glasses_fps;
    uint8_t  luminance;
    uint8_t  volume;
    uint8_t  side_by_side;
    uint8_t  psensor_enable;
    uint8_t  audio_mode;
    uint8_t  dp_status;
    uint8_t  status3;
    uint8_t  psensor_valid;
    uint8_t  lsensor_valid;
    uint8_t  gyro_valid;
    uint8_t  magnet_valid;
    float    reserve1;
    float    reserve2;
    uint8_t  max_luminance;
    uint8_t  max_volume;
    uint8_t  support_panel_color_adjust;
    uint8_t  flag;
} RAYNEO_DeviceInfoMini;

typedef struct RAYNEO_Event {
    RAYNEO_EventType type;
    uint64_t         seq; // monotonically increasing sequence number
    union {
        RAYNEO_ImuSample      imu;
        RAYNEO_DeviceInfoMini info;
        struct { int code; }  error;
        struct { int level; char message[96]; } log;
        struct { int code; char message[96]; } notify; // new in 1.2
    } data;
} RAYNEO_Event;

typedef void (*RAYNEO_EventCallback)(const RAYNEO_Event* evt, void* user);

// Human-readable string for a result code.
RAYNEO_API const char* Rayneo_ResultToString(RAYNEO_Result r);

// Lifecycle
// Enumerate unique supported VID/PID pairs currently connected, without opening
// a USB device or starting the service. outCount receives the total found.
// Pass NULL/0 to query the count. At most capacity entries are written; if
// outCount exceeds capacity, retry with a larger buffer. No devices is OK/0.
// Discovery does not guarantee access permissions or select a device for Start.
RAYNEO_API RAYNEO_Result Rayneo_Discovery(RAYNEO_VidPid* devices, size_t capacity, size_t* outCount);
// Lifecycle operations must not be invoked concurrently from multiple threads.
RAYNEO_API RAYNEO_Result Rayneo_Create(RAYNEO_Context* outCtx);
// Do not call Destroy from an SDK event callback. Call Stop from the callback,
// return from it, then destroy the context from its owning thread.
RAYNEO_API void          Rayneo_Destroy(RAYNEO_Context ctx);

// Configuration (call before Start). If unset, defaults may be used.
RAYNEO_API RAYNEO_Result Rayneo_SetTargetVidPid(RAYNEO_Context ctx, uint16_t vid, uint16_t pid);
// Select a USB interface on libusb platforms. -1 keeps automatic selection.
RAYNEO_API RAYNEO_Result Rayneo_SetTargetInterface(RAYNEO_Context ctx, int interfaceNumber);
// Callback and PollEvent are alternative event-delivery modes. PollEvent
// returns RAYNEO_ERR_BUSY while a callback is configured. Event delivery is
// best-effort under sustained consumer backlog; consumers must tolerate drops.
RAYNEO_API RAYNEO_Result Rayneo_SetEventCallback(RAYNEO_Context ctx, RAYNEO_EventCallback cb, void* user);
RAYNEO_API RAYNEO_Result Rayneo_SetLogLevel(int level); // global/simple; 0=errors .. 3=debug

// Service control
RAYNEO_API RAYNEO_Result Rayneo_Start(RAYNEO_Context ctx, uint32_t serviceFlags);
// May be called from an SDK event callback.
RAYNEO_API RAYNEO_Result Rayneo_Stop(RAYNEO_Context ctx);

// Optional polling (alternative to callback). timeoutMs=0 => non-blocking.
RAYNEO_API RAYNEO_Result Rayneo_PollEvent(RAYNEO_Context ctx, RAYNEO_Event* outEvent, uint32_t timeoutMs);

// Raw protocol I/O (64-byte frames)
RAYNEO_API RAYNEO_Result Rayneo_SendRaw(RAYNEO_Context ctx, const uint8_t frame64[64]);
RAYNEO_API RAYNEO_Result Rayneo_SendCommand(RAYNEO_Context ctx,
                                            uint8_t command, uint8_t value,
                                            const void* payload, size_t payloadLen);
RAYNEO_API RAYNEO_Result Rayneo_RoundTrip(RAYNEO_Context ctx,
                                          uint8_t command, uint8_t value,
                                          const void* payload, size_t payloadLen,
                                          uint32_t timeoutMs,
                                          uint8_t outFrame[64]);

// Convenience commands (thin wrappers around SendCommand)
RAYNEO_API RAYNEO_Result Rayneo_EnableImu(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_DisableImu(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_RequestDeviceInfo(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_DisplaySet3D(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_DisplaySet2D(RAYNEO_Context ctx);

// GT/Gemini calibration helpers. These require a running GT context.
RAYNEO_API RAYNEO_Result Rayneo_QueryGtFactoryCalibration(RAYNEO_Context ctx, uint32_t timeoutMs, RAYNEO_GtFactoryCalibration* out);
RAYNEO_API RAYNEO_Result Rayneo_QueryGtGyroBiasTable(RAYNEO_Context ctx, uint32_t timeoutMs, RAYNEO_GtGyroBiasTable* out);
RAYNEO_API RAYNEO_Result Rayneo_GetGtGyroBiasAtTemperature(const RAYNEO_GtGyroBiasTable* table,
                                                           float temperatureC,
                                                           float outBiasRadPerSec[3]);
RAYNEO_API RAYNEO_Result Rayneo_ApplyGtSensorCalibration(const RAYNEO_GtFactoryCalibration* calibration,
                                                         const RAYNEO_GtGyroBiasTable* biasTable,
                                                         const RAYNEO_ImuSample* input,
                                                         RAYNEO_ImuSample* output);
RAYNEO_API RAYNEO_Result Rayneo_ApplyGtRuntimeAxes(const RAYNEO_ImuSample* input,
                                                   RAYNEO_ImuSample* output);
RAYNEO_API RAYNEO_Result Rayneo_PrepareGtImuSample(const RAYNEO_GtFactoryCalibration* calibration,
                                                   const RAYNEO_GtGyroBiasTable* biasTable,
                                                   const RAYNEO_ImuSample* input,
                                                   RAYNEO_ImuSample* output);

// Optional GT/Gemini orientation service. Initialization queries 0x3C and 0x3E;
// raw IMU events remain unchanged and fused samples use the GT orientation channel.
RAYNEO_API void          Rayneo_GtTrackingConfigInit(RAYNEO_GtTrackingConfig* config);
RAYNEO_API RAYNEO_Result Rayneo_GtInitializeTracking(RAYNEO_Context ctx,
                                                     const RAYNEO_GtTrackingConfig* config,
                                                     uint32_t timeoutMs);
RAYNEO_API RAYNEO_Result Rayneo_GtShutdownTracking(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_GtResetFusion(RAYNEO_Context ctx);
RAYNEO_API RAYNEO_Result Rayneo_GtSetFusionMode(RAYNEO_Context ctx, RAYNEO_GtFusionMode mode);
RAYNEO_API RAYNEO_Result Rayneo_GtSetMagCalibration(RAYNEO_Context ctx, const RAYNEO_GtMagCalibration* calibration);
RAYNEO_API RAYNEO_Result Rayneo_GtGetMagCalibration(RAYNEO_Context ctx, RAYNEO_GtMagCalibration* out);
RAYNEO_API RAYNEO_Result Rayneo_GtPollOrientation(RAYNEO_Context ctx, RAYNEO_GtOrientation* out, uint32_t timeoutMs);
RAYNEO_API RAYNEO_Result Rayneo_GtGetLastOrientation(RAYNEO_Context ctx, RAYNEO_GtOrientation* out);

// Snapshots of last parsed data
RAYNEO_API RAYNEO_Result Rayneo_GetLastImu(RAYNEO_Context ctx, RAYNEO_ImuSample* out);
// Copy the most recent raw 64-byte IMU report.
RAYNEO_API RAYNEO_Result Rayneo_GetLastImuFrame(RAYNEO_Context ctx, uint8_t outFrame[64]);
RAYNEO_API RAYNEO_Result Rayneo_GetDeviceInfo(RAYNEO_Context ctx, RAYNEO_DeviceInfoMini* out);

#ifdef __cplusplus
} // extern "C"
// Layout sanity check to ensure new decoded fields start after 64 bytes (60 raw + 4 status bytes)
static_assert(offsetof(RAYNEO_DeviceInfoMini, tick) == 64, "RAYNEO_DeviceInfoMini unexpected layout (tick offset must be 64)");
#endif
