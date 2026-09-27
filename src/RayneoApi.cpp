#define RAYNEO_BUILD
#include "rayneo_api.h"
#include "vqf.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <deque>
#include <string>
#include <condition_variable>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <algorithm>
#include <cmath>
#include <memory>

#ifdef __APPLE__
#include <IOKit/hid/IOHIDManager.h>
#include <IOKit/hid/IOHIDKeys.h>
#include <CoreFoundation/CoreFoundation.h>
#else
#include <libusb.h>
#endif

#ifndef __APPLE__
static const char *rayneoUsbErr(int code)
{
    return libusb_error_name(code);
}
#endif

struct RayneoContext__
{
    uint16_t vid{0};
    uint16_t pid{0};
    std::atomic<bool> running{false};
    RAYNEO_EventCallback cb{nullptr};
    void *cbUser{nullptr};
    int logLevel{2};
    std::atomic<uint64_t> seq{0};
    // Simple last snapshots
    std::mutex snapshotMtx;
    RAYNEO_ImuSample lastImu{};
    RAYNEO_DeviceInfoMini lastInfo{};
    uint8_t lastImuFrame[64]{};
    bool lastImuFrameValid{false};
    // Event queue shared by transport/event producers and one consumer.
    std::mutex qMtx;
    std::condition_variable qCv;
    std::vector<RAYNEO_Event> queue; // naive FIFO
    size_t maxQueue{128};
    std::thread callbackWorker;
    std::atomic<bool> stopCallback{false};
    // --- transport (non-Apple libusb) ---
#ifndef __APPLE__
    libusb_context *usbCtx{nullptr};
    libusb_device_handle *handle{nullptr};
    int targetInterfaceNumber{-1};
    int interfaceNumber{-1};
    int altSetting{-1};
    uint8_t epIn{0};
    uint8_t epOut{0};
    uint16_t epInMaxPacket{64};
    bool hasInterrupt{false};
    // async interrupt transfer state
    libusb_transfer *inTransfer{nullptr};
    std::vector<uint8_t> inBuffer;
    std::atomic<bool> transferActive{false};
    std::atomic<bool> transferDone{false};
    std::atomic<bool> transferResubmit{false};
#else
    IOHIDManagerRef hidManager{nullptr};
    IOHIDDeviceRef hidDevice{nullptr};
    CFRunLoopRef hidRunLoop{nullptr};
    uint8_t *hidInputBuffer{nullptr};
    size_t hidReportSize{64};
    std::mutex hidMtx;
    std::mutex macReadyMtx;
    std::condition_variable macReadyCv;
    bool macReady{false};
    RAYNEO_Result macReadyStatus{RAYNEO_OK};
    std::atomic<bool> macDeviceOnline{false};
#endif
    // worker
    std::thread worker;
    std::atomic<bool> stopWorker{false};
    std::atomic<bool> stopIssued{false};
    std::atomic<bool> detachEmitted{false};
    // Outbound command serialization and RoundTrip sync
    std::mutex commandMtx;
    std::mutex rtMtx;
    std::condition_variable rtCv;
    uint8_t rtFrame[64]{};
    uint8_t rtCommand{0};
    bool rtPending{false};
    bool rtReady{false};

    // GT 0x3E returns multiple frames; collect them without changing RoundTrip semantics.
    std::mutex gtBiasMtx;
    std::condition_variable gtBiasCv;
    RAYNEO_GtGyroBiasTable gtBiasCollectTable{};
    bool gtBiasCollecting{false};

    // GT orientations use a separate queue to preserve the RAYNEO_Event ABI.
    std::mutex gtOrientationMtx;
    std::condition_variable gtOrientationCv;
    std::deque<RAYNEO_GtOrientation> gtOrientationQueue;
    size_t maxGtOrientationQueue{128};
    RAYNEO_GtOrientation lastGtOrientation{};

    std::mutex trackingMtx;
    RAYNEO_GtTrackingConfig trackingConfig{};
    RAYNEO_GtFactoryCalibration trackingCalibration{};
    RAYNEO_GtGyroBiasTable trackingBiasTable{};
    RAYNEO_GtMagCalibration trackingMagCalibration{{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, 0, {0, 0, 0}};
    std::unique_ptr<VQF> vqf;
    std::atomic<bool> trackingActive{false};
    bool trackingInitialized{false};
    bool trackingFactoryFromDevice{false};
    bool trackingBiasFromDevice{false};
    bool trackingHaveTick{false};
    bool trackingHaveMagTick{false};
    bool trackingEverFedMag{false};
    uint32_t trackingLastTick{0};
    uint32_t trackingLastMagTick{0};
    uint64_t trackingTimestampNs{0};

    std::atomic<uint8_t> lastType{0};
};

// Internal protocol message / command identifiers (mirrors RayNeoSimpleClient)
enum : uint8_t
{
    RAYNEO_PROTO_CMD_DEVICE_INFO = 0x00, // Outgoing command to request device info
    RAYNEO_PROTO_ACK_COMMAND = 0xC8,     // Generic command ack / may carry device info
    RAYNEO_PROTO_ACK_IMU_DATA = 0x65,    // IMU sample frame
    RAYNEO_PROTO_ACK_TRACE = 0xCA,       // Trace report
    RAYNEO_PROTO_NOTIFY_SLEEP = 0x41,    // Glasses sleep notify
    RAYNEO_PROTO_NOTIFY_WAKE = 0x42,     // Glasses wake notify
    RAYNEO_PROTO_NOTIFY_FAIL = 0x02
};

enum FXRUsbCommand {
    kCmdDeviceInfo = 0,
    kCmdImuOn = 1,
    kCmdImuOff = 2,
    kCmdSensorGyroCorrection = 3,
    kCmdDisplay3dMode = 6,
    kCmdDisplay2dMode = 7,
    kCmdPanelPresetSet = 9,
    kCmdSetPanelRotation = 0xA,
    kCmdPanelLumaSave = 0xD,
    kCmdPanelPowerOn = 0xE,
    kCmdPanelPowerOff = 0xF,
    kCmdPanelPowerSwitch = 0x10,
    kCmdPanelSwap = 0x12,
    kCmdAccelerateRate = 0x19,
    kCmdGyroRate = 0x1A,
    kCmdMagnetRate = 0x1B,
    kCmdParamReset = 0x1D,
    kCmdParamSave = 0x1F,
    kCmdPanelSet60fps = 0x20,
    kCmdPanelSet120fps = 0x21,
    kAckWheelKeyDoublePressed = 0x22,
    kCmdPanelGetFov = 0x23,
    KCmdAudioEnablePersistence = 0x25, //静音模式, 仅白羊1.0支持
    kCmdSideBySideChange = 0x30,
    kCmdTraceReport = 0x33, //设备ID
    kCmdAudioEnable = 0x34,
    kCmdPsensorEnable = 0x38,
    kCmdImuCalibration = 0x3C,
    kCmdGetGyroBias = 0x3E,
    kCmdSetGyroBias = 0x3F,
    kAckGlassesSleepNotify = 0x41,
    kAckGlassesWakeupNotify = 0x42,
    kCmdAudioMode = 0x49,
    kCmdVolumeSet = 0x50,
    kCmdVolumeUp = 0x51,
    kCmdVolumeDown = 0x52,
    kAckWheelKeyPressed = 0x54,
    kCmdLuminanceUp = 0x55,
    kCmdLuminanceDown = 0x56,
    kAckWheelKeyLongPressed = 0x57,
    kCmdWheelKeySideBySideDisable = 0x58,
    kAckImuData = 0x65,
    kCmdReboot2Bootloader = 0x66,
    kCmdCaluLum = 0x67,
    kCmdGetLT7211Version = 0x6A,
    kCmdPanelColorAdjust = 0x73,
    kAckCommand = 0xC8,
    kAckUsbCommandLog = 0xC9,
    kAckTraceReport = 0xCA,
    kCmdUsbCommandPARGESDump = 0xCB,
    kCmdTimeSynchronize = 0xE1, //设备间时钟同步
};

static constexpr uint8_t RAYNEO_GT_NOTIFY_SPATIAL_MODE = 0x08;
static constexpr float RAYNEO_GT_VQF_IMU_HZ = 500.0f;
static constexpr float RAYNEO_GT_VQF_MAG_HZ = 100.0f;
static constexpr uint32_t RAYNEO_GT_FUSION_MAX_GAP_MS = 250u;

static void enqueueEvent(RayneoContext__ *ctx, const RAYNEO_Event &evt);
static void processGtOrientation(RayneoContext__ *ctx, const RAYNEO_ImuSample &sample);
static bool rebuildGtFusionLocked(RayneoContext__ *ctx);
static bool decodeGtGyroBiasFrame(const uint8_t frame[64], RAYNEO_GtGyroBiasTable &table);

static void emitDetachedOnce(RayneoContext__ *ctx)
{
    bool expected = false;
    if (!ctx || !ctx->detachEmitted.compare_exchange_strong(expected, true))
        return;

    RAYNEO_Event evt{};
    evt.type = RAYNEO_EVENT_DEVICE_DETACHED;
    evt.seq = ++ctx->seq;
    enqueueEvent(ctx, evt);
}

static void processInboundFrame(RayneoContext__ *ctx, const uint8_t *buf, size_t len)
{
    if (!ctx || !buf || len < 2)
        return;
    if (len < 64 || buf[0] != 0x99)
    {
        return;
    }

    uint8_t type = buf[1];
    ctx->lastType.store(type);

    if (type == RAYNEO_PROTO_ACK_COMMAND)
    {
        bool matched = false;
        {
            std::lock_guard<std::mutex> lk(ctx->rtMtx);
            if (ctx->rtPending && buf[8] == ctx->rtCommand)
            {
                std::memcpy(ctx->rtFrame, buf, 64);
                ctx->rtReady = true;
                ctx->rtPending = false;
                matched = true;
            }
        }
        if (matched)
            ctx->rtCv.notify_all();
    }

    auto rdF = [&](int off)
    {
        float f = 0.f;
        if (off + 4 <= static_cast<int>(len))
            std::memcpy(&f, buf + off, 4);
        return f;
    };
    auto rdU32 = [&](int off)
    {
        if (off + 4 > static_cast<int>(len))
            return uint32_t{0};
        return (uint32_t)buf[off] | ((uint32_t)buf[off + 1] << 8) | ((uint32_t)buf[off + 2] << 16) | ((uint32_t)buf[off + 3] << 24);
    };

    if (type == RAYNEO_PROTO_ACK_IMU_DATA)
    {
        RAYNEO_ImuSample sample{};
        sample.valid = 1;
        for (int i = 0; i < 3; i++)
            sample.acc[i] = rdF(4 + i * 4);
        for (int i = 0; i < 3; i++)
            sample.gyroDps[i] = rdF(16 + i * 4);
        sample.temperature = rdF(28);
        sample.magnet[0] = rdF(32);
        sample.magnet[1] = rdF(36);
        sample.tick = rdU32(40);
        sample.psensor = rdF(44);
        sample.lsensor = rdF(48);
        sample.magnet[2] = rdF(52);
        if (ctx->vid == RAYNEO_GT_VID && ctx->pid == RAYNEO_GT_PID)
        {
            // GT/Gemini 0x65 trailer: count[56..59], checksum[62], flag[63].
            sample.count = rdU32(56);
            sample.checksum = buf[62];
            sample.flag = buf[63];
        }
        else
        {
            if (len > 56)
                sample.flag = buf[56];
            if (len > 57)
                sample.checksum = buf[57];
        }
        for (int i = 0; i < 3; i++)
            sample.gyroRad[i] = sample.gyroDps[i] * 0.0174532925f;
        {
            std::lock_guard<std::mutex> lk(ctx->snapshotMtx);
            ctx->lastImu = sample;
            std::memcpy(ctx->lastImuFrame, buf, 64);
            ctx->lastImuFrameValid = true;
        }
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_IMU_SAMPLE;
        evt.seq = ++ctx->seq;
        evt.data.imu = sample;
        enqueueEvent(ctx, evt);

        processGtOrientation(ctx, sample);
    }
    else if (type == RAYNEO_PROTO_ACK_COMMAND)
    {
        if (ctx->vid == RAYNEO_GT_VID && ctx->pid == RAYNEO_GT_PID &&
            buf[8] == kCmdGetGyroBias)
        {
            bool notify = false;
            {
                std::lock_guard<std::mutex> lk(ctx->gtBiasMtx);
                if (ctx->gtBiasCollecting && decodeGtGyroBiasFrame(buf, ctx->gtBiasCollectTable))
                    notify = ctx->gtBiasCollectTable.complete != 0;
            }
            if (notify)
                ctx->gtBiasCv.notify_all();
            return;
        }
        if (ctx->vid == RAYNEO_GT_VID && ctx->pid == RAYNEO_GT_PID &&
            (buf[8] == kCmdImuCalibration || buf[8] == kCmdSetGyroBias))
        {
            // GT calibration replies are not device-info or button notifications.
            return;
        }

        RAYNEO_DeviceInfoMini info{};
        info.valid = 1;
        if (len >= 64)
            std::memcpy(info.raw, buf + 4, 60);
        const uint8_t *p = info.raw;
        size_t off = 0;
        auto need = [&](size_t b)
        { return (off + b) <= 60; };
        auto rdU32p = [&]()
        {
            uint32_t v = 0;
            if (need(4))
                v = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) | ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
            off += 4;
            return v;
        };
        auto rdU16p = [&]()
        {
            uint16_t v = 0;
            if (need(2))
                v = (uint16_t)p[off] | ((uint16_t)p[off + 1] << 8);
            off += 2;
            return v;
        };
        auto rdBp = [&]()
        {
            uint8_t v = 0;
            if (need(1))
                v = p[off];
            off += 1;
            return v;
        };
        auto rdFp = [&]()
        {
            float f = 0;
            if (need(4))
                std::memcpy(&f, p + off, 4);
            off += 4;
            return f;
        };
        info.tick = rdU32p();
        auto value = rdBp();
        if (value == kAckGlassesSleepNotify) {
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_SLEEP;
            enqueueEvent(ctx, evt);
            return;
        }else if (value == kAckGlassesWakeupNotify) {
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_WAKE;
            enqueueEvent(ctx, evt);
            return;
        } else if (value == 0x49) { // todo: remove magic number
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_BUTTON;
            enqueueEvent(ctx, evt);
            return;
        } else if (value == 0x59){ // todo: remove magic number
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_BUTTON; // Probably is also a button notify, but in 3d mode
            enqueueEvent(ctx, evt);
            return;
        } else if (value == kCmdVolumeUp){
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_BUTTON_VOLUME_DOWN; // Minus button notify
            enqueueEvent(ctx, evt);
            return;
        } else if (value == kCmdVolumeDown){
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_BUTTON_VOLUME_UP; // Plus button notify
            enqueueEvent(ctx, evt);
            return;
        } else if (value == 0x09){ // todo: remove magic number
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_BUTTON_BRIGHTNESS;
            enqueueEvent(ctx, evt);
            return;
        } else if (value == kCmdImuOff){
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_IMU_OFF;
            enqueueEvent(ctx, evt);
            return;

        } else if (value == kCmdImuOn){
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_IMU_ON;
            enqueueEvent(ctx, evt);
            return;

        } else if (value == RAYNEO_GT_NOTIFY_SPATIAL_MODE &&
                   ctx->vid == RAYNEO_GT_VID &&
                   ctx->pid == RAYNEO_GT_PID) {
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_NOTIFY;
            evt.seq = ++ctx->seq;
            evt.data.notify.code = RAYNEO_NOTIFY_BUTTON_SPATIAL_MODE;
            enqueueEvent(ctx, evt);
            return;

        } else if (value == kCmdDisplay3dMode ||
                   value == kCmdDisplay2dMode ||
                   value == kCmdVolumeSet) {
            return;

        } else if (value != 0) {
            printf("[SimpleClient] Unknown Notify? 0x%02X\n", value);
        }

        if (need(12))
        {
            std::memcpy(info.cpuid, p + off, 12);
            off += 12;
        }
        info.board_id = rdBp();
        info.sensor_on = rdBp();
        info.support_fov = rdBp();
        if (need(12))
        {
            std::memcpy(info.date, p + off, 12);
            info.date[12] = '\0';
            off += 12;
        }
        info.year = rdU16p();
        info.month = rdBp();
        info.day = rdBp();
        info.glasses_fps = rdBp();
        info.luminance = rdBp();
        info.volume = rdBp();
        info.side_by_side = rdBp();
        info.psensor_enable = rdBp();
        info.audio_mode = rdBp();
        info.dp_status = rdBp();
        info.status3 = rdBp();
        info.psensor_valid = rdBp();
        info.lsensor_valid = rdBp();
        info.gyro_valid = rdBp();
        info.magnet_valid = rdBp();
        info.reserve1 = rdFp();
        info.reserve2 = rdFp();
        info.max_luminance = rdBp();
        info.max_volume = rdBp();
        info.support_panel_color_adjust = rdBp();
        info.flag = rdBp();
        bool emit = false;
        {
            std::lock_guard<std::mutex> lk(ctx->snapshotMtx);
            emit = (!ctx->lastInfo.valid) || (ctx->lastInfo.tick != info.tick);
            ctx->lastInfo = info;
        }
        if (emit)
        {
            RAYNEO_Event evt{};
            evt.type = RAYNEO_EVENT_DEVICE_INFO;
            evt.seq = ++ctx->seq;
            evt.data.info = info;
            enqueueEvent(ctx, evt);
        }
    }
    else if (type == RAYNEO_PROTO_NOTIFY_SLEEP || type == RAYNEO_PROTO_NOTIFY_WAKE)
    {
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_LOG;
        evt.seq = ++ctx->seq;
        evt.data.log.level = 1;
        std::snprintf(evt.data.log.message, sizeof(evt.data.log.message), "%s (type=0x%02X)", (type == RAYNEO_PROTO_NOTIFY_SLEEP) ? "Sleep notify" : "Wakeup notify", type);
        enqueueEvent(ctx, evt);
    }
}

#ifdef __APPLE__
static void macSignalReady(RayneoContext__ *ctx, RAYNEO_Result status)
{
    if (!ctx)
        return;
    {
        std::lock_guard<std::mutex> lk(ctx->macReadyMtx);
        ctx->macReadyStatus = status;
        ctx->macReady = true;
    }
    ctx->macReadyCv.notify_all();
}

static int macGetDeviceInt(IOHIDDeviceRef dev, CFStringRef key, int fallback = 0)
{
    if (!dev)
        return fallback;
    CFTypeRef prop = IOHIDDeviceGetProperty(dev, key);
    if (prop && CFGetTypeID(prop) == CFNumberGetTypeID())
    {
        int value = fallback;
        CFNumberGetValue((CFNumberRef)prop, kCFNumberIntType, &value);
        return value;
    }
    return fallback;
}

static void macDeviceMatchingCallback(void *context, IOReturn result, void *, IOHIDDeviceRef device)
{
    auto *ctx = static_cast<RayneoContext__ *>(context);
    if (!ctx || result != kIOReturnSuccess || !device)
        return;
    int vid = macGetDeviceInt(device, CFSTR(kIOHIDVendorIDKey));
    int pid = macGetDeviceInt(device, CFSTR(kIOHIDProductIDKey));
    if (vid == ctx->vid && pid == ctx->pid)
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (!ctx->hidDevice)
        {
            ctx->hidDevice = device;
            CFRetain(device);
        }
    }
}

static void macDeviceRemovalCallback(void *context, IOReturn result, void *, IOHIDDeviceRef device)
{
    auto *ctx = static_cast<RayneoContext__ *>(context);
    if (!ctx || result != kIOReturnSuccess || !device)
        return;
    bool emitDetach = false;
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (ctx->hidDevice == device && ctx->hidDevice)
        {
            IOHIDDeviceClose(ctx->hidDevice, kIOHIDOptionsTypeNone);
            CFRelease(ctx->hidDevice);
            ctx->hidDevice = nullptr;
            delete[] ctx->hidInputBuffer;
            ctx->hidInputBuffer = nullptr;
            ctx->macDeviceOnline.store(false);
            emitDetach = true;
        }
    }
    if (emitDetach && !ctx->stopWorker.load())
        emitDetachedOnce(ctx);
}

static void macInputReportCallback(void *context, IOReturn result, void *, IOHIDReportType, uint32_t, uint8_t *report, CFIndex reportLength)
{
    auto *ctx = static_cast<RayneoContext__ *>(context);
    if (!ctx || result != kIOReturnSuccess || !report)
        return;
    processInboundFrame(ctx, report, static_cast<size_t>(reportLength));
}

static bool macEnsureDeviceSelected(RayneoContext__ *ctx)
{
    if (!ctx || !ctx->hidManager)
        return false;

    CFSetRef set = IOHIDManagerCopyDevices(ctx->hidManager);
    if (!set)
        return false;
    CFIndex count = CFSetGetCount(set);
    // CFSetGetValues fills an array of const void* values. We store them as such
    // then cast to IOHIDDeviceRef (opaque pointer) for inspection. Removing const
    // is safe: we do not mutate device objects here, only read properties.
    std::vector<const void *> raw(static_cast<size_t>(count));
    if (count > 0)
        CFSetGetValues(set, raw.data());
    for (const void *entry : raw)
    {
        // Convert const void* from CFSet to IOHIDDeviceRef without triggering
        // qualifier warnings: IOHIDDeviceRef is an opaque pointer typedef.
        IOHIDDeviceRef dev = reinterpret_cast<IOHIDDeviceRef>(const_cast<void *>(entry));
        if (!dev)
            continue;
        int vid = macGetDeviceInt(dev, CFSTR(kIOHIDVendorIDKey));
        int pid = macGetDeviceInt(dev, CFSTR(kIOHIDProductIDKey));
        if (vid == ctx->vid && pid == ctx->pid)
        {
            std::lock_guard<std::mutex> lk(ctx->hidMtx);
            if (!ctx->hidDevice)
            {
                ctx->hidDevice = dev;
                CFRetain(dev);
            }
            CFRelease(set);
            return true;
        }
    }
    CFRelease(set);
    return false;
}

static void macReleaseDevice(RayneoContext__ *ctx)
{
    if (!ctx)
        return;
    std::lock_guard<std::mutex> lk(ctx->hidMtx);
    if (ctx->hidDevice)
    {
        IOHIDDeviceClose(ctx->hidDevice, kIOHIDOptionsTypeNone);
        CFRelease(ctx->hidDevice);
        ctx->hidDevice = nullptr;
    }
    delete[] ctx->hidInputBuffer;
    ctx->hidInputBuffer = nullptr;
    ctx->macDeviceOnline.store(false);
}

static void macCleanupManager(RayneoContext__ *ctx)
{
    if (!ctx)
        return;
    if (ctx->hidManager)
    {
        if (ctx->hidRunLoop)
            IOHIDManagerUnscheduleFromRunLoop(ctx->hidManager, ctx->hidRunLoop, kCFRunLoopDefaultMode);
        IOHIDManagerClose(ctx->hidManager, kIOHIDOptionsTypeNone);
        CFRelease(ctx->hidManager);
        ctx->hidManager = nullptr;
    }
    ctx->hidRunLoop = nullptr;
}

static void macWorkerThread(RayneoContext__ *ctx)
{
    if (!ctx)
        return;
    ctx->hidRunLoop = CFRunLoopGetCurrent();
    ctx->macDeviceOnline.store(false);
    bool signaled = false;
    auto signalOnce = [&](RAYNEO_Result status)
    {
        if (!signaled)
        {
            macSignalReady(ctx, status);
            signaled = true;
        }
    };

    ctx->hidManager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!ctx->hidManager)
    {
        signalOnce(RAYNEO_ERR_GENERAL);
        return;
    }

    CFMutableDictionaryRef dict = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    if (dict)
    {
        CFNumberRef vidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &ctx->vid);
        CFNumberRef pidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &ctx->pid);
        if (vidNum && pidNum)
        {
            CFDictionarySetValue(dict, CFSTR(kIOHIDVendorIDKey), vidNum);
            CFDictionarySetValue(dict, CFSTR(kIOHIDProductIDKey), pidNum);
        }
        if (vidNum)
            CFRelease(vidNum);
        if (pidNum)
            CFRelease(pidNum);
    }
    if (dict)
        IOHIDManagerSetDeviceMatching(ctx->hidManager, dict);
    if (dict)
        CFRelease(dict);

    IOHIDManagerRegisterDeviceMatchingCallback(ctx->hidManager, macDeviceMatchingCallback, ctx);
    IOHIDManagerRegisterDeviceRemovalCallback(ctx->hidManager, macDeviceRemovalCallback, ctx);

    IOReturn rc = IOHIDManagerOpen(ctx->hidManager, kIOHIDOptionsTypeNone);
    if (rc != kIOReturnSuccess)
    {
        signalOnce(RAYNEO_ERR_GENERAL);
        macCleanupManager(ctx);
        return;
    }

    IOHIDManagerScheduleWithRunLoop(ctx->hidManager, ctx->hidRunLoop, kCFRunLoopDefaultMode);

    auto waitForDevice = [&](double seconds, bool allowUnfiltered)
    {
        auto start = std::chrono::steady_clock::now();
        while (!ctx->stopWorker.load() && !ctx->hidDevice && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds))
        {
            CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
            if (!ctx->hidDevice)
                macEnsureDeviceSelected(ctx);
        }
        if (!ctx->hidDevice && allowUnfiltered)
        {
            IOHIDManagerSetDeviceMatching(ctx->hidManager, nullptr);
            start = std::chrono::steady_clock::now();
            while (!ctx->stopWorker.load() && !ctx->hidDevice && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds))
            {
                CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
                if (!ctx->hidDevice)
                    macEnsureDeviceSelected(ctx);
            }
        }
        return ctx->hidDevice != nullptr;
    };

    if (!waitForDevice(2.0, true))
    {
        signalOnce(RAYNEO_ERR_NO_DEVICE);
        macCleanupManager(ctx);
        return;
    }

    IOReturn openRc = kIOReturnError;
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (ctx->hidDevice)
        {
            openRc = IOHIDDeviceOpen(ctx->hidDevice, kIOHIDOptionsTypeNone);
            if (openRc != kIOReturnSuccess)
                openRc = IOHIDDeviceOpen(ctx->hidDevice, kIOHIDOptionsTypeSeizeDevice);
        }
    }
    if (openRc != kIOReturnSuccess)
    {
        signalOnce(RAYNEO_ERR_IO);
        macReleaseDevice(ctx);
        macCleanupManager(ctx);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (ctx->hidDevice)
        {
            CFTypeRef prop = IOHIDDeviceGetProperty(ctx->hidDevice, CFSTR(kIOHIDMaxInputReportSizeKey));
            if (prop && CFGetTypeID(prop) == CFNumberGetTypeID())
            {
                int sz = 0;
                CFNumberGetValue((CFNumberRef)prop, kCFNumberIntType, &sz);
                if (sz > 0)
                    ctx->hidReportSize = static_cast<size_t>(sz);
            }
            delete[] ctx->hidInputBuffer;
            ctx->hidInputBuffer = new uint8_t[ctx->hidReportSize];
            IOHIDDeviceRegisterInputReportCallback(ctx->hidDevice, ctx->hidInputBuffer, ctx->hidReportSize, macInputReportCallback, ctx);
        }
    }

    ctx->macDeviceOnline.store(true);
    signalOnce(RAYNEO_OK);

    while (!ctx->stopWorker.load())
    {
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
    }

    macReleaseDevice(ctx);
    macCleanupManager(ctx);
}
#endif

static void enqueueEvent(RayneoContext__ *ctx, const RAYNEO_Event &evt)
{
    if (!ctx)
        return;
    std::unique_lock<std::mutex> lk(ctx->qMtx);
    if (ctx->queue.size() >= ctx->maxQueue)
    {
        // drop oldest
        ctx->queue.erase(ctx->queue.begin());
    }
    ctx->queue.push_back(evt);
    lk.unlock();
    ctx->qCv.notify_one();
}

const char *Rayneo_ResultToString(RAYNEO_Result r)
{
    switch (r)
    {
    case RAYNEO_OK:
        return "OK";
    case RAYNEO_ERR_GENERAL:
        return "GENERAL";
    case RAYNEO_ERR_NO_DEVICE:
        return "NO_DEVICE";
    case RAYNEO_ERR_TIMEOUT:
        return "TIMEOUT";
    case RAYNEO_ERR_IO:
        return "IO";
    case RAYNEO_ERR_BUSY:
        return "BUSY";
    case RAYNEO_ERR_UNSUPPORTED:
        return "UNSUPPORTED";
    case RAYNEO_ERR_INVALID_ARG:
        return "INVALID_ARG";
    case RAYNEO_ERR_QUEUE_FULL:
        return "QUEUE_FULL";
    default:
        return "UNKNOWN";
    }
}

unsigned int Rayneo_GetApiVersion(void)
{
    return RAYNEO_API_VERSION;
}

RAYNEO_Result Rayneo_Discovery(RAYNEO_VidPid *devices, size_t capacity, size_t *outCount)
{
    if (!outCount)
        return RAYNEO_ERR_INVALID_ARG;
    *outCount = 0;
    if (!devices && capacity != 0)
        return RAYNEO_ERR_INVALID_ARG;

    constexpr auto supported = Rayneo_GetSupportedDevices();
    bool found[RAYNEO_SUPPORTED_DEVICE_COUNT]{};
    auto record = [&](int vid, int pid) {
        for (size_t i = 0; i < supported.size(); ++i)
            if (supported[i].vid == vid && supported[i].pid == pid)
                found[i] = true;
    };
#ifdef __APPLE__
    IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!manager)
        return RAYNEO_ERR_GENERAL;
    IOHIDManagerSetDeviceMatching(manager, nullptr);
    CFSetRef deviceSet = IOHIDManagerCopyDevices(manager);
    if (deviceSet)
    {
        struct DiscoveryState {
            decltype(record) *recordDevice;
        } state{&record};
        CFSetApplyFunction(deviceSet, [](const void *value, void *context) {
            auto device = static_cast<IOHIDDeviceRef>(const_cast<void *>(value));
            auto *state = static_cast<DiscoveryState *>(context);
            (*state->recordDevice)(macGetDeviceInt(device, CFSTR(kIOHIDVendorIDKey)),
                                   macGetDeviceInt(device, CFSTR(kIOHIDProductIDKey)));
        }, &state);
        CFRelease(deviceSet);
    }
    CFRelease(manager);
#else
    libusb_context *usb = nullptr;
    if (libusb_init(&usb) != 0)
        return RAYNEO_ERR_GENERAL;
    libusb_device **list = nullptr;
    const ssize_t count = libusb_get_device_list(usb, &list);
    if (count < 0)
    {
        libusb_exit(usb);
        return RAYNEO_ERR_IO;
    }
    for (ssize_t i = 0; i < count; ++i)
    {
        libusb_device_descriptor descriptor{};
        if (libusb_get_device_descriptor(list[i], &descriptor) == 0)
            record(descriptor.idVendor, descriptor.idProduct);
    }
    libusb_free_device_list(list, 1);
    libusb_exit(usb);
#endif
    for (size_t i = 0; i < supported.size(); ++i)
    {
        if (!found[i])
            continue;
        // Multiple models may intentionally share one USB identity.
        bool duplicate = false;
        for (size_t j = 0; j < i; ++j)
            if (found[j] && supported[j].vid == supported[i].vid && supported[j].pid == supported[i].pid)
                duplicate = true;
        if (duplicate)
            continue;
        if (*outCount < capacity)
            devices[*outCount] = supported[i];
        ++*outCount;
    }
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_Create(RAYNEO_Context *outCtx)
{
    if (!outCtx)
        return RAYNEO_ERR_INVALID_ARG;
    auto *c = new (std::nothrow) RayneoContext__();
    if (!c)
        return RAYNEO_ERR_GENERAL;
    *outCtx = c;
    return RAYNEO_OK;
}

void Rayneo_Destroy(RAYNEO_Context ctx)
{
    if (!ctx)
        return;

    // callbackWorker owns the callback's stack. It cannot destroy the context
    // that owns that std::thread before the callback returns. Stop is safe
    // here; the owning thread performs the actual destruction afterwards.
    if (ctx->callbackWorker.joinable() &&
        std::this_thread::get_id() == ctx->callbackWorker.get_id())
    {
        (void)Rayneo_Stop(ctx);
        return;
    }

    if (Rayneo_Stop(ctx) == RAYNEO_ERR_BUSY)
        return;
    delete ctx;
}

RAYNEO_Result Rayneo_SetTargetVidPid(RAYNEO_Context ctx, uint16_t vid, uint16_t pid)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->running.load())
        return RAYNEO_ERR_BUSY;
    ctx->vid = vid;
    ctx->pid = pid;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_SetTargetInterface(RAYNEO_Context ctx, int interfaceNumber)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->running.load())
        return RAYNEO_ERR_BUSY;
    if (interfaceNumber < -1 || interfaceNumber > 255)
        return RAYNEO_ERR_INVALID_ARG;
#ifdef __APPLE__
    return (interfaceNumber == -1) ? RAYNEO_OK : RAYNEO_ERR_UNSUPPORTED;
#else
    ctx->targetInterfaceNumber = interfaceNumber;
    return RAYNEO_OK;
#endif
}

RAYNEO_Result Rayneo_SetEventCallback(RAYNEO_Context ctx, RAYNEO_EventCallback cb, void *user)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->running.load())
        return RAYNEO_ERR_BUSY;

    // Stop called from the callback cannot join its own dispatcher. Reap it
    // before replacing callback state for the next session.
    if (ctx->callbackWorker.joinable())
    {
        if (std::this_thread::get_id() == ctx->callbackWorker.get_id())
            return RAYNEO_ERR_BUSY;
        ctx->stopCallback.store(true);
        ctx->qCv.notify_all();
        ctx->callbackWorker.join();
    }

    ctx->cb = cb;
    ctx->cbUser = user;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_SetLogLevel(int level)
{
    (void)level; // global not stored yet
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_Start(RAYNEO_Context ctx, uint32_t /*serviceFlags*/)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->running.load())
        return RAYNEO_ERR_BUSY;

    // Stop called from the callback cannot join its own dispatcher. Reap it
    // before starting the next session.
    if (ctx->callbackWorker.joinable())
    {
        if (std::this_thread::get_id() == ctx->callbackWorker.get_id())
            return RAYNEO_ERR_BUSY;
        ctx->stopCallback.store(true);
        ctx->qCv.notify_all();
        ctx->callbackWorker.join();
    }

    bool expected = false;
    if (!ctx->running.compare_exchange_strong(expected, true))
        return RAYNEO_ERR_BUSY;
    ctx->stopIssued.store(false);
    ctx->stopWorker.store(false);
    ctx->stopCallback.store(false);
    ctx->detachEmitted.store(false);
    {
        std::lock_guard<std::mutex> lk(ctx->snapshotMtx);
        ctx->lastImu = {};
        ctx->lastInfo = {};
        std::memset(ctx->lastImuFrame, 0, sizeof(ctx->lastImuFrame));
        ctx->lastImuFrameValid = false;
    }
    {
        std::lock_guard<std::mutex> lk(ctx->rtMtx);
        ctx->rtPending = false;
        ctx->rtReady = false;
        ctx->rtCommand = 0;
        std::memset(ctx->rtFrame, 0, sizeof(ctx->rtFrame));
    }
    {
        std::lock_guard<std::mutex> lk(ctx->qMtx);
        ctx->queue.clear();
    }
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        ctx->gtOrientationQueue.clear();
        ctx->lastGtOrientation = {};
    }
    {
        std::lock_guard<std::mutex> lk(ctx->gtBiasMtx);
        ctx->gtBiasCollecting = false;
        ctx->gtBiasCollectTable = {};
    }
    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        ctx->trackingInitialized = false;
        ctx->trackingActive.store(false);
        ctx->vqf.reset();
        ctx->trackingHaveTick = false;
        ctx->trackingHaveMagTick = false;
        ctx->trackingEverFedMag = false;
    }

    // Reserve the first sequence number before transport threads can publish
    // data. The attached event is inserted at the front after initialization
    // succeeds, preserving DEVICE_ATTACHED -> data ordering.
    const uint64_t attachedSeq = ++ctx->seq;

    // Initialize transport
#ifndef __APPLE__
    if (libusb_init(&ctx->usbCtx) != 0)
    {
        ctx->running.store(false);
        return RAYNEO_ERR_GENERAL;
    }
    libusb_device **list = nullptr;
    ssize_t n = libusb_get_device_list(ctx->usbCtx, &list);
    if (n < 0)
    {
        if (list)
            libusb_free_device_list(list, 1);
        libusb_exit(ctx->usbCtx);
        ctx->usbCtx = nullptr;
        ctx->running.store(false);
        return RAYNEO_ERR_IO;
    }
    libusb_device *target = nullptr;
    for (ssize_t i = 0; i < n; i++)
    {
        libusb_device *dev = list[i];
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(dev, &desc) == 0)
        {
            if (desc.idVendor == ctx->vid && desc.idProduct == ctx->pid)
            {
                target = dev;
                break;
            }
        }
    }
    if (!target)
    {
        // target not found
        libusb_free_device_list(list, 1);
        libusb_exit(ctx->usbCtx);
        ctx->usbCtx = nullptr;
        ctx->running.store(false);
        return RAYNEO_ERR_NO_DEVICE;
    }
    if (libusb_open(target, &ctx->handle) != 0 || !ctx->handle)
    {
        libusb_free_device_list(list, 1);
        if (ctx->handle)
        {
            libusb_close(ctx->handle);
            ctx->handle = nullptr;
        }
        libusb_exit(ctx->usbCtx);
        ctx->usbCtx = nullptr;
        ctx->running.store(false);
        return RAYNEO_ERR_IO;
    }
    libusb_free_device_list(list, 1);
    ctx->interfaceNumber = -1;
    ctx->altSetting = -1;
    ctx->epIn = 0;
    ctx->epOut = 0;
    ctx->epInMaxPacket = 64;
    ctx->hasInterrupt = false;

    const bool exactTarget = ctx->targetInterfaceNumber >= 0;
    libusb_config_descriptor *cfg = nullptr; // libusb_get_active_config_descriptor expects non-const**
    if (libusb_get_active_config_descriptor(libusb_get_device(ctx->handle), &cfg) == 0 && cfg)
    {
        for (uint8_t i = 0; i < cfg->bNumInterfaces && ctx->interfaceNumber < 0; i++)
        {
            const libusb_interface &iface = cfg->interface[i];
            for (int a = 0; a < iface.num_altsetting && ctx->interfaceNumber < 0; a++)
            {
                const libusb_interface_descriptor &id = iface.altsetting[a];
                if (exactTarget && id.bInterfaceNumber != ctx->targetInterfaceNumber)
                    continue;
                const bool automaticGtTarget = !exactTarget &&
                    ctx->vid == RAYNEO_GT_VID && ctx->pid == RAYNEO_GT_PID;
                if (automaticGtTarget && id.bInterfaceNumber != RAYNEO_GT_HID_INTERFACE)
                    continue;

                uint8_t epIn = 0;
                uint8_t epOut = 0;
                uint16_t epInMaxPacket = 64;
                for (uint8_t e = 0; e < id.bNumEndpoints; e++)
                {
                    const libusb_endpoint_descriptor &epd = id.endpoint[e];
                    if ((epd.bmAttributes & 0x3) == 3) // interrupt
                    {
                        if (epd.bEndpointAddress & 0x80)
                        {
                            epIn = epd.bEndpointAddress;
                            epInMaxPacket = epd.wMaxPacketSize;
                        }
                        else
                            epOut = epd.bEndpointAddress;
                    }
                }

                if (exactTarget && !epIn)
                    continue;
                if (automaticGtTarget &&
                    (epIn != RAYNEO_GT_HID_EP_IN || epOut != RAYNEO_GT_HID_EP_OUT || epInMaxPacket < 64))
                    continue;

                ctx->interfaceNumber = id.bInterfaceNumber;
                ctx->altSetting = id.bAlternateSetting;
                ctx->epIn = epIn;
                ctx->epOut = epOut;
                ctx->epInMaxPacket = epInMaxPacket;
                ctx->hasInterrupt = (epIn || epOut);
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    const bool automaticGtTarget = !exactTarget &&
        ctx->vid == RAYNEO_GT_VID && ctx->pid == RAYNEO_GT_PID;
    if ((exactTarget || automaticGtTarget) && ctx->interfaceNumber < 0)
    {
        libusb_close(ctx->handle);
        ctx->handle = nullptr;
        libusb_exit(ctx->usbCtx);
        ctx->usbCtx = nullptr;
        ctx->running.store(false);
        return RAYNEO_ERR_UNSUPPORTED;
    }
    if (ctx->interfaceNumber >= 0)
    {
        libusb_set_auto_detach_kernel_driver(ctx->handle, 1);

        int active = libusb_kernel_driver_active(ctx->handle, ctx->interfaceNumber);
        if (active == 1)
        {
            libusb_detach_kernel_driver(ctx->handle, ctx->interfaceNumber);
        }

        int cRc = libusb_claim_interface(ctx->handle, ctx->interfaceNumber);
        if (cRc != 0)
        {
            libusb_close(ctx->handle);
            ctx->handle = nullptr;
            libusb_exit(ctx->usbCtx);
            ctx->usbCtx = nullptr;
            ctx->running.store(false);
            return (cRc == LIBUSB_ERROR_BUSY) ? RAYNEO_ERR_BUSY : RAYNEO_ERR_IO;
        }
        if (ctx->altSetting > 0)
        {
            int altRc = libusb_set_interface_alt_setting(ctx->handle, ctx->interfaceNumber, ctx->altSetting);
            if (altRc != 0)
            {
                libusb_release_interface(ctx->handle, ctx->interfaceNumber);
                libusb_close(ctx->handle);
                ctx->handle = nullptr;
                libusb_exit(ctx->usbCtx);
                ctx->usbCtx = nullptr;
                ctx->running.store(false);
                return RAYNEO_ERR_IO;
            }
        }
    }
#else
    ctx->macReady = false;
    ctx->macReadyStatus = RAYNEO_OK;
    ctx->macDeviceOnline.store(false);
    try
    {
        ctx->worker = std::thread(macWorkerThread, ctx);
    }
    catch (...)
    {
        ctx->running.store(false);
        return RAYNEO_ERR_GENERAL;
    }
    RAYNEO_Result startStatus = RAYNEO_OK;
    {
        std::unique_lock<std::mutex> lk(ctx->macReadyMtx);
        if (!ctx->macReadyCv.wait_for(lk, std::chrono::seconds(5), [&]
                                      { return ctx->macReady; }))
        {
            startStatus = RAYNEO_ERR_TIMEOUT;
        }
        else
        {
            startStatus = ctx->macReadyStatus;
        }
    }
    if (startStatus != RAYNEO_OK)
    {
        ctx->stopWorker.store(true);
        if (ctx->hidRunLoop)
            CFRunLoopWakeUp(ctx->hidRunLoop);
        if (ctx->worker.joinable())
            ctx->worker.join();
        ctx->running.store(false);
        return startStatus;
    }
#endif
    // Start worker thread (only if we have IN endpoint)
#ifndef __APPLE__
    if (ctx->handle && ctx->epIn)
    {
        ctx->stopWorker.store(false);
        // Prepare async transfer
        ctx->inTransfer = libusb_alloc_transfer(0);
        if (!ctx->inTransfer)
        {
            libusb_release_interface(ctx->handle, ctx->interfaceNumber);
            libusb_close(ctx->handle);
            ctx->handle = nullptr;
            libusb_exit(ctx->usbCtx);
            ctx->usbCtx = nullptr;
            ctx->running.store(false);
            return RAYNEO_ERR_GENERAL;
        }
        static auto transferCallback = [](libusb_transfer *tr)
        {
            auto *ctx = static_cast<RayneoContext__ *>(tr->user_data);
            if (!ctx)
                return;

            uint8_t frame[64]{};
            bool haveFrame = false;

            if (tr->status == LIBUSB_TRANSFER_COMPLETED && tr->actual_length >= 64)
            {
                std::memcpy(frame, tr->buffer, 64);
                haveFrame = true;
            }
            else if (tr->status == LIBUSB_TRANSFER_NO_DEVICE)
            {
                ctx->transferResubmit.store(false);
                ctx->stopWorker.store(true);
                ctx->transferDone.store(true);
                emitDetachedOnce(ctx);
                return;
            }

            ctx->transferDone.store(true);
            if (!ctx->stopWorker.load() && ctx->transferResubmit.load())
            {
                ctx->transferDone.store(false);
                int r = libusb_submit_transfer(tr);
                if (r != 0)
                {
                    ctx->transferDone.store(true);
                    ctx->transferResubmit.store(false);
                    ctx->stopWorker.store(true);
                    RAYNEO_Event evt{};
                    evt.type = RAYNEO_EVENT_ERROR;
                    evt.seq = ++ctx->seq;
                    evt.data.error.code = r;
                    enqueueEvent(ctx, evt);
                }
            }

            if (haveFrame)
                processInboundFrame(ctx, frame, sizeof(frame));
        };
        size_t inSize = ctx->epInMaxPacket ? ctx->epInMaxPacket : 64;
        if (inSize < 64)
            inSize = 64;
        ctx->inBuffer.assign(inSize, 0);
        libusb_fill_interrupt_transfer(ctx->inTransfer, ctx->handle, ctx->epIn,
                                       ctx->inBuffer.data(), (int)inSize,
                                       transferCallback, ctx, 0);
        ctx->transferActive.store(false);
        ctx->transferDone.store(true);
        ctx->transferResubmit.store(false);
        try
        {
            ctx->worker = std::thread([ctx]()
            {
                while (!ctx->stopWorker.load() || !ctx->transferDone.load())
                {
                    timeval tv{0, 100000}; // 100ms
                    libusb_handle_events_timeout_completed(ctx->usbCtx, &tv, nullptr);
                }
            });
        }
        catch (...)
        {
            libusb_free_transfer(ctx->inTransfer);
            ctx->inTransfer = nullptr;
            libusb_release_interface(ctx->handle, ctx->interfaceNumber);
            libusb_close(ctx->handle);
            ctx->handle = nullptr;
            libusb_exit(ctx->usbCtx);
            ctx->usbCtx = nullptr;
            ctx->running.store(false);
            return RAYNEO_ERR_GENERAL;
        }

        ctx->transferDone.store(false);
        ctx->transferResubmit.store(true);
        int submitRc = libusb_submit_transfer(ctx->inTransfer);
        if (submitRc != 0)
        {
            ctx->transferDone.store(true);
            ctx->transferResubmit.store(false);
            ctx->stopWorker.store(true);
            if (ctx->worker.joinable())
                ctx->worker.join();
            libusb_free_transfer(ctx->inTransfer);
            ctx->inTransfer = nullptr;
            libusb_release_interface(ctx->handle, ctx->interfaceNumber);
            libusb_close(ctx->handle);
            ctx->handle = nullptr;
            libusb_exit(ctx->usbCtx);
            ctx->usbCtx = nullptr;
            ctx->running.store(false);
            return RAYNEO_ERR_IO;
        }
        ctx->transferActive.store(true);
    }
#endif

    {
        RAYNEO_Event evt{};
        evt.type = RAYNEO_EVENT_DEVICE_ATTACHED;
        evt.seq = attachedSeq;

        std::lock_guard<std::mutex> lk(ctx->qMtx);
        if (ctx->queue.size() >= ctx->maxQueue)
            ctx->queue.pop_back();
        ctx->queue.insert(ctx->queue.begin(), evt);
    }
    ctx->qCv.notify_one();

    if (ctx->cb)
    {
        const auto callback = ctx->cb;
        void *const callbackUser = ctx->cbUser;
        try
        {
            ctx->callbackWorker = std::thread([ctx, callback, callbackUser]()
            {
                for (;;)
                {
                    RAYNEO_Event evt{};
                    {
                        std::unique_lock<std::mutex> lk(ctx->qMtx);
                        ctx->qCv.wait(lk, [&]
                                      { return ctx->stopCallback.load() || !ctx->queue.empty(); });
                        if (ctx->queue.empty())
                        {
                            if (ctx->stopCallback.load())
                                return;
                            continue;
                        }
                        evt = ctx->queue.front();
                        ctx->queue.erase(ctx->queue.begin());
                    }
                    callback(&evt, callbackUser);
                }
            });
        }
        catch (...)
        {
            Rayneo_Stop(ctx);
            return RAYNEO_ERR_GENERAL;
        }
    }

    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_Stop(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;

    const auto self = std::this_thread::get_id();
    const bool fromTransportWorker =
        ctx->worker.joinable() && self == ctx->worker.get_id();
    const bool fromCallback =
        ctx->callbackWorker.joinable() && self == ctx->callbackWorker.get_id();

    // Transport cleanup cannot join the transport worker from itself.
    if (fromTransportWorker)
        return RAYNEO_ERR_BUSY;

    const bool firstStop = !ctx->stopIssued.exchange(true);
    if (firstStop)
    {
        const bool was = ctx->running.exchange(false);
        ctx->rtCv.notify_all();
        ctx->gtBiasCv.notify_all();
        ctx->gtOrientationCv.notify_all();
        {
            std::lock_guard<std::mutex> lk(ctx->gtBiasMtx);
            ctx->gtBiasCollecting = false;
        }
        {
            std::lock_guard<std::mutex> lk(ctx->trackingMtx);
            ctx->trackingInitialized = false;
            ctx->trackingActive.store(false);
            ctx->vqf.reset();
            ctx->trackingHaveTick = false;
            ctx->trackingHaveMagTick = false;
            ctx->trackingEverFedMag = false;
        }
        std::unique_lock<std::mutex> commandLock(ctx->commandMtx);
        if (was)
        {
            ctx->stopWorker.store(true);
#ifndef __APPLE__
            ctx->transferResubmit.store(false);
            if (ctx->inTransfer && ctx->transferActive.load() && !ctx->transferDone.load())
                libusb_cancel_transfer(ctx->inTransfer);
#endif
#ifdef __APPLE__
            if (ctx->hidRunLoop)
                CFRunLoopWakeUp(ctx->hidRunLoop);
#endif
            if (ctx->worker.joinable())
                ctx->worker.join();
#ifndef __APPLE__
            if (ctx->inTransfer)
            {
                libusb_free_transfer(ctx->inTransfer);
                ctx->inTransfer = nullptr;
            }
            if (ctx->handle && ctx->interfaceNumber >= 0)
                libusb_release_interface(ctx->handle, ctx->interfaceNumber);
            if (ctx->handle)
            {
                libusb_close(ctx->handle);
                ctx->handle = nullptr;
            }
            if (ctx->usbCtx)
            {
                libusb_exit(ctx->usbCtx);
                ctx->usbCtx = nullptr;
            }
#else
            ctx->macReadyStatus = RAYNEO_OK;
            ctx->macDeviceOnline.store(false);
            macReleaseDevice(ctx);
            macCleanupManager(ctx);
#endif
            commandLock.unlock();
            emitDetachedOnce(ctx);
        }
        else
        {
            commandLock.unlock();
        }
    }

    // Stop from the callback cannot join itself. The owning thread reaps the
    // finished dispatcher on a later Stop, Start, SetEventCallback or Destroy.
    if (ctx->callbackWorker.joinable())
    {
        ctx->stopCallback.store(true);
        ctx->qCv.notify_all();
        if (!fromCallback)
            ctx->callbackWorker.join();
    }

    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_PollEvent(RAYNEO_Context ctx, RAYNEO_Event *outEvent, uint32_t timeoutMs)
{
    if (!ctx || !outEvent)
        return RAYNEO_ERR_INVALID_ARG;
    if (ctx->cb)
        return RAYNEO_ERR_BUSY;
    std::unique_lock<std::mutex> lk(ctx->qMtx);
    ctx->qCv.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&]{ return !ctx->queue.empty(); });
    if (ctx->queue.empty())
        return RAYNEO_ERR_TIMEOUT;
    *outEvent = ctx->queue.front();
    ctx->queue.erase(ctx->queue.begin());
    return RAYNEO_OK;
}

static RAYNEO_Result sendRawTransport(RAYNEO_Context ctx, const uint8_t frame[64])
{
    if (!ctx || !frame)
        return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load())
        return RAYNEO_ERR_NO_DEVICE;
#ifndef __APPLE__
    if (!ctx->handle)
        return RAYNEO_ERR_NO_DEVICE;
    
    int transferred = 0;
    int rc = -1;
    if (ctx->epOut)
    {
        rc = libusb_interrupt_transfer(ctx->handle, ctx->epOut, const_cast<uint8_t *>(frame), 64, &transferred, 500);
        
        if (rc == 0 && transferred >= 64)
            return RAYNEO_OK;
    }
    // Fallback control: SET_REPORT Output report ID 0
    uint16_t wValue = (0x02 << 8) | 0x00; // Output report
    rc = libusb_control_transfer(ctx->handle, 0x21, 0x09, wValue, ctx->interfaceNumber >= 0 ? ctx->interfaceNumber : 0,
                                 const_cast<uint8_t *>(frame), 64, 500);
    
    if (rc == 64)
        return RAYNEO_OK;
    return RAYNEO_ERR_IO;
#else
    if (!ctx->macDeviceOnline.load())
        return RAYNEO_ERR_NO_DEVICE;
    IOHIDDeviceRef dev = nullptr;
    {
        std::lock_guard<std::mutex> lk(ctx->hidMtx);
        if (!ctx->hidDevice)
            return RAYNEO_ERR_NO_DEVICE;
        dev = ctx->hidDevice;
        CFRetain(dev);
    }
    IOReturn rc = IOHIDDeviceSetReport(dev, kIOHIDReportTypeOutput, 0, frame, 64);
    CFRelease(dev);
    return (rc == kIOReturnSuccess) ? RAYNEO_OK : RAYNEO_ERR_IO;
#endif
}

RAYNEO_Result Rayneo_SendRaw(RAYNEO_Context ctx, const uint8_t frame[64])
{
    if (!ctx || !frame)
        return RAYNEO_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(ctx->commandMtx);
    return sendRawTransport(ctx, frame);
}

static void buildFrame(uint8_t command, uint8_t value, const void *payload, size_t payloadLen, uint8_t out[64])
{
    std::memset(out, 0, 64);
    out[0] = 0x66;
    out[1] = command;
    out[2] = value;
    if (payload && payloadLen)
    {
        if (payloadLen > 52)
            payloadLen = 52;
        std::memcpy(out + 3, payload, payloadLen);
    }
}

RAYNEO_Result Rayneo_SendCommand(RAYNEO_Context ctx, uint8_t command, uint8_t value, const void *payload, size_t payloadLen)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    uint8_t frame[64];
    buildFrame(command, value, payload, payloadLen, frame);
    return Rayneo_SendRaw(ctx, frame);
}

RAYNEO_Result Rayneo_RoundTrip(RAYNEO_Context ctx, uint8_t command, uint8_t value, const void *payload, size_t payloadLen, uint32_t timeoutMs, uint8_t outFrame[64])
{
    if (!ctx || !outFrame)
        return RAYNEO_ERR_INVALID_ARG;

    std::unique_lock<std::mutex> commandLock(ctx->commandMtx);

    uint8_t frame[64];
    buildFrame(command, value, payload, payloadLen, frame);

    {
        std::lock_guard<std::mutex> lk(ctx->rtMtx);
        ctx->rtCommand = command;
        ctx->rtPending = true;
        ctx->rtReady = false;
    }

    auto rc = sendRawTransport(ctx, frame);
    if (rc != RAYNEO_OK)
    {
        std::lock_guard<std::mutex> lk(ctx->rtMtx);
        ctx->rtPending = false;
        ctx->rtReady = false;
        return rc;
    }

    std::unique_lock<std::mutex> lk(ctx->rtMtx);
    const auto waitTime = std::chrono::milliseconds(timeoutMs == 0 ? 500 : timeoutMs);
    if (!ctx->rtCv.wait_for(lk, waitTime, [&]
                            { return ctx->rtReady || !ctx->running.load(); }))
    {
        ctx->rtPending = false;
        ctx->rtReady = false;
        return RAYNEO_ERR_TIMEOUT;
    }
    if (!ctx->rtReady)
    {
        ctx->rtPending = false;
        return RAYNEO_ERR_NO_DEVICE;
    }

    std::memcpy(outFrame, ctx->rtFrame, 64);
    ctx->rtReady = false;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_EnableImu(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    // Command 0x01 = IMU ON (по соглашению из клиента)
    return Rayneo_SendCommand(ctx, kCmdImuOn, 0x00, nullptr, 0);
}

RAYNEO_Result Rayneo_DisableImu(RAYNEO_Context ctx) { return Rayneo_SendCommand(ctx, kCmdImuOff, 0x00, nullptr, 0); }
RAYNEO_Result Rayneo_RequestDeviceInfo(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    return Rayneo_SendCommand(ctx, kCmdDeviceInfo, 0x00, nullptr, 0);
}
// Set display into 3D (stereoscopic) mode.
// Command IDs from XRDeviceState.h: kCmdDisplay3dMode = 6, kCmdDisplay2dMode = 7
// We send 0x66 magic, command=6, value=0, no payload (same pattern as IMU enable).
RAYNEO_Result Rayneo_DisplaySet3D(RAYNEO_Context ctx)
{
    if (!ctx) return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load()) return RAYNEO_ERR_NO_DEVICE;
    return Rayneo_SendCommand(ctx, kCmdDisplay3dMode, 0x00, nullptr, 0);
}

// Set display into 2D (mono mirror) mode.
RAYNEO_Result Rayneo_DisplaySet2D(RAYNEO_Context ctx)
{
    if (!ctx) return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load()) return RAYNEO_ERR_NO_DEVICE;
    return Rayneo_SendCommand(ctx, kCmdDisplay2dMode, 0x00, nullptr, 0);
}


static bool decodeGtFactoryCalibrationFrame(const uint8_t frame[64], RAYNEO_GtFactoryCalibration &calibration)
{
    if (!frame || frame[0] != 0x99 || frame[1] != RAYNEO_PROTO_ACK_COMMAND || frame[8] != kCmdImuCalibration)
        return false;
    if (frame[9] == 0xFF)
        return false;

    calibration = {};
    std::memcpy(calibration.raw, frame + 9, sizeof(calibration.raw));
    for (int i = 0; i < 9; ++i)
        std::memcpy(&calibration.transform[i], frame + 9 + i * 4, 4);
    for (int i = 0; i < 3; ++i)
        std::memcpy(&calibration.accelOffset[i], frame + 9 + (9 + i) * 4, 4);

    for (float v : calibration.transform)
        if (!std::isfinite(v))
            return false;
    for (float v : calibration.accelOffset)
        if (!std::isfinite(v))
            return false;

    const float *m = calibration.transform;
    const float det = m[0] * (m[4] * m[8] - m[5] * m[7])
                    - m[1] * (m[3] * m[8] - m[5] * m[6])
                    + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (!std::isfinite(det) || std::fabs(det) < 1.0e-6f)
        return false;

    calibration.valid = 1;
    return true;
}

static bool decodeGtGyroBiasFrame(const uint8_t frame[64], RAYNEO_GtGyroBiasTable &table)
{
    if (!frame || frame[0] != 0x99 || frame[1] != RAYNEO_PROTO_ACK_COMMAND || frame[8] != kCmdGetGyroBias)
        return false;
    if (frame[9] == 0xFF)
        return false;

    // RayNeo XR 2.1.1: [9]=page index, [10]=page count,
    // [11]=signed first temperature C, [12]=entry count,
    // [13..]=entryCount * XYZ float32.
    const unsigned pageIndex = frame[9];
    const unsigned pageCount = frame[10];
    const int startTemp = static_cast<int>(static_cast<int8_t>(frame[11]));
    const unsigned count = frame[12];
    if (pageCount == 0 || pageIndex >= pageCount || count == 0 || count > 4 || 13u + count * 12u > 64u)
        return false;

    for (unsigned i = 0; i < count; ++i)
    {
        const int temp = startTemp + static_cast<int>(i);
        if (temp < RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C || temp > RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C)
            return false;
        const int index = temp - RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C;
        float bias[3]{};
        for (int axis = 0; axis < 3; ++axis)
        {
            std::memcpy(&bias[axis], frame + 13 + i * 12 + axis * 4, 4);
            if (!std::isfinite(bias[axis]))
                return false;
            table.biasRadPerSec[index][axis] = bias[axis];
        }
        if (!table.validEntry[index])
        {
            table.validEntry[index] = 1;
            ++table.validCount;
        }
    }

    table.complete = (table.validCount == RAYNEO_GT_GYRO_BIAS_TABLE_COUNT) ? 1 : 0;
    return true;
}

RAYNEO_Result Rayneo_QueryGtFactoryCalibration(RAYNEO_Context ctx,
                                                uint32_t timeoutMs,
                                                RAYNEO_GtFactoryCalibration *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load())
        return RAYNEO_ERR_NO_DEVICE;
    if (ctx->vid != RAYNEO_GT_VID || ctx->pid != RAYNEO_GT_PID)
        return RAYNEO_ERR_UNSUPPORTED;

    uint8_t frame[64]{};
    const RAYNEO_Result rc = Rayneo_RoundTrip(ctx, kCmdImuCalibration, 0x00,
                                              nullptr, 0, timeoutMs, frame);
    if (rc != RAYNEO_OK)
        return rc;

    RAYNEO_GtFactoryCalibration calibration{};
    if (!decodeGtFactoryCalibrationFrame(frame, calibration))
        return RAYNEO_ERR_IO;
    *out = calibration;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_QueryGtGyroBiasTable(RAYNEO_Context ctx,
                                           uint32_t timeoutMs,
                                           RAYNEO_GtGyroBiasTable *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load())
        return RAYNEO_ERR_NO_DEVICE;
    if (ctx->vid != RAYNEO_GT_VID || ctx->pid != RAYNEO_GT_PID)
        return RAYNEO_ERR_UNSUPPORTED;

    // Request -20..+60 C and collect all 0x3E reply pages.
    std::unique_lock<std::mutex> commandLock(ctx->commandMtx);
    {
        std::lock_guard<std::mutex> lk(ctx->gtBiasMtx);
        if (ctx->gtBiasCollecting)
            return RAYNEO_ERR_BUSY;
        ctx->gtBiasCollectTable = {};
        ctx->gtBiasCollecting = true;
    }

    const uint8_t rangeCount = static_cast<uint8_t>(RAYNEO_GT_GYRO_BIAS_TABLE_COUNT);
    uint8_t frame[64]{};
    buildFrame(kCmdGetGyroBias,
               static_cast<uint8_t>(static_cast<int8_t>(RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C)),
               &rangeCount, sizeof(rangeCount), frame);
    const RAYNEO_Result sendRc = sendRawTransport(ctx, frame);
    if (sendRc != RAYNEO_OK)
    {
        std::lock_guard<std::mutex> lk(ctx->gtBiasMtx);
        ctx->gtBiasCollecting = false;
        return sendRc;
    }

    const auto waitTime = std::chrono::milliseconds(timeoutMs == 0 ? 1000 : timeoutMs);
    std::unique_lock<std::mutex> lk(ctx->gtBiasMtx);
    const bool completed = ctx->gtBiasCv.wait_for(lk, waitTime, [&]
    {
        return ctx->gtBiasCollectTable.complete || !ctx->running.load();
    });
    ctx->gtBiasCollecting = false;

    if (!ctx->running.load())
        return RAYNEO_ERR_NO_DEVICE;
    if (!completed || !ctx->gtBiasCollectTable.complete)
        return RAYNEO_ERR_TIMEOUT;

    *out = ctx->gtBiasCollectTable;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GetGtGyroBiasAtTemperature(const RAYNEO_GtGyroBiasTable *table,
                                                 float temperatureC,
                                                 float outBiasRadPerSec[3])
{
    if (!table || !outBiasRadPerSec || !std::isfinite(temperatureC) || table->validCount == 0)
        return RAYNEO_ERR_INVALID_ARG;

    const float clamped = std::max(static_cast<float>(RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C),
                                   std::min(static_cast<float>(RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C), temperatureC));
    const int loTemp = static_cast<int>(std::floor(clamped));
    const int hiTemp = static_cast<int>(std::ceil(clamped));

    auto validAt = [&](int temp) -> bool
    {
        if (temp < RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C || temp > RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C)
            return false;
        return table->validEntry[temp - RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C] != 0;
    };

    int lo = loTemp;
    while (lo >= RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C && !validAt(lo))
        --lo;
    int hi = hiTemp;
    while (hi <= RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C && !validAt(hi))
        ++hi;
    if (lo < RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C)
    {
        lo = hi;
        if (lo > RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C)
            return RAYNEO_ERR_IO;
    }
    if (hi > RAYNEO_GT_GYRO_BIAS_MAX_TEMP_C)
        hi = lo;

    const int loIndex = lo - RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C;
    const int hiIndex = hi - RAYNEO_GT_GYRO_BIAS_MIN_TEMP_C;
    const float alpha = (hi == lo) ? 0.0f
                                    : (clamped - static_cast<float>(lo)) / static_cast<float>(hi - lo);
    for (int axis = 0; axis < 3; ++axis)
        outBiasRadPerSec[axis] = table->biasRadPerSec[loIndex][axis] * (1.0f - alpha)
                               + table->biasRadPerSec[hiIndex][axis] * alpha;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_ApplyGtSensorCalibration(const RAYNEO_GtFactoryCalibration *calibration,
                                               const RAYNEO_GtGyroBiasTable *biasTable,
                                               const RAYNEO_ImuSample *input,
                                               RAYNEO_ImuSample *output)
{
    if (!calibration || !input || !output || !calibration->valid || !input->valid)
        return RAYNEO_ERR_INVALID_ARG;

    float gyroBias[3]{};
    if (biasTable && biasTable->validCount)
    {
        const RAYNEO_Result biasRc = Rayneo_GetGtGyroBiasAtTemperature(
            biasTable, input->temperature, gyroBias);
        if (biasRc != RAYNEO_OK)
            return biasRc;
    }

    RAYNEO_ImuSample corrected = *input;
    float gyroUnbiasedRad[3]{};
    for (int i = 0; i < 3; ++i)
        gyroUnbiasedRad[i] = input->gyroRad[i] - gyroBias[i];

    for (int row = 0; row < 3; ++row)
    {
        corrected.acc[row] = calibration->accelOffset[row];
        corrected.gyroRad[row] = 0.0f;
        for (int col = 0; col < 3; ++col)
        {
            const float m = calibration->transform[row * 3 + col];
            corrected.acc[row] += m * input->acc[col];
            corrected.gyroRad[row] += m * gyroUnbiasedRad[col];
        }
        corrected.gyroDps[row] = corrected.gyroRad[row] * 57.295779513082320876f;
    }

    *output = corrected;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_ApplyGtRuntimeAxes(const RAYNEO_ImuSample *input,
                                        RAYNEO_ImuSample *output)
{
    if (!input || !output || !input->valid)
        return RAYNEO_ERR_INVALID_ARG;

    RAYNEO_ImuSample mapped = *input;
    // GT/Gemini runtime mapping: [x,y,z] -> [x,-z,y].
    mapped.acc[0] = input->acc[0];
    mapped.acc[1] = -input->acc[2];
    mapped.acc[2] = input->acc[1];
    mapped.gyroRad[0] = input->gyroRad[0];
    mapped.gyroRad[1] = -input->gyroRad[2];
    mapped.gyroRad[2] = input->gyroRad[1];
    mapped.gyroDps[0] = input->gyroDps[0];
    mapped.gyroDps[1] = -input->gyroDps[2];
    mapped.gyroDps[2] = input->gyroDps[1];
    mapped.magnet[0] = input->magnet[0];
    mapped.magnet[1] = -input->magnet[2];
    mapped.magnet[2] = input->magnet[1];
    *output = mapped;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_PrepareGtImuSample(const RAYNEO_GtFactoryCalibration *calibration,
                                        const RAYNEO_GtGyroBiasTable *biasTable,
                                        const RAYNEO_ImuSample *input,
                                        RAYNEO_ImuSample *output)
{
    if (!calibration || !input || !output)
        return RAYNEO_ERR_INVALID_ARG;
    RAYNEO_ImuSample calibrated{};
    const RAYNEO_Result rc = Rayneo_ApplyGtSensorCalibration(calibration, biasTable, input, &calibrated);
    if (rc != RAYNEO_OK)
        return rc;
    return Rayneo_ApplyGtRuntimeAxes(&calibrated, output);
}

static bool isFiniteVector3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

static float vectorNorm3(const float v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static bool validFusionMode(int32_t mode)
{
    return mode == RAYNEO_GT_FUSION_6D ||
           mode == RAYNEO_GT_FUSION_AUTO_9D ||
           mode == RAYNEO_GT_FUSION_FORCE_9D;
}

static void makeIdentityGtCalibration(RAYNEO_GtFactoryCalibration &calibration)
{
    calibration = {};
    calibration.transform[0] = 1.0f;
    calibration.transform[4] = 1.0f;
    calibration.transform[8] = 1.0f;
    calibration.valid = 1;
}

static bool rebuildGtFusionLocked(RayneoContext__ *ctx)
{
    if (!ctx || !validFusionMode(ctx->trackingConfig.fusionMode))
        return false;

    VQFParams params;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    params.motionBiasEstEnabled = true;
#endif
    params.restBiasEstEnabled = true;
    params.magDistRejectionEnabled = ctx->trackingConfig.fusionMode != RAYNEO_GT_FUSION_FORCE_9D;

    try
    {
        ctx->vqf = std::make_unique<VQF>(params,
                                         1.0f / RAYNEO_GT_VQF_IMU_HZ,
                                         1.0f / RAYNEO_GT_VQF_IMU_HZ,
                                         1.0f / RAYNEO_GT_VQF_MAG_HZ);
    }
    catch (...)
    {
        ctx->vqf.reset();
        return false;
    }

    ctx->trackingHaveTick = false;
    ctx->trackingHaveMagTick = false;
    ctx->trackingEverFedMag = false;
    ctx->trackingLastTick = 0;
    ctx->trackingLastMagTick = 0;
    ctx->trackingTimestampNs = 0;
    return true;
}

static void transformTemperatureBiasToRuntime(const RAYNEO_GtFactoryCalibration &calibration,
                                              const float packageBias[3],
                                              float runtimeBias[3])
{
    float transformed[3]{};
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            transformed[row] += calibration.transform[row * 3 + col] * packageBias[col];

    runtimeBias[0] = transformed[0];
    runtimeBias[1] = -transformed[2];
    runtimeBias[2] = transformed[1];
}

static void enqueueGtOrientation(RayneoContext__ *ctx, const RAYNEO_GtOrientation &orientation)
{
    if (!ctx)
        return;
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        if (ctx->gtOrientationQueue.size() >= ctx->maxGtOrientationQueue)
            ctx->gtOrientationQueue.pop_front();
        ctx->gtOrientationQueue.push_back(orientation);
        ctx->lastGtOrientation = orientation;
    }
    ctx->gtOrientationCv.notify_one();
}

static void processGtOrientation(RayneoContext__ *ctx, const RAYNEO_ImuSample &sample)
{
    if (!ctx || !sample.valid || ctx->vid != RAYNEO_GT_VID || ctx->pid != RAYNEO_GT_PID)
        return;

    RAYNEO_GtOrientation orientation{};
    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        if (!ctx->trackingInitialized || !ctx->vqf)
            return;

        RAYNEO_ImuSample prepared{};
        const RAYNEO_GtGyroBiasTable *biasTable =
            ctx->trackingBiasTable.validCount ? &ctx->trackingBiasTable : nullptr;
        if (Rayneo_PrepareGtImuSample(&ctx->trackingCalibration, biasTable, &sample, &prepared) != RAYNEO_OK)
            return;

        float packageTempBias[3]{};
        if (biasTable &&
            Rayneo_GetGtGyroBiasAtTemperature(biasTable, sample.temperature, packageTempBias) == RAYNEO_OK)
        {
            transformTemperatureBiasToRuntime(ctx->trackingCalibration, packageTempBias,
                                              orientation.appliedTemperatureBiasRadPerSec);
        }

        float mag[3] = {prepared.magnet[0], prepared.magnet[1], prepared.magnet[2]};
        if (ctx->trackingMagCalibration.valid)
        {
            for (int i = 0; i < 3; ++i)
                mag[i] = (mag[i] - ctx->trackingMagCalibration.hardIron[i]) *
                         ctx->trackingMagCalibration.scale[i];
        }

        const bool magFinite = isFiniteVector3(mag);
        const float magNorm = magFinite ? vectorNorm3(mag) : 0.0f;
        const bool magUsable = magFinite && std::isfinite(magNorm) && magNorm > 1.0e-6f;

        bool didReset = false;
        if (!ctx->trackingHaveTick)
        {
            ctx->trackingHaveTick = true;
            ctx->trackingLastTick = sample.tick;
            ctx->trackingTimestampNs = 0;
            didReset = true;
        }
        else
        {
            const uint32_t deltaTicks = static_cast<uint32_t>(sample.tick - ctx->trackingLastTick);
            ctx->trackingLastTick = sample.tick;
            ctx->trackingTimestampNs += static_cast<uint64_t>(deltaTicks) *
                                        static_cast<uint64_t>(RAYNEO_GT_SENSOR_TICK_US) * 1000ull;
            const uint64_t maxGapTicks =
                (static_cast<uint64_t>(RAYNEO_GT_FUSION_MAX_GAP_MS) * RAYNEO_GT_SENSOR_TICK_HZ + 999u) / 1000u;
            if (deltaTicks == 0 || deltaTicks > maxGapTicks)
            {
                ctx->vqf->resetState();
                ctx->trackingHaveMagTick = false;
                ctx->trackingEverFedMag = false;
                didReset = true;
            }
        }

        vqf_real_t gyr[3] = {prepared.gyroRad[0], prepared.gyroRad[1], prepared.gyroRad[2]};
        vqf_real_t acc[3] = {prepared.acc[0], prepared.acc[1], prepared.acc[2]};
        ctx->vqf->updateGyr(gyr);
        ctx->vqf->updateAcc(acc);

        const bool wantsMag = ctx->trackingConfig.fusionMode != RAYNEO_GT_FUSION_6D;
        if (wantsMag && magUsable)
        {
            const uint32_t magPeriodTicks = std::max<uint32_t>(
                1u, static_cast<uint32_t>(std::lround(
                    static_cast<double>(RAYNEO_GT_SENSOR_TICK_HZ) / RAYNEO_GT_VQF_MAG_HZ)));
            const uint32_t sinceMag = ctx->trackingHaveMagTick
                                          ? static_cast<uint32_t>(sample.tick - ctx->trackingLastMagTick)
                                          : magPeriodTicks;
            if (!ctx->trackingHaveMagTick || sinceMag >= magPeriodTicks)
            {
                vqf_real_t m[3] = {mag[0], mag[1], mag[2]};
                ctx->vqf->updateMag(m);
                ctx->trackingHaveMagTick = true;
                ctx->trackingEverFedMag = true;
                ctx->trackingLastMagTick = sample.tick;
            }
        }

        vqf_real_t q[4]{};
        if (wantsMag && ctx->trackingEverFedMag)
            ctx->vqf->getQuat9D(q);
        else
            ctx->vqf->getQuat6D(q);

        vqf_real_t residualBias[3]{};
        (void)ctx->vqf->getBiasEstimate(residualBias);

        for (int i = 0; i < 4; ++i)
            orientation.quat[i] = static_cast<float>(q[i]);
        orientation.timestampNs = ctx->trackingTimestampNs;
        orientation.tick = sample.tick;
        orientation.count = sample.count;
        orientation.temperature = sample.temperature;
        for (int i = 0; i < 3; ++i)
        {
            orientation.residualBiasRadPerSec[i] = static_cast<float>(residualBias[i]);
            orientation.magnetRuntime[i] = mag[i];
        }
        orientation.magnetFieldStrength = magNorm;
        orientation.fusionMode = ctx->trackingConfig.fusionMode;
        orientation.valid = 1;
        orientation.magneticDisturbance =
            wantsMag && ctx->trackingEverFedMag && ctx->vqf->getMagDistDetected() ? 1 : 0;
        orientation.restDetected = ctx->vqf->getRestDetected() ? 1 : 0;
        orientation.usingMagnetometer =
            wantsMag && ctx->trackingEverFedMag &&
            (ctx->trackingConfig.fusionMode == RAYNEO_GT_FUSION_FORCE_9D || !orientation.magneticDisturbance)
                ? 1 : 0;
        orientation.factoryCalibrationValid = ctx->trackingFactoryFromDevice ? 1 : 0;
        orientation.temperatureBiasValid = ctx->trackingBiasFromDevice ? 1 : 0;
        orientation.magnetometerCalibrationValid = ctx->trackingMagCalibration.valid ? 1 : 0;
        orientation.fusionReset = didReset ? 1 : 0;
    }

    enqueueGtOrientation(ctx, orientation);
}

void Rayneo_GtTrackingConfigInit(RAYNEO_GtTrackingConfig *config)
{
    if (!config)
        return;
    *config = {};
    config->structSize = sizeof(*config);
    config->fusionMode = RAYNEO_GT_FUSION_6D;
    config->requireFactoryCalibration = 1;
    config->requireTemperatureBias = 0;
}

RAYNEO_Result Rayneo_GtInitializeTracking(RAYNEO_Context ctx,
                                           const RAYNEO_GtTrackingConfig *config,
                                           uint32_t timeoutMs)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->running.load())
        return RAYNEO_ERR_NO_DEVICE;
    if (ctx->vid != RAYNEO_GT_VID || ctx->pid != RAYNEO_GT_PID)
        return RAYNEO_ERR_UNSUPPORTED;

    RAYNEO_GtTrackingConfig cfg{};
    Rayneo_GtTrackingConfigInit(&cfg);
    if (config)
    {
        if (config->structSize < sizeof(RAYNEO_GtTrackingConfig) || !validFusionMode(config->fusionMode))
            return RAYNEO_ERR_INVALID_ARG;
        cfg = *config;
    }

    RAYNEO_GtFactoryCalibration calibration{};
    bool factoryFromDevice = false;
    RAYNEO_Result rc = Rayneo_QueryGtFactoryCalibration(ctx, timeoutMs, &calibration);
    if (rc == RAYNEO_OK)
        factoryFromDevice = true;
    else if (cfg.requireFactoryCalibration)
        return rc;
    else
        makeIdentityGtCalibration(calibration);

    RAYNEO_GtGyroBiasTable biasTable{};
    bool biasFromDevice = false;
    rc = Rayneo_QueryGtGyroBiasTable(ctx, timeoutMs, &biasTable);
    if (rc == RAYNEO_OK && biasTable.complete)
        biasFromDevice = true;
    else if (cfg.requireTemperatureBias)
        return rc == RAYNEO_OK ? RAYNEO_ERR_IO : rc;
    else
        biasTable = {};

    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        ctx->trackingConfig = cfg;
        ctx->trackingCalibration = calibration;
        ctx->trackingBiasTable = biasTable;
        ctx->trackingFactoryFromDevice = factoryFromDevice;
        ctx->trackingBiasFromDevice = biasFromDevice;
        if (!rebuildGtFusionLocked(ctx))
            return RAYNEO_ERR_GENERAL;
        ctx->trackingInitialized = true;
        ctx->trackingActive.store(true);
    }
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        ctx->gtOrientationQueue.clear();
        ctx->lastGtOrientation = {};
    }
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtShutdownTracking(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        ctx->trackingInitialized = false;
        ctx->trackingActive.store(false);
        ctx->vqf.reset();
        ctx->trackingHaveTick = false;
        ctx->trackingHaveMagTick = false;
        ctx->trackingEverFedMag = false;
    }
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        ctx->gtOrientationQueue.clear();
        ctx->lastGtOrientation = {};
    }
    ctx->gtOrientationCv.notify_all();
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtResetFusion(RAYNEO_Context ctx)
{
    if (!ctx)
        return RAYNEO_ERR_INVALID_ARG;
    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        if (!ctx->trackingInitialized)
            return RAYNEO_ERR_UNSUPPORTED;
        if (!rebuildGtFusionLocked(ctx))
            return RAYNEO_ERR_GENERAL;
    }
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        ctx->gtOrientationQueue.clear();
        ctx->lastGtOrientation = {};
    }
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtSetFusionMode(RAYNEO_Context ctx, RAYNEO_GtFusionMode mode)
{
    if (!ctx || !validFusionMode(static_cast<int32_t>(mode)))
        return RAYNEO_ERR_INVALID_ARG;
    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        if (!ctx->trackingInitialized)
            return RAYNEO_ERR_UNSUPPORTED;
        ctx->trackingConfig.fusionMode = static_cast<int32_t>(mode);
        if (!rebuildGtFusionLocked(ctx))
            return RAYNEO_ERR_GENERAL;
    }
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        ctx->gtOrientationQueue.clear();
        ctx->lastGtOrientation = {};
    }
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtSetMagCalibration(RAYNEO_Context ctx,
                                          const RAYNEO_GtMagCalibration *calibration)
{
    if (!ctx || !calibration)
        return RAYNEO_ERR_INVALID_ARG;

    RAYNEO_GtMagCalibration next{};
    if (calibration->valid)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(calibration->hardIron[i]) ||
                !std::isfinite(calibration->scale[i]) || calibration->scale[i] <= 0.0f)
                return RAYNEO_ERR_INVALID_ARG;
        }
        next = *calibration;
    }
    else
    {
        // Zero/invalid calibration disables host-side magnetometer correction.
        next.scale[0] = next.scale[1] = next.scale[2] = 1.0f;
        next.valid = 0;
    }

    bool resetFusion = false;
    {
        std::lock_guard<std::mutex> lk(ctx->trackingMtx);
        ctx->trackingMagCalibration = next;
        if (ctx->trackingInitialized)
        {
            if (!rebuildGtFusionLocked(ctx))
                return RAYNEO_ERR_GENERAL;
            resetFusion = true;
        }
    }
    if (resetFusion)
    {
        std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
        ctx->gtOrientationQueue.clear();
        ctx->lastGtOrientation = {};
    }
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtGetMagCalibration(RAYNEO_Context ctx, RAYNEO_GtMagCalibration *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(ctx->trackingMtx);
    *out = ctx->trackingMagCalibration;
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtPollOrientation(RAYNEO_Context ctx,
                                        RAYNEO_GtOrientation *out,
                                        uint32_t timeoutMs)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    if (!ctx->trackingActive.load())
        return RAYNEO_ERR_UNSUPPORTED;

    std::unique_lock<std::mutex> lk(ctx->gtOrientationMtx);
    ctx->gtOrientationCv.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&]
    {
        return !ctx->gtOrientationQueue.empty() || !ctx->running.load() || !ctx->trackingActive.load();
    });
    if (ctx->gtOrientationQueue.empty())
    {
        if (!ctx->running.load())
            return RAYNEO_ERR_NO_DEVICE;
        if (!ctx->trackingActive.load())
            return RAYNEO_ERR_UNSUPPORTED;
        return RAYNEO_ERR_TIMEOUT;
    }
    *out = ctx->gtOrientationQueue.front();
    ctx->gtOrientationQueue.pop_front();
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GtGetLastOrientation(RAYNEO_Context ctx, RAYNEO_GtOrientation *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(ctx->gtOrientationMtx);
    *out = ctx->lastGtOrientation;
    return out->valid ? RAYNEO_OK : RAYNEO_ERR_TIMEOUT;
}

RAYNEO_Result Rayneo_GetLastImu(RAYNEO_Context ctx, RAYNEO_ImuSample *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(ctx->snapshotMtx);
    *out = ctx->lastImu;
    return out->valid ? RAYNEO_OK : RAYNEO_ERR_TIMEOUT;
}

RAYNEO_Result Rayneo_GetLastImuFrame(RAYNEO_Context ctx, uint8_t outFrame[64])
{
    if (!ctx || !outFrame)
        return RAYNEO_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(ctx->snapshotMtx);
    if (!ctx->lastImuFrameValid)
        return RAYNEO_ERR_TIMEOUT;
    std::memcpy(outFrame, ctx->lastImuFrame, 64);
    return RAYNEO_OK;
}

RAYNEO_Result Rayneo_GetDeviceInfo(RAYNEO_Context ctx, RAYNEO_DeviceInfoMini *out)
{
    if (!ctx || !out)
        return RAYNEO_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(ctx->snapshotMtx);
    *out = ctx->lastInfo;
    return out->valid ? RAYNEO_OK : RAYNEO_ERR_TIMEOUT;
}

