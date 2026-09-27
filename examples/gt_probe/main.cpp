#include "rayneo_api.h"

#include <chrono>
#include <cstdio>
#include <cstring>

namespace {

RAYNEO_GtFusionMode parse_mode(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--auto-9d") == 0)
            return RAYNEO_GT_FUSION_AUTO_9D;
        if (std::strcmp(argv[i], "--force-9d") == 0)
            return RAYNEO_GT_FUSION_FORCE_9D;
    }
    return RAYNEO_GT_FUSION_6D;
}

const char* mode_name(RAYNEO_GtFusionMode mode)
{
    switch (mode)
    {
    case RAYNEO_GT_FUSION_AUTO_9D: return "auto-9d";
    case RAYNEO_GT_FUSION_FORCE_9D: return "force-9d";
    default: return "vendor-6d";
    }
}

void print_rc(const char* what, RAYNEO_Result rc)
{
    std::fprintf(stderr, "%s: %s (%d)\n", what, Rayneo_ResultToString(rc), static_cast<int>(rc));
}

} // namespace

int main(int argc, char** argv)
{
    const RAYNEO_GtFusionMode mode = parse_mode(argc, argv);
    std::printf("RayNeo GT probe - SDK %s - mode=%s\n", RAYNEO_VERSION_STRING, mode_name(mode));

    RAYNEO_VidPid devices[RAYNEO_SUPPORTED_DEVICE_COUNT]{};
    size_t deviceCount = 0;
    RAYNEO_Result rc = Rayneo_Discovery(devices, RAYNEO_SUPPORTED_DEVICE_COUNT, &deviceCount);
    if (rc != RAYNEO_OK)
    {
        print_rc("Rayneo_Discovery", rc);
        return 1;
    }

    bool gtFound = false;
    for (size_t i = 0; i < deviceCount && i < RAYNEO_SUPPORTED_DEVICE_COUNT; ++i)
    {
        std::printf("found %04X:%04X\n", unsigned(devices[i].vid), unsigned(devices[i].pid));
        if (devices[i].vid == RAYNEO_GT_VID && devices[i].pid == RAYNEO_GT_PID)
            gtFound = true;
    }
    if (!gtFound)
    {
        std::fprintf(stderr, "RayNeo GT (3941:AF50) not found.\n");
        return 2;
    }

    RAYNEO_Context ctx = nullptr;
    rc = Rayneo_Create(&ctx);
    if (rc != RAYNEO_OK)
    {
        print_rc("Rayneo_Create", rc);
        return 1;
    }

    auto cleanup = [&]() {
        if (!ctx) return;
        (void)Rayneo_DisableImu(ctx);
        (void)Rayneo_GtShutdownTracking(ctx);
        (void)Rayneo_Stop(ctx);
        Rayneo_Destroy(ctx);
        ctx = nullptr;
    };

    rc = Rayneo_SetTargetVidPid(ctx, RAYNEO_GT_VID, RAYNEO_GT_PID);
    if (rc != RAYNEO_OK)
    {
        print_rc("Rayneo_SetTargetVidPid", rc);
        cleanup();
        return 1;
    }

    // Use GT automatic interface/endpoint selection.
    rc = Rayneo_Start(ctx, RAYNEO_SERVICE_FLAG_AUTODETECT | RAYNEO_SERVICE_FLAG_ASYNC_IMU);
    if (rc != RAYNEO_OK)
    {
        print_rc("Rayneo_Start", rc);
        cleanup();
        return 1;
    }

    RAYNEO_GtFactoryCalibration calibration{};
    rc = Rayneo_QueryGtFactoryCalibration(ctx, 1000, &calibration);
    if (rc == RAYNEO_OK)
    {
        std::puts("0x3C Tsb:");
        for (int row = 0; row < 3; ++row)
            std::printf("  % .7f % .7f % .7f\n",
                        calibration.transform[row*3], calibration.transform[row*3+1], calibration.transform[row*3+2]);
        std::printf("0x3C accel offset: [% .7f % .7f % .7f]\n",
                    calibration.accelOffset[0], calibration.accelOffset[1], calibration.accelOffset[2]);
    }
    else
        print_rc("0x3C factory calibration", rc);

    RAYNEO_GtGyroBiasTable biasTable{};
    rc = Rayneo_QueryGtGyroBiasTable(ctx, 1500, &biasTable);
    if (rc == RAYNEO_OK)
    {
        std::printf("0x3E gyro temperature table: %u/%u entries, complete=%u\n",
                    unsigned(biasTable.validCount), unsigned(RAYNEO_GT_GYRO_BIAS_TABLE_COUNT),
                    unsigned(biasTable.complete));
    }
    else
        print_rc("0x3E gyro temperature table", rc);

    RAYNEO_GtTrackingConfig config{};
    Rayneo_GtTrackingConfigInit(&config);
    config.fusionMode = mode;
    rc = Rayneo_GtInitializeTracking(ctx, &config, 1500);
    if (rc != RAYNEO_OK)
    {
        print_rc("Rayneo_GtInitializeTracking", rc);
        cleanup();
        return 1;
    }

    rc = Rayneo_EnableImu(ctx);
    if (rc != RAYNEO_OK)
    {
        print_rc("Rayneo_EnableImu", rc);
        cleanup();
        return 1;
    }

    std::puts("Streaming fused orientation for 10 seconds; quaternion order [w x y z].");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    unsigned shown = 0;
    while (std::chrono::steady_clock::now() < deadline)
    {
        RAYNEO_GtOrientation o{};
        rc = Rayneo_GtPollOrientation(ctx, &o, 500);
        if (rc == RAYNEO_ERR_TIMEOUT)
            continue;
        if (rc != RAYNEO_OK)
        {
            print_rc("Rayneo_GtPollOrientation", rc);
            break;
        }
        if ((shown++ % 50u) != 0u)
            continue;
        std::printf("tick=%10u count=%10u T=%6.2f q=[% .6f % .6f % .6f % .6f] "
                    "Tbias=[% .5f % .5f % .5f] residual=[% .5f % .5f % .5f] "
                    "|mag|=%8.3f mag=%u disturbed=%u rest=%u cal=%u temp=%u reset=%u\n",
                    o.tick, o.count, o.temperature,
                    o.quat[0], o.quat[1], o.quat[2], o.quat[3],
                    o.appliedTemperatureBiasRadPerSec[0], o.appliedTemperatureBiasRadPerSec[1],
                    o.appliedTemperatureBiasRadPerSec[2],
                    o.residualBiasRadPerSec[0], o.residualBiasRadPerSec[1], o.residualBiasRadPerSec[2],
                    o.magnetFieldStrength, unsigned(o.usingMagnetometer), unsigned(o.magneticDisturbance),
                    unsigned(o.restDetected), unsigned(o.factoryCalibrationValid),
                    unsigned(o.temperatureBiasValid), unsigned(o.fusionReset));
    }

    cleanup();
    return 0;
}
