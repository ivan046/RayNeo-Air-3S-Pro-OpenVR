#!/usr/bin/env bash
set -euo pipefail

rm -rf build dist

cmake -S . -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=/opt/rayneo-toolchain/xwin-clang-cl.cmake \
    -DRAYNEO_BUILD_OPENVR_DRIVER=OFF \
    -DRAYNEO_BUILD_EXAMPLES=OFF \
    -DLIBUSB_INCLUDE_DIR=/opt/libusb/include \
    -DLIBUSB_LIBRARY=/opt/libusb/VS2022/MS64/dll/libusb-1.0.lib \
    -DVCPKG_INSTALLED_DIR=/opt/libusb-vcpkg \
    -DVCPKG_TARGET_TRIPLET=x64-windows

cmake --build build --target RayNeoSDK

PKG=dist/RayNeoSDK-${SDK_VERSION}-win64-msvc
mkdir -p "${PKG}/include" "${PKG}/bin" "${PKG}/lib"
cp include/rayneo_api.h "${PKG}/include/"
cp build/RayNeoSDK.dll "${PKG}/bin/"
cp build/RayNeoSDK.lib "${PKG}/lib/"
cp /opt/libusb/VS2022/MS64/dll/libusb-1.0.dll "${PKG}/bin/"

tar -C dist -czf ${PKG}.tar.gz RayNeoSDK-${SDK_VERSION}-win64-msvc
