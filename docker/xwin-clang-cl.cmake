set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR AMD64)

set(CMAKE_C_COMPILER clang-cl CACHE FILEPATH "")
set(CMAKE_CXX_COMPILER clang-cl CACHE FILEPATH "")
set(CMAKE_AR llvm-lib CACHE FILEPATH "")
set(CMAKE_LINKER lld-link CACHE FILEPATH "")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL" CACHE STRING "" FORCE)

set(XWIN_DIR "/opt/cargo-xwin-cache/xwin")
set(XWIN_ARCH "x86_64")
set(XWIN_TARGET "x86_64-pc-windows-msvc")

set(_XWIN_COMPILE_FLAGS
    --target=${XWIN_TARGET}
    -Wno-unused-command-line-argument
    -fuse-ld=lld-link
    "/imsvc ${XWIN_DIR}/crt/include"
    "/imsvc ${XWIN_DIR}/sdk/include/ucrt"
    "/imsvc ${XWIN_DIR}/sdk/include/um"
    "/imsvc ${XWIN_DIR}/sdk/include/shared"
    "/imsvc ${XWIN_DIR}/sdk/include/winrt")
string(REPLACE ";" " " _XWIN_COMPILE_FLAGS "${_XWIN_COMPILE_FLAGS}")

set(CMAKE_C_FLAGS_INIT "${_XWIN_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${_XWIN_COMPILE_FLAGS} /EHsc")

set(_XWIN_LINK_FLAGS
    /manifest:no
    "-libpath:${XWIN_DIR}/crt/lib/${XWIN_ARCH}"
    "-libpath:${XWIN_DIR}/sdk/lib/um/${XWIN_ARCH}"
    "-libpath:${XWIN_DIR}/sdk/lib/ucrt/${XWIN_ARCH}")
string(REPLACE ";" " " _XWIN_LINK_FLAGS "${_XWIN_LINK_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${_XWIN_LINK_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_XWIN_LINK_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_XWIN_LINK_FLAGS}")

# Match cargo-xwin's generated clang-cl CMake toolchain behavior.
set(CMAKE_C_STANDARD_LIBRARIES "" CACHE STRING "" FORCE)
set(CMAKE_CXX_STANDARD_LIBRARIES "" CACHE STRING "" FORCE)
set(CMAKE_TRY_COMPILE_CONFIGURATION Release)
set(CMAKE_USER_MAKE_RULES_OVERRIDE "/opt/rayneo-toolchain/override.cmake")
