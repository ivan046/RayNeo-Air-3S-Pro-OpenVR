FROM messense/cargo-xwin:0.22.0

USER root

RUN apt-get update \
    && apt-get install --no-install-recommends -y p7zip-full \
    && rm -rf /var/lib/apt/lists/*

ENV XWIN_CACHE_DIR=/opt/cargo-xwin-cache \
    XWIN_CROSS_COMPILER=clang-cl \
    XWIN_ARCH=x86_64 \
    XWIN_VARIANT=desktop \
    XWIN_VERSION=17 \
    XWIN_SDK_VERSION=10.0.26100 \
    XWIN_CRT_VERSION=14.44.17.14

RUN cargo xwin cache xwin

# cargo-xwin's clang-cl backend normalizes these LLVM tool names the same way.
RUN ln -sf "$(command -v clang)" /usr/local/bin/clang-cl \
    && ln -sf "$(dirname "$(rustc --print target-libdir)")/bin/rust-lld" /usr/local/bin/lld-link \
    && ln -sf "$(command -v llvm-ar)" /usr/local/bin/llvm-lib

ARG LIBUSB_VERSION=1.0.30
ARG LIBUSB_SHA256=7fb1dfec805b97983763d7d0ae244320da12add1003d4249c96cc4d586398c79
RUN curl -fL \
      "https://github.com/libusb/libusb/releases/download/v${LIBUSB_VERSION}/libusb-${LIBUSB_VERSION}.7z" \
      -o /tmp/libusb.7z \
    && echo "${LIBUSB_SHA256}  /tmp/libusb.7z" | sha256sum -c - \
    && mkdir -p /opt/libusb \
    && 7z x -y /tmp/libusb.7z -o/opt/libusb >/dev/null \
    && mkdir -p /opt/libusb-vcpkg/x64-windows/bin \
    && ln -sf /opt/libusb/VS2022/MS64/dll/libusb-1.0.dll \
        /opt/libusb-vcpkg/x64-windows/bin/libusb-1.0.dll \
    && rm -f /tmp/libusb.7z

COPY xwin-clang-cl.cmake /opt/rayneo-toolchain/xwin-clang-cl.cmake
COPY override.cmake /opt/rayneo-toolchain/override.cmake

WORKDIR /work
