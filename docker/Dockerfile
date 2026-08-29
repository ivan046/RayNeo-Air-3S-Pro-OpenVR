FROM messense/cargo-xwin:0.22.0

RUN apt-get update \
    && apt-get install --no-install-recommends -y p7zip-full \
    && rm -rf /var/lib/apt/lists/*

ENV XWIN_CACHE_DIR=/opt/cargo-xwin-cache \
    XWIN_ARCH=x86_64 \
    XWIN_VARIANT=desktop \
    XWIN_VERSION=17 \
    XWIN_SDK_VERSION=10.0.26100 \
    XWIN_CRT_VERSION=14.44.17.14

RUN cargo xwin cache xwin \
    && cargo xwin env --target x86_64-pc-windows-msvc > /opt/rayneo-xwin-env.sh

ARG LIBUSB_VERSION=1.0.30
ARG LIBUSB_SHA256=7fb1dfec805b97983763d7d0ae244320da12add1003d4249c96cc4d586398c79
RUN curl -fL \
      "https://github.com/libusb/libusb/releases/download/v${LIBUSB_VERSION}/libusb-${LIBUSB_VERSION}.7z" \
      -o /tmp/libusb.7z \
    && echo "${LIBUSB_SHA256}  /tmp/libusb.7z" | sha256sum -c - \
    && mkdir -p /opt/libusb \
    && 7z x -y /tmp/libusb.7z -o/opt/libusb >/dev/null \
    && rm -f /tmp/libusb.7z
