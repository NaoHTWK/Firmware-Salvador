FROM ubuntu:22.04

LABEL maintainer="HTWK Robots Team"
LABEL description="Simulation build environment"
LABEL version="6.0"
LABEL target="simulation"

SHELL ["/bin/bash", "-c"]

RUN apt-get clean autoclean
RUN apt-get autoremove --yes
RUN apt-get update
RUN apt-get install -y cmake g++ git libusb-1.0-0-dev libudev-dev pkg-config wget

RUN mkdir -p /l4t/targetfs/
RUN apt-get update
RUN apt-get download libusb-1.0-0.dev libudev-dev
RUN for deb in *.deb; do dpkg -x $deb /l4t/targetfs/; done
RUN rm *.deb
RUN rm /l4t/targetfs/usr/lib/x86_64-linux-gnu/libusb-1.0.a
