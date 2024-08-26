FROM ubuntu:20.04

ENV HOME=/sdk-setup

RUN apt-get update && apt-get install -y \
    curl wget xz-utils \
    make \
    gcc-arm-none-eabi binutils-arm-none-eabi

WORKDIR $HOME

COPY . $HOME

RUN bash $HOME/scripts/install_tools.sh setup_f4pga_environement