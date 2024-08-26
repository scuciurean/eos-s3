#!/bin/bash

function setup_f4pga_environement {
  export F4PGA_INSTALL_DIR=/opt/f4pga
  export FPGA_FAM=eos-s3
  export F4PGA_TIMESTAMP='20220920-124259'
  export F4PGA_HASH='007d1c1'
  export F4PGA_PACKAGES='install-ql ql-eos-s3_wlcsp'

  wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O conda_installer.sh
  bash conda_installer.sh -u -b -p $F4PGA_INSTALL_DIR/$FPGA_FAM/conda;
  wget -qO- https://storage.googleapis.com/symbiflow-arch-defs-install/quicklogic-arch-defs-d6d05185.tar.gz | tar -xzC $F4PGA_INSTALL_DIR/eos-s3/
  source "$F4PGA_INSTALL_DIR/$FPGA_FAM/conda/etc/profile.d/conda.sh";
  rm conda_installer.sh

  for PKG in $F4PGA_PACKAGES; do
    wget -qO- https://storage.googleapis.com/symbiflow-arch-defs/artifacts/prod/foss-fpga-tools/symbiflow-arch-defs/continuous/install/${F4PGA_TIMESTAMP}/symbiflow-arch-defs-${PKG}-${F4PGA_HASH}.tar.xz | tar -xJC $F4PGA_INSTALL_DIR/${FPGA_FAM}
  done

  conda env create -f scripts/eos_s3_env.yml
}

function setup_arm_gcc {
  apt-get install -y gcc-arm-none-eabi
}

$1