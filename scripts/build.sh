export F4PGA_INSTALL_DIR=/opt/f4pga
export FPGA_FAM=eos-s3
source "$F4PGA_INSTALL_DIR/$FPGA_FAM/conda/etc/profile.d/conda.sh"
conda activate eos-s3
make -C app/fpga top.h

cp app/fpga/build/top_bit.h app/embedded/src/top_bit.h
make -C app/embedded