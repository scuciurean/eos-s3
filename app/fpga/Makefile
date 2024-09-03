SRCS = \
	rtl/ip/wishbone/wb_interconnect.v \
	rtl/ip/wishbone/wb_gpio.v \
	rtl/ip/wishbone/wb_pwm.v \
	rtl/ip/ad7984/ad7984.v \
	rtl/sys/sdma.v \
	rtl/top.v

PCF = rtl/pinout.pcf
SDC = rtl/timing.sdc

QL_FLAGS= \
	-d ql-eos-s3 \
	-P PU64 \
	-t top \
	-v $(SRCS) \
	-p $(PCF) \
	-s $(SDC)

all:
	ql_symbiflow -compile $(QL_FLAGS) -dump binary -dump header -dump jlink -dump header

top.h: all