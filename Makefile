PIO ?= pio
MONITOR_SPEED = 115200

all:
	$(PIO) run -e uno_r4_wifi

upload:
	$(PIO) run -e uno_r4_wifi -t upload

monitor:
	$(PIO) device monitor -b $(MONITOR_SPEED)

test:
	$(PIO) test -e native

# One invocation per environment, so a failure names the environment it came from.
test-all:
	$(PIO) test -e native
	$(PIO) test -e native_bench
	$(PIO) test -e native_cal
	$(PIO) test -e native_measured
	$(PIO) test -e native_nosimcli
	$(PIO) test -e native_live

test-device:
	@echo "DEVICE TESTS - the board must be on USB; this uploads and runs test_device on it"
	$(PIO) test -e uno_r4_wifi_test

# Everything tools/check.sh reads out of .pio/build: the three binaries, plus the device
# test environment compiled without a board, for its own lib/Network check.
build-all:
	$(PIO) run -e uno_r4_wifi -e uno_r4_wifi_bringup -e uno_r4_wifi_sim
	$(PIO) test -e uno_r4_wifi_test -f test_device --without-uploading --without-testing

bringup:
	@echo "BRING-UP BUILD - pump/cal/servo/home/goto/hang are compiled in. This is NOT the binary left running."
	$(PIO) run -e uno_r4_wifi_bringup -t upload

sim:
	@echo "SIM BUILD - the 12 V brick must be unplugged"
	$(PIO) run -e uno_r4_wifi_sim -t upload
	cp .pio/build/uno_r4_wifi_sim/firmware.bin firmware-SIM.bin

calib:
	@echo "BRING-UP 7b: upload the bringup binary, then type calib in the monitor"
	$(PIO) run -e uno_r4_wifi_bringup -t upload
	$(PIO) device monitor -b $(MONITOR_SPEED)

check:
	./tools/check.sh

clean:
	$(PIO) run -t clean

compiledb:
	$(PIO) run -t compiledb

.PHONY: all upload monitor test test-all test-device build-all bringup sim calib check clean compiledb
