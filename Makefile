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

.PHONY: all upload monitor test bringup sim calib check clean compiledb
