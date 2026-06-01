ENV ?= esp32doit-devkit-v1

build:
	@pio run -e $(ENV)

upload:
	@pio run -e $(ENV) --target upload --disable-auto-clean

monitor:
	@pio device monitor -e $(ENV)

clean:
	@pio run -e $(ENV) --target clean

flash:
	@pio run -e $(ENV) --target upload --disable-auto-clean
	@pio device monitor -e $(ENV)

.PHONY: build upload monitor clean flash