all: firmware eeprom.bin eeprom.txt

# These require the pinoccio-eeprom-tool:
# https://github.com/Pinoccio/pinoccio-backpack-reference/tree/master/tools
eeprom.bin: eeprom.yaml
	pinoccio-eeprom-tool $< -o $@

eeprom.txt: eeprom.yaml
	pinoccio-eeprom-tool --bitlash $< > $@ || (rm -f $@ && false)

.PHONY: firmware
firmware:
	$(MAKE) -C firmware
