@echo off
title UNWDS FLASHER
color 0A
:start
	backdoor-bootloader.py -r -l 8 -a 0x500012F0 -b 115200 -p COM3 ieee_adr.bin
	generate-license.exe ieee_adr.bin
	rm ieee_adr.bin
	srec_cat.exe root-firmware.hex -intel license.bin -binary -offset 0x1D000 eeprom.bin -binary -offset 0x1E000 -o root_firmware.hex -intel
	rm license.bin
	pause
	backdoor-bootloader.py -e -w -b 115200 -p COM3 -v root_firmware.hex
	rm root_firmware.hex
	pause
	cls
goto start
