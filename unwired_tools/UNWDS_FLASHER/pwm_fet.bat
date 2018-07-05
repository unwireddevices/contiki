@echo off
title UNWDS FLASHER
color 0A
set /p com="COM port: "
:start
	backdoor-bootloader.py -r -l 8 -a 0x500012F0 -b 115200 -p COM%com% ieee_adr.bin
	generate-license.exe ieee_adr.bin
	rm ieee_adr.bin
	generate-eeprom.exe 0xAABB 26 11223344556677889900AABBCCDDEEFF CAN 0
	srec_cat.exe pwm_fet-firmware.hex -intel license.bin -binary -offset 0x1D000 eeprom.bin -binary -offset 0x1E000 -o pwm_fet_firmware.hex -intel
	rm license.bin
	rm eeprom.bin
	pause
	backdoor-bootloader.py -e -w -b 115200 -p COM%com% -v pwm_fet_firmware.hex
	rm pwm_fet_firmware.hex
	pause
	cls
goto start
