@echo off
title UNWDS FLASHER
color 0A
set /p com="COM port: "
:start
	generate-eeprom.exe 0xAABB 26 11223344556677889900AABBCCDDEEFF
	srec_cat.exe lit-firmware-cc13xx.hex -intel eeprom.bin -binary -offset 0x1E000 -o lit_firmware_cc13xx.hex -intel
	rm eeprom.bin
	backdoor-bootloader.py -e -w -b 115200 -p COM%com% -v lit_firmware_cc13xx.hex
	rm lit_firmware_cc13xx.hex
	pause
	cls
goto start
