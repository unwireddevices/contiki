@echo off
title UNWDS FLASHER
color 0A
set /p com="COM port: "
:start
	generate-eeprom.exe 0xAABB 26 11223344556677889900AABBCCDDEEFF
	srec_cat.exe test_uart-firmware-cc26xx.hex -intel eeprom.bin -binary -offset 0x1E000 -o test_uart_firmware_cc26xx.hex -intel
	rm eeprom.bin
	backdoor-bootloader.py -e -w -b 115200 -p COM%com% -v test_uart_firmware_cc26xx.hex
	rm test_uart_firmware_cc26xx.hex
	pause
	cls
goto start
