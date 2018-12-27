@echo off
title UNWDS FLASHER
color 0A
set /p com="[OTA] COM port: "
:start
	ota.py ota-image.bin COM%com%
	pause
	cls
goto start
