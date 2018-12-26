@echo off
title UNWDS FLASHER
color 0A
set /p com="COM port: "
:start
	ota.py 
	pause
	cls
goto start
REM ota.py ota-image.bin COM%com%
