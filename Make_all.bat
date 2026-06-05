del /Q OUTPUT\*.bin

REM Прошивка с настройками PacAT
xcopy /Y configs\PacAT\*.h  sources\
C:/avr-gcc/bin/make.exe -f MakefileWin %1

del /Q OUTPUT\*.o
del /Q OUTPUT\*.elf
move output\A650E_Transmission_Controller.bin output\A650E_Transmission_Controller_PacAt.bin
 
REM Прошивка с настройками Kosh
xcopy /Y configs\Kosh\*.h  sources\
C:/avr-gcc/bin/make.exe -f MakefileWin %1

del /Q OUTPUT\*.o
del /Q OUTPUT\*.elf
move output\A650E_Transmission_Controller.bin output\A650E_Transmission_Controller_Kosh.bin
