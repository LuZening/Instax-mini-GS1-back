@echo off
set FILENAME=instax_back
sdas8051 -gloaxsff %FILENAME%.asm
sdld -f %FILENAME%.link
packihx %FILENAME%.ihx > %FILENAME%.hex
del /Q/F %FILENAME%.ihx
del /Q/F %FILENAME%.rel
del /Q/F %FILENAME%.map
del /Q/F %FILENAME%.rst
del /Q/F %FILENAME%.sym
del /Q/F %FILENAME%.lst
cat %FILENAME%.mem
del /Q/F %FILENAME%.mem
pause
@echo on
