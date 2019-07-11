@echo off

SET COM_PORT=%1
SET BAUD_RATE=115200
SET FILE_NAME=%2

echo Kill putty
taskkill /im putty.exe /f > NUL
timeout 1 > NUL

echo Upload script "%FILE_NAME%"
ampy -p %COM_PORT% -b %BAUD_RATE% put main.py

echo Reset board
ampy -p %COM_PORT% -b %BAUD_RATE% reset

echo Start script "%FILE_NAME%"
start ampy -p %COM_PORT% -b %BAUD_RATE% run %FILE_NAME%
echo Wait a moment for script to start

timeout 2 > NUL
taskkill /im ampy.exe /f > NUL

echo Running putty
start putty.exe -serial %COM_PORT% -sercfg %BAUD_RATE%,8,n,1,N