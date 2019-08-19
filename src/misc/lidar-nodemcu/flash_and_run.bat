@echo off

SET COM_PORT=%1
SET BAUD_RATE=115200
SET FILES_TO_UPLOAD=("main.py", "sweep.py", "lidarlite.py")

echo Kill putty
taskkill /im putty.exe /f > NUL
timeout 1 > NUL
for %%i in %FILES_TO_UPLOAD% do (
    echo Upload script %%i
    ampy -p %COM_PORT% -b %BAUD_RATE% put %%i
)

echo Reset board
ampy -p %COM_PORT% -b %BAUD_RATE% reset

echo Start "main.py"
start ampy -p %COM_PORT% -b %BAUD_RATE% run "main.py"
echo Wait a moment for script to start

timeout 2 > NUL
taskkill /im ampy.exe /f > NUL

echo Running putty
start putty.exe -serial %COM_PORT% -sercfg %BAUD_RATE%,8,n,1,N