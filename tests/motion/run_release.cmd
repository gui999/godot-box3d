@echo off
rem Builds the motion core harness optimised (RelWithDebInfo, no Box3D validation) and runs it. Build dir: %TEMP%\b3motion_rel
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul
set PATH=C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja;%PATH%
set OUT=%TEMP%\b3motion_rel
cd /d %~dp0
if not exist %OUT%\build.ninja cmake -S . -B %OUT% -G Ninja -DCMAKE_BUILD_TYPE=Release -DB3M_VALIDATE=OFF || exit /b 1
cmake --build %OUT% --target motion_test 2>&1 || exit /b 1
%OUT%\motion_test.exe %*
exit /b %ERRORLEVEL%
