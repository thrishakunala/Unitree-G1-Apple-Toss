@echo off
setlocal
cd /d "%~dp0"
set PY=python
where python >nul 2>nul
if errorlevel 1 set PY=py
%PY% -c "import mujoco, numpy, matplotlib, imageio, PIL" >nul 2>nul
if errorlevel 1 (
  echo Installing required Python packages. This happens once.
  %PY% -m pip install mujoco numpy matplotlib imageio imageio-ffmpeg pillow
)
%PY% apple_toss.py %*
if errorlevel 1 (
  echo.
  echo Something went wrong. If Python is not installed, get it from python.org and run this again.
)
pause
