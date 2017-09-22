
# Building
```
mkdir build && cd build
cmake ../
make
```

# Building on Windows
```
mkdir build && cd build
cmake-gui ..
```

Configure for VS 2015/17 within Cmake-gui

Generate

In the build folder (in file explorer), open the newly generated Visual Studio solution

# Running Headless

```
sudo apt-get install xserver-xorg-video-dummy x11-apps

sudo Xorg -noreset +extension GLX +extension RANDR + extension RENDER -logfile xdummy.log -config utils/xorg.conf :1 &

DISPLAY=:1 path/to/executable
```
