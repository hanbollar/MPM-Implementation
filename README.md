
# Building
```
mkdir build && cd build
cmake ../
make
```

# Running Headless

```
sudo apt-get install xserver-xorg-video-dummy x11-apps

sudo Xorg -noreset +extension GLX +extension RANDR + extension RENDER -logfile xdummy.log -config utils/xorg.conf :1 &

DISPLAY=:1 path/to/executable
```
