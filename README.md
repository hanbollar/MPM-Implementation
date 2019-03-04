# Project Report
#### Read Project [Report](https://github.com/hanbollar/MPM-Implementation/blob/master/ProjectReport.pdf)

#### Watch Final [Demo](https://www.vimeo.com/hannahbollar/mpm)

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

# Division of Responsibilities

Austin - Implemented the initial background grid and was responsible for heavy project
framework and the main infrastructure. Also involved in debugging and optimizations for CPU
overall speed for each iteration.

Joe - Involved in the implementation of snow elastic and plastic calculations, grid collision
checks, and final base renders.

Hannah - Implemented the particle to grid and grid to particle math-based transfers and also
involved in the math understanding for cauchy-stress calculations for the MPM implementation
and other math aspects of the project.

Youssef - Implemented Affine Particle In Cell (APIC) method to allow for angular momentum
conservation in the MPM Simulation visuals. Also added particle exporting into .geo format and
rendering using Houdini.
