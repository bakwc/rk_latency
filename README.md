# rk_latency

Example of low latency frame by frame decoding using rockchip mpp library https://github.com/rockchip-linux/mpp
For rk3399 it gives 6-8 ms. latency with 1920x1080 100kb h264 frames.

## Run steps
```
git clone https://github.com/bakwc/rk_latency
cd rk_latency
mkdir build
cd build
cmake ..
make
cd ../frames/
../build/rk_latency
```
