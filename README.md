This is forked from coyote009.

# calib_intrinsic
Tools to calibrate for intrinsic parameters of raspberry pi camera

## Description
This project is made up of two parts: calib_chart_generator to generate calibration chart image, and calib_intrinsic to calibrate for parameters.

## Prerequisites
You need OpenCV 3.4.X with contrib module installed on your computer.

## Build instruction
### calib_chart_generator
If windows,
```
Open calib_chart_generator/build_vc12/calib_chart_generator.sln in Visual Studio 2013
```
If linux,
```
> cd calib_chart_generator
> mkdir build
> cd build
> cmake ..
> make
```

### calib_intrinsic
```
> cd calib_intrinsic
> mkdir build
> cd build
> cmake ..
> make
```
