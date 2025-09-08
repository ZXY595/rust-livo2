# rust-livo2
> [!WARNING]
>
> Don't use this library for now, it's still in deep development.

# Overview
> [!WARNING]
>
> (WIP) Rust implement of FAST_LIVO2.

# Features
## 1. Eazy to use
The library doesn't have any external dependencies, like Ros, Eigen or OpenCV.
It's easy to use as a Rust library.
All you need to do is to add the library to your Cargo.toml file.
## 2. Simple and readeable code
Modular and well designed. The code is easy to read and understand how FAST-LIVO2 works.
## 3. Faster and more accurate than C++ implementation
The original C++ implementation is lack of optimizations, and differs significantly from some of the related formulas in the original paper. 

# Current Status
- [x] Basic error propagate (jocobian) framework 
- [x] Point error propagate
- [x] Plane error propagate
- [ ] Voxel map and octree implement
- [ ] imu implement
- [ ] esikf state estimator implement
- [ ] vio implement
- [ ] split modules into different crates
- [ ] ...

# Get Started
## Clone the repository
```sh
git clone https://github.com/ZXY595/rust-livo2.git
cd rust-livo2

```
## Run the example
> [!TODO]
> todo

# Licence
Licensed under the GPLv2 license.
