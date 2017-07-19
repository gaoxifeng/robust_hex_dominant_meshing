[![Build status](https://ci.appveyor.com/api/projects/status/vvpv7t7oobr47i8g?svg=true)](https://ci.appveyor.com/project/gaoxifeng/robust-hex-dominant-meshing)

# Robust Quad/Hex-dominant Meshes

This repository contains the meshing software developed as part of the publication

Robust Hex-Dominant Mesh Generation using Field-Guided Polyhedral Agglomeration 

Xifeng Gao, Wenzel Jakob, Marco Tarini, Daniele Panozzo

In ACM Transactions on Graphics (Proceedings of SIGGRAPH 2017)

> **paper|supplemental materials|dataset|videos:** https://gaoxifeng.github.io/research.html

# Compiling

Compiling from scratch requires CMake and a recent version of Clion on Mac and Visual Studio 2015 on Windows.

git clone --recursive https://github.com/gaoxifeng/robust_hex_dominant_meshing.git

On MacOS, compiling should be as simple as Load the project "robust_hex_dominant_meshing" into CLion and Build from Clion

On Windows, open the generated file robust_hex_dominant_meshing.sln after CMake compilation and proceed building as usual from within Visual Studio.

# Usage

Step 1: launch the program and click "Open mesh" to select a mesh in .obj format;

Step 2: click either the "Surface" or "Volume" option to start the Quad-dominant or Hex-dominant meshing;

Step 3: click "Rosy" to optimize the orientation field;

Step 4: click "Posy" to optimize the position field;

Step 5: click "Extract" to obtain the hybrid mesh.
