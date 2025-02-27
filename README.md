# mesh_sampling

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI of mesh_sampling](https://github.com/arntanguy/mesh_sampling/workflows/CI%20of%20mesh_sampling/badge.svg)](https://github.com/arntanguy/mesh_sampling/actions?query=workflow%3A%22CI+of+mesh_sampling%22)
[![Package mesh_sampling](https://github.com/arntanguy/mesh_sampling/workflows/Package%20mesh_sampling/badge.svg)](https://github.com/arntanguy/mesh_sampling/actions?query=workflow%3A%22Package%20mesh_sampling%22)

C++ Implementation of pointcloud generation from mesh sampling methods.

![Sampling example](https://raw.githubusercontent.com/arntanguy/mesh_sampling/master/sample/sampling_example.png)

So far, the following samplers have been implemented:

- Weighted random sampling: generates a given number of points uniformely distributed according to triangle areas.
  See [this blog post](https://medium.com/@daviddelaiglesiacastro/3f-point-cloud-generation-from-3f-triangular-mesh-bbb602ecf238) for details on the method.

It is provided as-is, and could probably be optimized should the need arise. Feel free to submit merge requests.

## Installation

### From Ubunu packages (18.04, 20.04, 22.04)

```sh
# Setup the mirror
curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | sudo -E bash
# Install packages
sudo apt install libmesh-sampling-dev
```

### From source

Requirements:
- cmake >3.11
- Eigen3
- PCL 1.7
- Boost (program_options and filesystem)

If you do not already have a recent cmake installation (>3.11), you will need to install it. On Ubuntu bionic, this can be done by adding the official [Kitware PPA](https://apt.kitware.com/), and updating cmake

For bionic:

```sh
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
```

For xenial:

```sh
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial-rc main'
sudo apt-get update
```

Then install cmake
```sh
sudo apt install cmake
```


You can now build and install this package

```
git clone --recursive https://github.com/arntanguy/mesh_sampling.git
cd mesh_sampling
mkdir build && cd build
cmake ..
make
sudo make install
```

## Usage

### Command-line tool

A simple binary executable `mesh_sampling` is provided. It'll convert any model supported by ASSIMP into its corresponding pointcloud with a given number of points. The command line is of the general form: 

```
mesh_sampling /path/to/model.<supported_mesh_format> /path/to/cloud/cloud.<supported_cloud_format> --type xyz_rgb_normal --samples 10000 --binary
```

Where:
- `supported_mesh_format` is one of the mesh format supported by `ASSIMP` (commonly DAE, STL, OBJ)
- `supported_cloud_format` is a PCL formal (`pcd` or `ply`), or qhull's format (`qc`)

See `mesh_sampling --help` for more options.

Example:

```bash
mesh_sampling /path/to/model.dae /tmp/cloud.pcd --type xyz_rgb_normal --samples 10000 --binary
pcl_viewer /tmp/cloud.pcd -normals_scale 5 -normals 1
```

### Generating convex files using qhull

To generate convex files, you will first need to convex your mesh to qhull's `.qc` format, then use `qconvex` to generate the convex hull.

```
mesh_sampling /path/to/model.<supported_mesh_format> /tmp/test.qc --type xyz --samples 10000
qconvex TI /tmp/test.qc TO /tmp/test-ch.txt Qt o f
```
