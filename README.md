# LiDAR Visualizer 🚀

A flexible, easy-to-use, LiDAR (or any point cloud) visualizer for Linux, Windows, and macOS.

![out](https://user-images.githubusercontent.com/21349875/234777083-eeb4ec57-cb50-4c69-babd-4cc8e63cff86.png)

If you also need to obtain poses from your dataset, consider checking out [KISS-ICP](https://github.com/PRBonn/kiss-icp).

## Install (\*)

```sh
pip install lidar-visualizer
```

(\*) This package relies on the power of [Open3D](https://www.open3d.org) but does not list it as a dependency. If you haven't installed `open3d` then `pip install open3d` or check [the official instructions](https://www.open3d.org/docs/release/getting_started.html)

## Optional dependencies

Depending on the [dataloaders](./src/lidar_visualizer/datasets/) you plan to use you might need to install optional dependencies. The tool will prompt which tools is the one you are requesting and is not accessible, but if you want to go for brute force and install all of it just run:

```sh
pip install lidar-visualizer[all]
```

## Usage

```sh
lidar_visualizer --help
```

## Citation

If you use this visualizer for any academic work, please cite our original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf).

```bibtex
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1029--1036},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023},
  codeurl   = {https://github.com/PRBonn/kiss-icp},
}
```
