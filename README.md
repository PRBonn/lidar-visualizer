# LiDAR Visualizer

A flexible, easy to use, LiDAR (or any point cloud) visualizer for Linux, Windows and macOS.

If you also need to obtain poses from your dataset, considering checking out [KISS-ICP](https://github.com/PRBonn/kiss-icp), a LiDAR Odometry pipeline that just works. This simple visualizer is nothing but a wrapper around all the machinery built for KISS-ICP.

## Install

```sh
pip install lidar-visualizer
```

Next, follow the instructions on how to run the system by typing:

```sh
lidar_visualizer --help
```

This should print the following help message:
![out]()


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
