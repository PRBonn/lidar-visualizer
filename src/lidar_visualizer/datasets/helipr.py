# MIT License
#
# Copyright (c) 2024 Saurabh Gupta, Ignacio Vizzo, Tiziano Guadagnino,
# Benedikt Mersch, Cyrill Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import importlib
import os
import sys
from pathlib import Path

import natsort
import numpy as np

from lidar_visualizer.datasets import supported_file_extensions


class HeLiPRLivoxDataset:
    def __init__(self, data_dir: Path, *_, **__):
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as e:
            raise ModuleNotFoundError(
                "Open3D is not installed on your system, to fix this either "
                'run "pip install open3d" '
                "or check https://www.open3d.org/docs/release/getting_started.html"
            ) from e

        self.sequence_id = os.path.basename(data_dir).lower()
        self.scans_dir = os.path.join(os.path.realpath(data_dir), "")
        self.scan_files = np.array(
            natsort.natsorted(
                [
                    os.path.join(self.scans_dir, fn)
                    for fn in os.listdir(self.scans_dir)
                    if any(fn.endswith(ext) for ext in supported_file_extensions())
                ]
            ),
            dtype=str,
        )
        if len(self.scan_files) == 0:
            raise ValueError(f"Tried to read point cloud files in {self.scans_dir} but none found")
        self.file_extension = self.scan_files[0].split(".")[-1]
        if self.file_extension not in supported_file_extensions():
            raise ValueError(f"Supported formats are: {supported_file_extensions()}")

        self.intensity_channel = None
        # Obtain the pointcloud reader for the given data folder
        if self.sequence_id.find("avia"):
            self.dtype = np.dtype(
                [
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("reflectivity", np.uint8),
                    ("tag", np.uint8),
                    ("line", np.uint8),
                    ("offset_time", np.uint32),
                ]
            )

        elif self.sequence_id.find("aeva"):
            self.dtype = np.dtype(
                [
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("reflectivity", np.float32),
                    ("velocity", np.float32),
                    ("time_offset_ns", np.int32),
                    ("line_index", np.uint8),
                    ("intensity", np.float32),
                ]
            )
            self.intensity_channel = 7

        elif self.sequence_id.find("ouster"):
            self.dtype = np.dtype(
                [
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("intensity", np.float32),
                    ("t", np.int32),
                    ("reflectivity", np.float16),
                    ("ring", np.float16),
                    ("ambient", np.float16),
                ]
            )
            self.intensity_channel = 3

        elif self.sequence_id.find("velodyne"):
            self.dtype = np.dtype(
                [
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("intensity", np.float32),
                    ("ring", np.float16),
                    ("time", np.float32),
                ]
            )
            self.intensity_channel = 3

        else:
            print("[ERROR] Unsupported LiDAR Type")
            sys.exit()

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def read_point_cloud(self, file_path: str):
        scan = self.o3d.geometry.PointCloud()
        if self.intensity_channel is not None:
            points_xyzi = np.stack(
                [
                    [line[0], line[1], line[2], line[self.intensity_channel]]
                    for line in np.fromfile(file_path, dtype=self.dtype).tolist()
                ]
            )
            points = points_xyzi[:, 0:3]
            intensity = points_xyzi[:, -1]
            intensity = intensity / intensity.max()
            colors = self.cmap(intensity)[:, :3].reshape(-1, 3)
            scan.points = self.o3d.utility.Vector3dVector(points)
            scan.colors = self.o3d.utility.Vector3dVector(colors)

        else:
            points = np.stack(
                [
                    [line[0], line[1], line[2]]
                    for line in np.fromfile(file_path, dtype=self.dtype).tolist()
                ]
            )
            scan.points = self.o3d.utility.Vector3dVector(points)

        return scan
