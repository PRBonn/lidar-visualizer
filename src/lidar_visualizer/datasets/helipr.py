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
        # Intensity stuff
        import matplotlib.cm as cm

        self.cmap = cm.viridis

        self.sequence_id = os.path.basename(data_dir)
        self.scan_files = np.array(
            natsort.natsorted(
                [
                    os.path.join(data_dir, fn)
                    for fn in os.listdir(data_dir)
                    if any(fn.endswith(ext) for ext in supported_file_extensions())
                ]
            ),
            dtype=str,
        )
        if len(self.scan_files) == 0:
            raise ValueError(f"Tried to read point cloud files in {data_dir} but none found")
        self.file_extension = self.scan_files[0].split(".")[-1]
        if self.file_extension not in supported_file_extensions():
            raise ValueError(f"Supported formats are: {supported_file_extensions()}")

        # Obtain the pointcloud reader for the given data folder
        if self.sequence_id == "Avia":
            self.fields = [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("reflectivity", np.uint8),
                ("tag", np.uint8),
                ("line", np.uint8),
                ("offset_time", np.uint32),
            ]

        elif self.sequence_id == "Aeva":
            self.fields = [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("reflectivity", np.float32),
                ("velocity", np.float32),
                ("time_offset_ns", np.int32),
                ("line_index", np.uint8),
                ("intensity", np.float32),
            ]

        elif self.sequence_id == "Ouster":
            self.fields = [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("intensity", np.float32),
                ("t", np.uint32),
                ("reflectivity", np.uint16),
                ("ring", np.uint16),
                ("ambient", np.uint16),
            ]

        elif self.sequence_id == "Velodyne":
            self.fields = [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("intensity", np.float32),
                ("ring", np.uint16),
                ("time", np.float32),
            ]

        else:
            print("[ERROR] Unsupported LiDAR Type")
            sys.exit()

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(idx)

    def get_intensity_channel(self):
        matches = [index for index, (name, _) in enumerate(self.fields) if name == "intensity"]
        return matches[0] if len(matches) > 0 else None

    def read_point_cloud(self, idx: int):
        file_path = self.scan_files[idx]

        intensity_channel = self.get_intensity_channel()
        dtype = np.dtype(self.fields)

        # Special case, see https://github.com/minwoo0611/HeLiPR-File-Player/blob/e8d95e390454ece1415ae9deb51515f63730c10a/src/ROSThread.cpp#L632
        if self.sequence_id == "Aeva" and int(Path(file_path).stem) <= 1691936557946849179:
            intensity_channel = None
            dtype = np.dtype(
                [(name, np_type) for name, np_type in self.fields if name != "intensity"]
            )

        scan = self.o3d.geometry.PointCloud()
        if intensity_channel is not None:
            points_xyzi = np.stack(
                [
                    [line[0], line[1], line[2], line[intensity_channel]]
                    for line in np.fromfile(file_path, dtype=dtype).tolist()
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
                    for line in np.fromfile(file_path, dtype=dtype).tolist()
                ]
            )
            scan.points = self.o3d.utility.Vector3dVector(points)

        return scan
