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
import struct

from lidar_visualizer.datasets import supported_file_extensions


class HeLiPRDataset:
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
            self.format_string = "fffBBBL"
            self.index_intensity = None
        elif self.sequence_id == "Aeva":
            self.format_string = "ffffflBf"
            self.format_string_no_intensity = "ffffflB"
            self.index_intensity = 7
        elif self.sequence_id == "Ouster":
            self.format_string = "ffffIHHH"
            self.index_intensity = 3
        elif self.sequence_id == "Velodyne":
            self.format_string = "ffffHf"
            self.index_intensity = 3
        else:
            print("[ERROR] Unsupported LiDAR Type")
            sys.exit()

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(idx)

    def get_data(self, idx: int):
        file_path = self.scan_files[idx]
        list_lines = []

        # Special case, see https://github.com/minwoo0611/HeLiPR-File-Player/blob/e8d95e390454ece1415ae9deb51515f63730c10a/src/ROSThread.cpp#L632
        if self.sequence_id == "Aeva" and int(Path(file_path).stem) <= 1691936557946849179:
            self.index_intensity = None
            format_string = self.format_string_no_intensity
        else:
            format_string = self.format_string

        chunk_size = struct.calcsize(f"={format_string}")
        with open(file_path, "rb") as f:
            binary = f.read()
            offset = 0
            while offset < len(binary):
                list_lines.append(struct.unpack_from(f"={format_string}", binary, offset))
                offset += chunk_size
        data = np.stack(list_lines)
        return data

    def read_point_cloud(self, idx: int):
        data = self.get_data(idx)
        points = data[:, :3]
        scan = self.o3d.geometry.PointCloud()
        scan.points = self.o3d.utility.Vector3dVector(points)
        if self.index_intensity is not None:
            intensity = data[:, self.index_intensity]
            intensity = (intensity - intensity.min()) / (intensity.max() - intensity.min())
            colors = self.cmap(intensity)[:, :3].reshape(-1, 3)
            scan.colors = self.o3d.utility.Vector3dVector(colors)
        return scan
