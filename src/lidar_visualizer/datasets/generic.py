# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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


class GenericDataset:
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

        # Config stuff
        self.sequence_id = os.path.basename(data_dir)
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

        # Obtain the pointcloud reader for the given data folder
        self._read_point_cloud = self._get_point_cloud_reader()

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self._read_point_cloud(self.scan_files[idx])

    def _get_point_cloud_reader(self):
        """Attempt to guess with try/catch blocks which is the best point cloud reader to use for
        the given dataset folder. Supported readers so far are:
            - np.fromfile
            - open3d
            - trimesh.load
            - PyntCloud
        """
        # This is easy, the old KITTI format
        if self.file_extension == "bin":
            print("[WARNING] Reading .bin files, the only format supported is the KITTI format")

            def read_kitti_scan(file):
                points_xyzi = (
                    np.fromfile(file, dtype=np.float32).reshape((-1, 4)).astype(np.float64)
                )
                points = points_xyzi[:, 0:3]
                intensity = points_xyzi[:, -1]
                scan = self.o3d.geometry.PointCloud()
                intensity = intensity / intensity.max()
                colors = self.cmap(intensity)[:, :3].reshape(-1, 3)
                scan.points = self.o3d.utility.Vector3dVector(points)
                scan.colors = self.o3d.utility.Vector3dVector(colors)
                return scan

            return read_kitti_scan

        first_scan_file = self.scan_files[0]
        tried_libraries = []
        missing_libraries = []
        # First with open3d
        try:
            self.o3d.t.io.read_point_cloud(first_scan_file)

            def read_scan_with_intensities(file):
                scan = self.o3d.t.io.read_point_cloud(file)
                intensity = scan.point.intensity.numpy()
                intensity = intensity / intensity.max()
                scan.point.colors = self.cmap(intensity)[:, :, :3].reshape(-1, 3)
                return scan.to_legacy()

            return read_scan_with_intensities
        except ModuleNotFoundError:
            missing_libraries.append("open3d")
        except:
            tried_libraries.append("open3d")

        try:
            import trimesh

            trimesh.load(first_scan_file)
            return lambda file: np.asarray(trimesh.load(file).vertices)
        except ModuleNotFoundError:
            missing_libraries.append("trimesh")
        except:
            tried_libraries.append("trimesh")

        try:
            from pyntcloud import PyntCloud

            PyntCloud.from_file(first_scan_file)
            return lambda file: PyntCloud.from_file(file).points[["x", "y", "z"]].to_numpy()
        except ModuleNotFoundError:
            missing_libraries.append("pyntcloud")
        except:
            tried_libraries.append("pyntcloud")

        # If reach this point means that none of the librares exist/could read the file
        if not tried_libraries:
            print(
                "No 3D library is insallted in your system. Install one of the following "
                "to read the pointclouds"
            )
            print("\n".join(missing_libraries))
        else:
            print("[ERROR] File format not supported")

            print("Tried to load the point cloud with:")
            print("\n".join(tried_libraries))
            print("Skipped libraries (not installed):")
            print("\n".join(missing_libraries))
        sys.exit(1)
