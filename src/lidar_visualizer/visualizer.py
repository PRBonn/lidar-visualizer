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
from functools import partial
from typing import Callable, List

from tqdm import tqdm


class Visualizer:
    def __init__(self, dataset, n_scans: int = -1, jump: int = 0):
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as e:
            raise ModuleNotFoundError(
                "Open3D is not installed on your system, to fix this either "
                'run "pip install open3d" '
                "or check https://www.open3d.org/docs/release/getting_started.html"
            ) from e
        # Initialize GUI controls
        self.block_vis = True
        self.play_crun = False
        self.reset_bounding_box = True

        # Create data
        self.source = self.o3d.geometry.PointCloud()

        # Initialize visualizer
        self.vis = self.o3d.visualization.VisualizerWithKeyCallback()
        self._register_key_callbacks()
        self._initialize_visualizer()

        self._dataset = dataset
        if n_scans == -1:
            self.n_scans = len(self._dataset)
            self.start_idx = 0
            self.stop_idx = self.n_scans
        else:
            self.n_scans = min(len(self._dataset) - jump, n_scans)
            self.start_idx = jump
            self.stop_idx = self.n_scans + jump

        self.idx = jump
        self.pbar = tqdm(total=self.n_scans, dynamic_ncols=True)
        self.update_pbar()

    def run(self):
        while True:
            self.update()
            self.advance()

    def update(self, poll_events=True):
        source = self._get_frame(self.idx)
        self._update_geometries(source)
        while poll_events:
            self.vis.poll_events()
            self.vis.update_renderer()
            if self.play_crun:
                break

    def advance(self):
        self.idx = self.start_idx if self.idx == self.stop_idx - 1 else self.idx + 1
        self.update_pbar()

    def rewind(self):
        self.idx = self.start_idx if self.idx == self.stop_idx - 1 else self.idx - 1
        self.update_pbar()

    def update_pbar(self):
        self.pbar.n = self.idx % self.n_scans
        self.pbar.refresh()

    # Private Interaface ---------------------------------------------------------------------------
    def _get_frame(self, idx):
        # Let's do a bit of duck typing to support eating different monsters
        dataframe = self._dataset[idx]
        try:
            # old KISS-ICP dataframe, spits points, timestamps. We don't care about the last
            frame, _ = dataframe
        except:
            frame = dataframe
        if not isinstance(frame, self.o3d.geometry.PointCloud):
            # convert to Open3D::Geometry::PointCloud
            frame = self.o3d.geometry.PointCloud(self.o3d.utility.Vector3dVector(frame))
        return frame

    def _next_frame(self, vis):
        self.play_crun = False
        self.advance()
        self.update(False)

    def _prev_frame(self, vis):
        self.play_crun = False
        self.rewind()
        self.update(False)

    def _update_geometries(self, source):
        self.source.points = source.points
        self.source.colors = source.colors
        self.vis.update_geometry(self.source)
        if self.reset_bounding_box:
            self.vis.reset_view_point(True)
            self.reset_bounding_box = False

    # GUI controls ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        w_name = self.__class__.__name__
        self.vis.create_window(window_name=w_name, width=1920, height=1080)
        self.vis.add_geometry(self.source, reset_bounding_box=False)
        self._set_black_background(self.vis)
        self.vis.get_render_option().point_size = 1
        print(
            f"{w_name} initialized. Press:\n"
            "\t[SPACE] to pause/start\n"
            "\t  [ESC] to exit\n"
            "\t    [N] to render next frame\n"
            "\t    [P] to render prev frame\n"
            "\t    [W] to toggle a white background\n"
            "\t    [B] to toggle a black background\n"
        )

    def _register_key_callback(self, keys: List, callback: Callable):
        for key in keys:
            self.vis.register_key_callback(ord(str(key)), partial(callback))

    def _register_key_callbacks(self):
        self._register_key_callback(["Ā", "Q", "\x1b"], self._quit)
        self._register_key_callback([" "], self._start_stop)
        self._register_key_callback(["N"], self._next_frame)
        self._register_key_callback(["P"], self._prev_frame)
        self._register_key_callback(["B"], self._set_black_background)
        self._register_key_callback(["W"], self._set_white_background)

    def _set_black_background(self, vis):
        vis.get_render_option().background_color = [0.0, 0.0, 0.0]

    def _set_white_background(self, vis):
        vis.get_render_option().background_color = [1.0, 1.0, 1.0]

    def _quit(self, vis):
        print("Destroying Visualizer")
        vis.destroy_window()
        os._exit(0)

    def _start_stop(self, vis):
        self.play_crun = not self.play_crun
