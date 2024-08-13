# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
# Copyright (c) 2024  Luca Lobefaro
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
import time

# Button names
START_BUTTON = "START [SPACE]"
PAUSE_BUTTON = "PAUSE [SPACE]"
NEXT_FRAME_BUTTON = "NEXT FRAME [N]"
PREVIOUS_FRAME_BUTTON = "PREVIOUS FRAME [P]"
CENTER_VIEWPOINT_BUTTON = "CENTER VIEWPOINT [C]"
QUIT_BUTTON = "QUIT [Q]"

# Colors
BACKGROUND_COLOR = [0.0, 0.0, 0.0]
FRAME_COLOR = [0.8470, 0.1058, 0.3764]  # Only used if no color in original cloud

# Size constants
FRAME_PTS_SIZE = 0.06
FRAME_PTS_SIZE_STEP = 0.01
FRAME_PTS_SIZE_MIN = 0.001
FRAME_PTS_SIZE_MAX = 0.25


class Visualizer:
    def __init__(self, dataset, random_accessible_dataset: bool, n_scans: int = -1, jump: int = 0):
        try:
            self._ps = importlib.import_module("polyscope")
            self._gui = self._ps.imgui
        except ModuleNotFoundError as err:
            print(f'polyscope is not installed on your system, run "pip install polyscope"')
            exit(1)

        # Initialize GUI controls
        self._background_color = BACKGROUND_COLOR
        self._frame_size = FRAME_PTS_SIZE
        self._play_mode = False
        self._toggle_frame = True
        self._speed_level = 5

        # Initialize dataset and fix input based on its nature
        self._dataset = dataset
        self._random_accessible_dataset = random_accessible_dataset
        self.start_idx = min(jump, len(self._dataset) - 1) if self._random_accessible_dataset else 0
        self.n_scans = len(self._dataset) if n_scans == -1 else min(len(self._dataset), n_scans)
        self.stop_idx = min(len(self._dataset), self.n_scans + self.start_idx)
        self.idx = self.start_idx
        self.current_filename = self._get_current_filename(self.idx)
        self.end_reached = False

        # Initialize visualizer
        self._initialize_visualizer()

    def run(self):
        while True:
            self.update()
            self.advance()

    def update(self):
        self._update_visualized_frame()
        while True:
            time.sleep(self._compute_speed())
            self._ps.frame_tick()
            if self._play_mode and not self.end_reached:
                break

    def advance(self):
        self.idx = self.start_idx if self.idx == self.stop_idx - 1 else self.idx + 1
        self.end_reached = self.idx == self.stop_idx - 1 and not self._random_accessible_dataset

    def rewind(self):
        self.idx = self.stop_idx - 1 if self.idx == self.start_idx else self.idx - 1

    # Private Interface ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        self._ps.set_program_name("LIDAR Visualizer")
        self._ps.init()
        self._ps.set_ground_plane_mode("none")
        self._ps.set_background_color(BACKGROUND_COLOR)
        self._ps.set_verbosity(0)
        self._ps.set_user_callback(self._main_gui_callback)
        self._ps.set_build_default_gui_panels(False)

    def _get_current_filename(self, idx):
        # Try to fetch the current filename
        try:
            filename = self._dataset.scan_files[idx]
            return os.path.splitext(os.path.basename(filename))[0]
        except:
            return None

    def _get_frame(self, idx):
        # Let's do a bit of duck typing to support eating different monsters
        dataframe = self._dataset[idx]
        points, colors = dataframe
        return points, colors

    def _update_visualized_frame(self):
        self.current_filename = self._get_current_filename(self.idx)
        points, colors = self._get_frame(self.idx)
        self._register_frame(points, colors)

    def _register_frame(self, points, colors):
        frame_cloud = self._ps.register_point_cloud(
            "current_frame",
            points,
            point_render_mode="quad",
        )
        if colors is None:
            frame_cloud.set_color(FRAME_COLOR)
        else:
            frame_cloud.add_color_quantity("colors", colors, enabled=True)
        frame_cloud.set_radius(self._frame_size, relative=False)
        frame_cloud.set_enabled(self._toggle_frame)

    def _compute_speed(self):
        return 0.1 - 0.02 * self._speed_level

    # GUI Callbacks ---------------------------------------------------------------------------
    def _main_gui_callback(self):
        self._gui.TextUnformatted("Controls:")
        if not self.end_reached:
            self._start_pause_callback()
            if not self._play_mode:
                self._gui.SameLine()
                self._next_frame_callback()
                if self._random_accessible_dataset:
                    self._gui.SameLine()
                    self._previous_frame_callback()
        self._gui.Separator()
        self._progress_bar_callback()
        self._speed_level_callback()
        self._gui.Separator()
        self._gui.TextUnformatted("Scene Options:")
        self._background_color_callback()
        self._points_controlles_callback()
        if not self._random_accessible_dataset:
            self._gui.Separator()
            self._information_callback()
        self._gui.Separator()
        self._center_viewpoint_callback()
        self._gui.SameLine()
        self._quit_callback()

    def _start_pause_callback(self):
        button_name = PAUSE_BUTTON if self._play_mode else START_BUTTON
        if self._gui.Button(button_name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_Space):
            self._play_mode = not self._play_mode

    def _next_frame_callback(self):
        if self._gui.Button(NEXT_FRAME_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_N):
            self.advance()
            self._update_visualized_frame()

    def _previous_frame_callback(self):
        if self._gui.Button(PREVIOUS_FRAME_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_P):
            self.rewind()
            self._update_visualized_frame()

    def _progress_bar_callback(self):
        changed, idx = self._gui.SliderInt(
            f"/{self.stop_idx-1}###Progress Bar",
            self.idx,
            v_min=self.start_idx,
            v_max=self.stop_idx - 1,
            format="Frame: %d",
        )
        if changed and self._random_accessible_dataset:
            self.idx = idx
            self._update_visualized_frame()

    def _speed_level_callback(self):
        _, self._speed_level = self._gui.SliderInt(
            "Speed Level",
            self._speed_level,
            v_min=0,
            v_max=5,
            format="%d",
        )

    def _points_controlles_callback(self):
        key_changed = False
        if self._gui.IsKeyPressed(self._gui.ImGuiKey_Minus):
            self._frame_size = max(FRAME_PTS_SIZE_MIN, self._frame_size - FRAME_PTS_SIZE_STEP)
            key_changed = True
        if self._gui.IsKeyPressed(self._gui.ImGuiKey_Equal):
            self._frame_size = min(FRAME_PTS_SIZE_MAX, self._frame_size + FRAME_PTS_SIZE_STEP)
            key_changed = True
        changed, self._frame_size = self._gui.SliderFloat(
            "Points Size", self._frame_size, v_min=FRAME_PTS_SIZE_MIN, v_max=FRAME_PTS_SIZE_MAX
        )
        if changed or key_changed:
            self._ps.get_point_cloud("current_frame").set_radius(self._frame_size, relative=False)

    def _background_color_callback(self):
        changed, self._background_color = self._gui.ColorEdit3(
            "Background Color",
            self._background_color,
        )
        if changed:
            self._ps.set_background_color(self._background_color)

    def _information_callback(self):
        self._gui.TextUnformatted(
            f"[WARNING] The current dataloader does not allow you to access frames\nrandomly..."
        )

    def _center_viewpoint_callback(self):
        if self._gui.Button(CENTER_VIEWPOINT_BUTTON) or self._gui.IsKeyPressed(
            self._gui.ImGuiKey_C
        ):
            self._ps.reset_camera_to_home_view()

    def _quit_callback(self):
        self._gui.SetCursorPosX(
            self._gui.GetCursorPosX() + self._gui.GetContentRegionAvail()[0] - 50
        )
        if (
            self._gui.Button(QUIT_BUTTON)
            or self._gui.IsKeyPressed(self._gui.ImGuiKey_Escape)
            or self._gui.IsKeyPressed(self._gui.ImGuiKey_Q)
        ):
            print("Destroying Visualizer")
            self._ps.unshow()
            os._exit(0)
