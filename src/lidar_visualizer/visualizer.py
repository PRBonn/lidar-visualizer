# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
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
import os
import importlib

# Button names
START_BUTTON = " START\n[SPACE]"
PAUSE_BUTTON = " PAUSE\n[SPACE]"
NEXT_FRAME_BUTTON = "NEXT FRAME\n\t\t [N]"
PREVIOUS_FRAME_BUTTON = "PREVIOUS FRAME\n\t\t\t [P]"
QUIT_BUTTON = "QUIT\n  [Q]"

# Colors
BACKGROUND_COLOR = [0.0, 0.0, 0.0]
FRAME_COLOR = [0.8470, 0.1058, 0.3764]  # Only used if no color in original cloud

# Size constants
FRAME_PTS_SIZE = 0.06


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

        # Initialize dataset and fix input based on its nature
        self._dataset = dataset
        self._random_accessible_dataset = random_accessible_dataset
        if not self._random_accessible_dataset:
            self.jump = 0
            self.n_scans = -1
        if n_scans == -1:
            self.n_scans = len(self._dataset)
            self.start_idx = 0
            self.stop_idx = self.n_scans
        else:
            self.n_scans = min(len(self._dataset) - jump, n_scans)
            self.start_idx = jump
            self.stop_idx = self.n_scans + jump
        self.idx = jump
        self.current_filename = self._get_current_filename(self.idx)

        # Initialize visualizer
        self._initialize_visualizer()

    def run(self):
        while True:
            self.update()
            self.advance()

    def update(self):
        self._update_visualized_frame()
        while True:
            self._ps.frame_tick()
            if self._play_mode:
                break

    def advance(self):
        self.idx = self.start_idx if self.idx == self.stop_idx - 1 else self.idx + 1

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

        # Let's do a bit of duck typing to support eating different monsters
        dataframe = self._dataset[idx]

    def _get_frame(self, idx):
        # Let's do a bit of duck typing to support eating different monsters
        dataframe = self._dataset[idx]

        try:
            # old KISS-ICP dataframe, spits points, timestamps. We don't care about the last
            points, colors, _ = dataframe  # TODO: this is not correct anymore
        except:
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

    # GUI Callbacks ---------------------------------------------------------------------------
    def _main_gui_callback(self):
        self._start_pause_callback()
        if not self._play_mode:
            self._gui.SameLine()
            self._next_frame_callback()
            if self._random_accessible_dataset:
                self._gui.SameLine()
                self._previous_frame_callback()
        self._gui.Separator()
        self._progress_bar_callback()
        if not self._random_accessible_dataset:
            self._gui.Separator()
            self._information_callback()
        self._gui.Separator()
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

    def _information_callback(self):
        self._gui.TextUnformatted(
            f"[WARNING] The current dataloader does not allow you to access frames\nrandomly, for this reason only sequential reading is possibile.\nFurthermore, the options '--jump' and '--n-scans' will be ignored \nand all the dataset, starting from the beginning will be visualized."
        )

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
