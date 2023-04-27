# MIT License
#
# Copyright (c) 2023 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
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
from pathlib import Path
from typing import Optional

import typer
from kiss_icp.datasets import (
    available_dataloaders,
    dataset_factory,
    jumpable_dataloaders,
    sequence_dataloaders,
    supported_file_extensions,
)
from kiss_icp.tools.cmd import guess_dataloader, name_callback
from kiss_icp.tools.visualizer import RegistrationVisualizer
from tqdm import tqdm


def version_callback(value: bool):
    from kiss_icp.tools.cmd import version_callback as kiss_icp_version_callback

    if value:
        import lidar_visualizer

        print(f"Lidar Visualizer Version: {lidar_visualizer.__version__}")
        return kiss_icp_version_callback(value)


class Visualizer(RegistrationVisualizer):
    def __init__(self, dataset, n_scans: int = -1, jump: int = 0):
        super().__init__()
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
        self.pbar = tqdm(total=self.n_scans)
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
    def _next_frame(self, vis):
        self.play_crun = False
        self.advance()
        self.update(False)

    def _prev_frame(self, vis):
        self.play_crun = False
        self.rewind()
        self.update(False)

    def _get_frame(self, idx):
        dataframe = self._dataset[idx]
        try:
            frame, _ = dataframe
        except ValueError:
            frame = dataframe
        return frame

    def _update_geometries(self, source):
        self.source.points = self.o3d.utility.Vector3dVector(source)
        self.vis.update_geometry(self.source)
        if self.reset_bounding_box:
            self.vis.reset_view_point(True)
            self.reset_bounding_box = False

    # GUI controls ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        w_name = self.__class__.__name__
        self.vis.create_window(window_name=w_name, width=1920, height=1080)
        self.vis.add_geometry(self.source)
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

    def _register_key_callbacks(self):
        self._register_key_callback(["Ā", "Q", "\x1b"], self._quit)
        self._register_key_callback([" "], self._start_stop)
        self._register_key_callback(["N"], self._next_frame)
        self._register_key_callback(["P"], self._prev_frame)
        self._register_key_callback(["B"], self._set_black_background)
        self._register_key_callback(["W"], self._set_white_background)


docstring = f"""
:kiss: KISS-ICP dataset visualizer :kiss:\n
\b
[bold green]Examples: [/bold green]
# Visualize all pointclouds in the given <data-dir> \[{", ".join(supported_file_extensions())}]
$ lidar_visualizer <data-dir>:open_file_folder:

# Visualize a given [bold]ROS1/ROS2 [/bold]rosbag file (directory:open_file_folder:, ".bag":page_facing_up:, or "metadata.yaml":page_facing_up:)
$ lidar_visualizer <path-to-my-rosbag>[:open_file_folder:/:page_facing_up:]

# Visualize [bold]mcap [/bold] recording
$ lidar_visualizer <path-to-file.mcap>:page_facing_up:

# Visualize [bold]Ouster pcap[/bold] recording (requires ouster-sdk Python package installed)
$ lidar_visualizer <path-to-ouster.pcap>:page_facing_up: \[--meta <path-to-metadata.json>:page_facing_up:]

# Use a more specific dataloader: {", ".join(available_dataloaders())}
$ lidar_visualizer --dataloader kitti --sequence 07 <path-to-kitti-root>:open_file_folder:

"""

app = typer.Typer(add_completion=False, rich_markup_mode="rich")


@app.command(help=docstring)
def lidar_visualizer(
    data: Path = typer.Argument(
        ...,
        help="The data directory used by the specified dataloader",
        show_default=False,
    ),
    dataloader: str = typer.Option(
        None,
        show_default=False,
        case_sensitive=False,
        autocompletion=available_dataloaders,
        callback=name_callback,
        help="[Optional] Use a specific dataloader from those supported by KISS-ICP",
    ),
    sequence: Optional[int] = typer.Option(
        None,
        "--sequence",
        "-s",
        show_default=False,
        help="[Optional] For some dataloaders, you need to specify a given sequence",
        rich_help_panel="Additional Options",
    ),
    topic: Optional[str] = typer.Option(
        None,
        "--topic",
        "-t",
        show_default=False,
        help="[Optional] Only valid when processing rosbag files",
        rich_help_panel="Additional Options",
    ),
    n_scans: int = typer.Option(
        -1,
        "--n-scans",
        "-n",
        show_default=False,
        help="[Optional] Specify the number of scans to process, default is the entire dataset",
        rich_help_panel="Additional Options",
    ),
    jump: int = typer.Option(
        0,
        "--jump",
        "-j",
        show_default=False,
        help="[Optional] Specify if you want to start to process scans from a given starting point",
        rich_help_panel="Additional Options",
    ),
    meta: Optional[Path] = typer.Option(
        None,
        "--meta",
        "-m",
        exists=True,
        show_default=False,
        help="[Optional] For Ouster pcap dataloader, specify metadata json file path explicitly",
        rich_help_panel="Additional Options",
    ),
    version: Optional[bool] = typer.Option(
        None,
        "--version",
        help="Show the current version of KISS-ICP",
        callback=version_callback,
        is_eager=True,
    ),
):
    if not dataloader:
        dataloader, data = guess_dataloader(data, default_dataloader="generic")

    # Validate some options
    if dataloader in sequence_dataloaders() and sequence is None:
        print('You must specify a sequence "--sequence"')
        raise typer.Exit(code=1)

    if (jump != 0 or n_scans != -1) and dataloader not in jumpable_dataloaders():
        print(f"[WARNING] '{dataloader}' does not support '-jump' or '--n_scans'")
        print(f"[WARNING] Visualazing entire dataset")
        jump = 0
        n_scans = -1

    Visualizer(
        dataset=dataset_factory(
            dataloader=dataloader,
            data_dir=data,
            # Additional options
            sequence=sequence,
            topic=topic,
            meta=meta,
        ),
        n_scans=n_scans,
        jump=jump,
    ).run()


def main():
    app()
