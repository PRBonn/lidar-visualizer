install:
	@pip install --verbose .

uninstall:
	@pip -v uninstall lidar_visualizer

clean:
	@git clean -xff .

editable:
	@pip install --verbose --editable .
