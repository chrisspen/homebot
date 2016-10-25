#!/usr/bin/env python
"""
Simple use of SceneCanvas to display a cube with an arcball camera.

    sudo apt-get install libglfw3*
"""
import sys

from vispy import scene
from vispy.color import Color
canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#efefef'
view.camera = 'turntable'
view.padding = 100

color = Color("#3f51b5")

cube = scene.visuals.Cube(size=1, color=color, edge_color="black",
                          parent=view.scene)
if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()
