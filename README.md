# Drawbot Story 1
### Jonah Spear and Carl Moser

Our project idea is modifying the neato so that it can draw images using dots. This spans the range of low level (creating a pen actuator node on the neato) to high level (path planning and precise odometry).

We've been busy implementing a few key features that will be used in the final design.

### Dot Visualizer

Good visualizations have been really important so far in developing and debugging projects in this class. We decided to create a Dot Visualizer to accomplish this. This visualizer takes an image, and publishes the location of the dots that the neato will need to draw and publishes that to rviz.

![A comparison of an original image with the dot visualizer's output.](https://github.com/Joboman555/Drawbot/blob/master/resources/image_comparison.png)

The above image shows the input image, in our case a heart, and the output that rviz sees.
