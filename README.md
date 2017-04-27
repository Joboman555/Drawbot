# Drawbot Story 1
## Jonah Spear and Carl Moser

Our project idea is modifying the neato so that it can draw images using dots. This spans the range of low level (creating a pen actuator node on the neato) to high level (path planning and precise odometry).

We've been busy implementing a few key features that will be used in the final design.

## Dot Visualizer

Good visualizations have been really important so far in developing and debugging projects in this class. We decided to create a Dot Visualizer to accomplish this. This visualizer takes an image, and publishes the location of the dots that the neato will need to draw and publishes that to rviz.

![A comparison of an original image with the dot visualizer's output.](https://github.com/Joboman555/Drawbot/blob/master/resources/image_comparison.png)

The above image shows the input image, in our case a heart, and the output that rviz sees. In the future, the Dot Visualizer will also get information back from the main running code to show more information, such as what dot is currently being navigated to.

## Pen Node
The pen node takes in a byte message that corresponds to the angle of the servo connected to the pen. It sends the message to a udp server that is running on the raspberry pi. The raspberry pi then sends that byte to the arduino over serial.

## Our First Test

Once the pen node was developed, we were able to do a quick test of our system drawing straight dots in a row. Although the quality of the dots needs to be improved, our general method seems promising.

![The first run of our neato](https://github.com/Joboman555/Drawbot/blob/master/resources/dry_run.gif)

(Sped up x6)

## Going Forward

In the coming weeks, we plan on making several changes to packaging that will improve reliability and overall accuracy of the pen actuator. In parallel, we will also be working on the path planner. We seem to be in a pretty good place with our progress so far, and are excited about the work we have in front of us.

# Drawbot Story 2

# Pen Holder

# Our Second Test