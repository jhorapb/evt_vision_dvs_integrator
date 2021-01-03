# dvs_integrator

Event-by-event processing in ROS C++ for the purpose of image reconstruction.

The package reads events from a topic and outputs the reconstructed images using per-pixel temporal filters.

### Input / Output
**Input**:
- events (topic)

**Output**:
- state (for illustration purposes): time surface
- reconstructed intensity

**Parameters** (dynamic reconfigure):
- Decaying factor ("cutoff frequency" alpha).
- Selector of the convolution mask (e.g., identity, Sobel-x, Sobel-y, Blur, Laplacian)
- Rate of output images published (number of events or Delta-t)

### Dependencies

Requires [catkin_simple](https://github.com/catkin/catkin_simple) and [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) driver

### Compile

**Preliminaries**:
First, build `catkin simple` and the `davis_ros_driver`. The most important commands are:

	catkin build catkin_simple
	catkin build davis_ros_driver

Then, incorporate the packages to the path, so that ROS finds them:
	
	source ~/ros/catkin_ws_evis/devel/setup.bash
	
**Compile this package**:
	
	catkin build dvs_integrator --force-cmake
	
The flag `--force-cmake` is optional.	
After building, at least the first time, remember to run:

	source ~/ros/catkin_ws_evis/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/dvs_integrator/
	
An alternative command to start from scratch (cleaning all catkin pakages) is (to be used with *caution*): `catkin clean`


### Run example
Download a ROS bag dataset, for example [slider_depth](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag) to play it in a terminal, that is, to use it as source of data (as if an event camera was connected to the computer).

Every time that you open a terminal, you may need to run:

	source ~/ros/catkin_ws_evis/devel/setup.bash

to have access to the ROS commands.

Run in a terminal (you may need to "source" first):

	roscore
	
In another terminal play the bag with the event data:

	rosbag play -l path_to_file/slider_depth.bag
	
Then, in another terminal play run the visualizer node:
	
	roslaunch dvs_integrator display_monocular.launch
	
In another terminal open the dynamic reconfigure and play around with the parameters in the window named `dvs_displayer_one`
	
	rosrun rqt_reconfigure rqt_reconfigure

End the program execution with `Ctrl + C` keyboard shortcut. 


### TO DO:
- Display events using a fixed number of them (currently we are displaying the events as arranged in messages)
- Display events using a fixed time interval
- Allow interaction, to change the number of events or the size of the time slice
- Add visualization of time surfaces
