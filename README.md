# AUV6 Communicator

This ROS Node provide a way to establish a dialog between AUV6 and ROS AUV7.

As we did not want to migrate the complete SONIA AUV Software to ROS at once,
we set up this little node that allow us to send messages from Java and send
the response to it.

## How to Compile AUV6 Communicator

### SONIA Dependencies

The SONIA Software architecture will require that some changes to your system.

If you did not follow the [SONIA Wiki](http://sonia.etsmtl.ca:120/soniapedia/index.php/Proc%C3%A9dure_d%27installation),
please read it before trying to do anything.

Here are the main step you should follow :

1. Be sure to have the Vitals library compiled and installed on your machine.

	Visit the SoniaWiki for the How-To section about vitals

2. Be sure to have the sonia_msgs project in your ROS package alongside
	VisionServer package

	It's in the same git branch, therefore, you should probably already have it

3. VisionClient will not compiled if you do not compile sonia_msgs before.

	In order to do this, please run `catkin_make` once then
	`source $SONIA_WORKSPACE_ROOT/ros/devel/setup.bash`
	 and once again `catkin_make`

4. Be sure to have `$SONIA_WORKSPACE_ROOT/vitals`
 in your `LD_LIBRARY_PATH` variable

	To do so, please run
	`export LD_LIBRARY_PATH=$SONIA_WORKSPACE_ROOT/vitals:$LD_LIBRARY_PATH`
	or add it to your `~/.bashrc`

### Compile the whole project

Finally, just execute a `catkin_make` in your `$SONIA_WORKSPACE_ROOT/ros`
directory:

```
cd $SONIA_WORKSPACE_ROOT/ros/ && \
source devel/setup.bash && \
catkin_make && \
cd -
```

## How to run AUV6 Communicator

AUV6 Communicator is on the node ros_java_communicator on ROS.
To launch the thing, simply run:

```
cd $SONIA_WORKSPACE_ROOT/ros/ && \
source devel/setup.bash && \
rosrun auv6_communicator ros_java_communicator && \
cd -
```
