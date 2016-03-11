This project is a virtual reality system for your desktop or laptop. You can view through your monitor like a virtual window, moving your head around a 3D model. Stereoscopy is enabled with blue/red glasses. Head tracking is done using the Kinect camera.

<a href='http://www.youtube.com/watch?feature=player_embedded&v=-I-El2t-vJU' target='_blank'><img src='http://img.youtube.com/vi/-I-El2t-vJU/0.jpg' width='425' height=344 /></a>

Required: Ubuntu with ROS, a Kinect (for head tracking) and a pair of blue/red glasses (for stereoscopy).

See [main.cpp](https://code.google.com/p/vvindow/source/browse/trunk/src/main.cpp) for the whole code.

# Build from source #
Install OpenSceneGraph from source (tested version 3.2.0) and put this **[patch](https://code.google.com/p/vvindow/source/browse/trunk/osgPatch.diff)** (issue explained [here](http://forum.openscenegraph.org/viewtopic.php?t=10991)).
```
 $ svn co http://svn.openscenegraph.org/osg/OpenSceneGraph/tags/OpenSceneGraph-3.2.0 OpenSceneGraph
 $ cd OpenSceneGraph
 $ patch -p0 -i osgPatch.diff
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make -j4
 $ sudo make install
```

You must have the openni\_tracker package:
```
 $ sudo apt-get install ros-groovy-openni-tracker 
```

Then in your ROS\_PACKAGE\_PATH
```
 $ svn checkout http://vvindow.googlecode.com/svn/trunk/ vvindow
 $ rosmake vvindow
```

# Run #

You should set **screenWidth** and **screenHeight** in [vvindow.launch](https://code.google.com/p/vvindow/source/browse/trunk/launch/vvindow.launch) (in meters). You should set also the transform between the Kinect and the center of the monitor!

```
 $ roslaunch vvindow openni.launch
 $ roslaunch vvindow vvindow.launch
```

## Controls ##

  * KEY\_RIGHT : Move model right
  * KEY\_LEFT : Move model left
  * KEY\_UP : Move model away from the user
  * KEY\_DOWN : Move model toward the user
  * KEY\_UP + CTRL : Move model up
  * KEY\_DOWN + CTRL : Move model down

  * KEY\_RIGHT + SHIFT : Rotate model to right
  * KEY\_LEFT + SHIFT : Rotate model to left
  * KEY\_UP + SHIFT : Rotate model to top
  * KEY\_DOWN + SHIFT : Rotate model to bottom

  * KEY\_PLUS : Scale bigger the model
  * KEY\_MINUS : Scale smaller the model

## Parameters ##
See [vvindow.launch](https://code.google.com/p/vvindow/source/browse/trunk/launch/vvindow.launch).

Order:
  1. If a **modelPath** is set, the model is used;
  1. Then if **cloud** is "true", cloud is used;
  1. Then a grid is used.