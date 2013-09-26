__WallFrame__ is envisioned as a high level framework for creating, managing and deploying interactive applications on __Large Scale Wall Displays__. WallFrame brings together several core elements and some elegant wrapping to provide an easy path to developing applications for _natural interaction with wall displays_:

  * Middleware from [OpenNI](http://www.openni.org/) and [NITE](http://www.primesense.com/solutions/nite-middleware/) to provide full 3D user tracking of up to 6 individuals.  This middleware is compatible with the [Microsoft Kinect](http://www.xbox.com/en-US/kinect), [ASUS Xtion Pro](http://www.asus.com/Multimedia/Xtion_PRO/) or other commercial depth sensor.
  * The [Qt Framework](http://qt-project.org/) for easy authoring of graphical user interfaces, and the easy management of windows across the multiple displays of a display wall
  * OpenGL, via Python PyGL or with [OpenSceneGraph](http://openscenegraph.org/) C++ wrapper for 3D application authoring
  * The [Robot Operating System](http://ros.org/), which allows for the easy and efficient connection of modular software components
  * The __WallFrame Core Libraries__ consist of a set of simple abstraction layers and tools to develop applications:
   * A user manager which publishes user interaction data (including 3D pose, gestures, events) to applications or widgets
   * An application manager to easily launch and manage applications from an intuitive and customisable menu interface
   * Tools for notifications and debugging
   * Base applications written in Python and C++ that can be inherited to provide direct access to user data and other resources, as well as display application data seamlessly across multiple displays 
   * Simple configuration files (using ROS and YAML) for changing application parameters without compiling
   * A set of [sample applications](https://github.com/futureneer/wallframe_apps)
