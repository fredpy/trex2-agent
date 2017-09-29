# T-REX:
## Version Chla  
This is the NTNU branch of the TREX code.

Contains - Updated measurements added to Estimated state:
CTD (Temperature, Salinity, Depth and Conductivity)
ECOpuck (TSM, cDOM, and Chl a)
Inclusion of PSI in Estimated state (for automatic front tracking)

### Dependencies

The core library of t-rex depends on:
 * cmake: http://www.cmake.org
 * C++ boost libraries 1.47 or above: http://www.boost.org

And these two are the only strong requirements to install t-rex
 (although a very bare-bone version of it).

The compilation is also setup by default to compile the europa
reactor which in turn requires:
* europa-pso 2.6:  https://github.com/nasa/europa

But you can safely disable europa via setting WITH_EUROPA to OFF
via cmake tools (ccmake or cmake-gui)

Code documentation (the little there is...) is formated for doxygen:
http://www.doxygen.org

Additionally t-rex generates few graphviz files in its log directory.
Installing graphviz (http://www.graphviz.org/) or compatible graph
visualization tools is a good way to visualize these graphs even if
not really required.

Further, optional plug-ins may require extra dependencies;
for example the ROS extension if enabled naturally expect ROS to
be installed (http://www.ros.org) and so on for others.
