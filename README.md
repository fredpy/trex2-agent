## T-REX: A deliberative executive for goal directed control

This project implements the t-rex agent that allows the execution of
multiple loosely coupled deliberative control loops called reactors.

The general idea is to distribute deliberation (such as automated
planning) into simpler control loops called _reactors_.

In contrast to classical planner+executive found in 3-tiered robotics
architectures, the system also tends to truly interleave planning and
execution in the sense that often (especially with the europa based
deliberative reactor) the execution will interrupt deliberation in
order to ensure that the agent is always aware of the world state
evolution.

While a reactor is not necessarily based on planning techniques, the
timeline based medium of interaction is perfectly fit for timelien
based planners and allow the user to abstract tasks into a more high
level goal directed control.

# Dependencies

The core library of t-rex depends on:
 * cmake: http://www.cmake.org
 * C++ boost libraries 1.47 or above: http://www.boost.org

The compilation is also setup by default to compile the europa
reactor which the will require:
* europa-pso 2.6:  https://github.com/nasa/europa

Code documentation (the little there is...) is formated for doxygen:
http://www.doxygen.org

Although one can disable europa via setting WITH_EUROPA to OFF
via cmake tools (ccmake or cmake-gui)

Further plugin may require extra dependencies; for example the ROS
extension if enabled naturally expect ROS to be installed
(http://www.ros.org) and so on for others.
