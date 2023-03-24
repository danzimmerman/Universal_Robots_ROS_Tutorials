ur_examples_gazebo_classic
--------------------------

These demos give some examples on how to use position or velocity control with UR robots in Gazebo Classic and ROS 2.

Requirements & Build
^^^^^^^^^^^^^^^^^^^^

You need to have the ``ur_description`` package installed. Currently you should use the ``dz/gz-classic-xacro-friction`` branch of 
`danzimmerman's fork of Universal_Robots_ROS2_Description <https://github.com/danzimmerman/Universal_Robots_ROS2_Description/tree/dz/gz-classic-xacro-friction>`_ 
to address the issues discussed in `PR #55 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/pull/55>`_

To build and use this package, copy it to your Colcon workspace, install its
dependencies using ``rosdep install --ignore-src --from-paths . -r -y`` and build your workspace as
usual.


Startup
^^^^^^^



.. code-block:: bash

   rosrun ur_example_dual_robot docker_alice_bob.sh

Wait, until the robots are started up. You can connect to the robots using their web interface:
 - Alice: `http://10.5.0.5:6080/vnc.html <http://10.5.0.5:6080/vnc.html>`_
 - Bob: `http://10.5.0.6:6080/vnc.html <http://10.5.0.6:6080/vnc.html>`_

When the robots have booted, start the driver instances as follows

.. code-block:: bash

   roslaunch ur_example_dual_robot dual_robot_startup.launch

This should startup the drivers, an RViz instance and an rqt_joint_trajectory_controller window.

To steer the robots, you'll first have to start the external_control program on both using the web
interface (the programs should be loaded by default, simply start the robots and press the play
button). In the shell running the drivers, you should now see ``Robot connected to reverse interface.
Ready to receive control commands.`` twice.

Using the rqt_joint_trajectory_controller window you can select one of the robots, click on the big
red button and then use the sliders to move the robots around.
