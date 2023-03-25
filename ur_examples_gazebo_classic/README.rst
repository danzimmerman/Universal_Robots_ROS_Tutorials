ur_examples_gazebo_classic
--------------------------

These demos give some examples on how to use position or velocity control with UR robots in Gazebo Classic and ROS 2.

Requirements & Build
^^^^^^^^^^^^^^^^^^^^

The ``ur_simulation_gazebo`` package is not yet released in binary form.

Currently, this tutorial also needs a customization of the ``ur_description`` package provided by
`danzimmerman's fork <https://github.com/danzimmerman/Universal_Robots_ROS2_Description/tree/dz/gz-classic-xacro-friction>`_ 

The fork is necessary to address other simulation issues discussed in `PR #55 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/pull/55>`_.

These unreleased dependencies are provided in the included ``.repos`` file for use with `vcstool <https://github.com/dirk-thomas/vcstool>`_. 
To install them, copy this package directory to your workspace ``src`` directory, and from the root of the workspace run:

.. code-block:: bash

   vcs import < src/ur_examples_gazebo_classic.repos


Basic Simulation
^^^^^^^^^^^^^^^^


Soft-Mounted Robot
^^^^^^^^^^^^^^^^^^



Some rst Hints
^^^^^^^^^^^^^^


Bullet list:
 - Alice: 
 - Bob: 

