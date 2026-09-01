Python API
==========

Easy Motion provides high-level clients for robot motion and controller
management, together with utilities for ROS 2 poses and transforms.

Explore the API you need
------------------------

:doc:`Motion Client <easy_motion.motion_client>`
    Plan and execute robot motions, compute kinematics, control a gripper,
    and manage attached objects.

:doc:`Transform Utilities <easy_motion.easy_motion_utils>`
    Convert between ROS 2 poses, transforms, quaternions, and 4x4 affine
    matrices.

:doc:`Controller Manager Client <easy_motion.controller_manager_client>`
    Load, configure, activate, switch, inspect, and unload ros2_control
    controllers.

Advanced API
------------

:doc:`Motion Server <easy_motion.motion_server>`
    Explore the server-side implementation that exposes Easy Motion actions
    and services.

.. toctree::
   :hidden:
   :maxdepth: 2

   easy_motion.motion_client
   easy_motion.easy_motion_utils
   easy_motion.controller_manager_client
   easy_motion.motion_server
