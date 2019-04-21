^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package head_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
------------------
* some parameters on the parameter server are now private
* Added license details

2.0.0 (2019-01-10)
------------------
* Now conforms to ROS standard using radians rotating around an axis
* Head movement now controlled by an action server
* Default head position (config.yaml file) now set to 90 for tilt
* pan_tilt_node/head_position topic now latched to ensure default values detetced down the chain

1.0.0 (2018-12-05)
------------------
* Removed action to scan for faces. Node now only controls servo speeds

0.1.2 (2018-09-26)
------------------
* Manual Camera movements

0.1.1 (2018-08-16)
------------------
* Update email address in package.xml file
* Moved list of seen people to this node
* Servo position velocity now ramps to avoid shuddering

0.1.0 (2018-06-14)
------------------
* First formal release of the package
