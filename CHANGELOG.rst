^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package head_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
------------------
* Head movement now controlled by an action server
* Default head position (config.yaml file) now set to 90 for tilt
* pan_tilt_node/head_position topic now latched to ensure default values detetced down the chain
* maunal_view_step values added to config

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
