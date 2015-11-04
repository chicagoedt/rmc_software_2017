^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ar_sys
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2014-09-30)
------------------
* Added arguments to launch files
* Contributors: Hamdi Sahloul

1.0.4 (2014-09-26)
------------------
* Added install instructions
* Contributors: Hamdi Sahloul

1.0.3 (2014-09-23)
------------------
* Changed size to support small screens
* Prefixed frame_ids by / for topic_tools/relay compatibility in groovy
* Employed topic_tools/relay to monitor both single and multi boards transform
* Update the package URL
* Remove false dependency
* Correct the default path of boards_config
* Contributors: Hamdi Sahloul

1.0.2 (2014-09-17)
------------------
* Update the package to declare its visualization_msgs dependency
* Initial tracks.yaml
* Contributors: Hamdi Sahloul

1.0.1 (2014-09-17)
------------------
* Fix the single marker coordinates to match ROS
* Introducing system viewer; a node that is capable of handling input from multi-cameras and display all the boards and virtual relative points in 3D using Rviz
* Replaced arrays and its count by vectors of enums
* Fixed the rotation of ArUco and OpenCV coordinates to match the camera and ROS specifications
* Enhanced time and source tracking of images
* Prevented a buffer overflow and made a naming convention
* Enhanced the multi detector performance by drawing the markers once, calculating the markers once using the minimum indicated size
* Fixed a bug in identification due to changes made to the orginal image
* Added launch command for the multi boards detector
* Added multi-boards detection capability
* Added a test board image for testing purposes
* Initial package: single board detection
* Contributors: Hamdi Sahloul
