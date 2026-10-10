# COBOTTA mesh source

Files: base_link.dae, J1-J6.dae (arm), gripper_base.dae, left_finger.dae, right_finger.dae (parallel hand).

From DENSO's own ROS package, unmodified:
https://github.com/DENSORobot/denso_cobotta_ros
  denso_cobotta_descriptions/cobotta_description/

Units are metres, so no scaling is applied. Mesh frames are the URDF link frames;
the DH alignment (qtest / qtest_transforms) is in the model code.

Kinematics in the same repo: denso_cobotta_descriptions/cobotta_description/cobotta.urdf.xacro
Licence: see LICENSE_denso_cobotta_meshes.txt (MIT, DENSO WAVE INCORPORATED).
