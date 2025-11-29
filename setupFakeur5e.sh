gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=scaled_joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false '

sleep 5

gnome-terminal --title="MoveitServer" -- bash -c \
'ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true \
    use_fake_hardware:=true \
    description_package:=ur_with_gripper_description \
    description_file:=ur_with_gripper.xacro; \
    exec bash'

