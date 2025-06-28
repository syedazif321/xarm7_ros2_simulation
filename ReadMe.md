ros2 launch xarm7_bringup xarm7_bringup.launch.py
ros2 run xarm7_pipeline xarm7_pipeline_node 
ros2 run xarm7_pose_saver pose_saver 
ros2 service call /AttachDetach msg_gazebo/srv/AttachDetach "{model1: 'UF_ROBOT', link1: 'link7', model2: 'box_2', link2: 'link', attach: true}"
ros2 run xarm7_aruco_pick aruco_detector_node
