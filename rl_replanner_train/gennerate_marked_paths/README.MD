# 生成路径到地图中步骤:

## step 1: 使用路径规划算法生成经过一系列指定路径点的路径
python3 generate_path.py

## step 2: 将路径生成到gazebo地图中，并标注路径点
python3 world_generator.py

## step 3：打开gazebo查看生成效果
export SVGA_VGPU10=0

gazebo /home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/maps/sim_maps/circle_clutter_with_path.world