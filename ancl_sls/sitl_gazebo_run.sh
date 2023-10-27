source ~/coderepo/PX4Hardware/PX4-Autopilot/Tools/setup_gazebo.bash ~/coderepo/PX4Hardware/PX4-Autopilot ~/coderepo/PX4Hardware/PX4-Autopilot/build/px4_sitl_default

# kill process names that might stil
# be running from last time
pkill gazebo || true
pkill gzserver || true
pkill px4 || true

# make px4_sitl_default gazebo no_sim=1 &
# sleep 1
gzserver ~/coderepo/PX4Hardware/PX4-Autopilot/Tools/sitl_gazebo/worlds/empty.world --verbose &
sleep 1
# gz model --spawn-file=/home/zifei/coderepo/PX4Hardware/PX4-Autopilot/Tools/sitl_gazebo/ancl_sls/iris_pendulum/iris_pendulum.sdf --model-name=test -x 0.0 -y 0.0 -z 0.2 &&
gz model --spawn-file=/home/zifei/coderepo/PX4Hardware/PX4-Autopilot/Tools/sitl_gazebo/models/firefly/firefly.sdf --model-name=test -x 0.0 -y 0.0 -z 0.2 &&
sleep 1
gzclient



