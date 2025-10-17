# camera_bot

Implementation of a camera-mounted steerable-wheel robot.

---

## Requirements

- ROS 2 Jazzy
- `ros_gz_sim` 
- `xacro`
- `ros2_control`, `ros2_controllers`, `controller_manager`
- A workspace with this package in `src/`

---

## 1) Build

~~~bash
# from your workspace root
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select camera_bot --cmake-clean-cache --symlink-install
. install/setup.bash
~~~

---

## 2) Pub/Sub

Publisher:
~~~bash
ros2 run camera_bot publisher \
  --ros-args -p topic:=/readings -p publish_rate_hz:=10.0
~~~

Subscriber in another terminal:
~~~bash
ros2 run camera_bot subscriber \
  --ros-args -p topic:=/readings -p threshold:=0.5
~~~

This pub/sub checks whether the published value exceeds the threshold.

---

## 3) Generate URDF for Robot State Publisher (RSP) purpose

### 3.1 Convert XACRO to URDF
~~~bash
PKG_PREFIX="$(ros2 pkg prefix camera_bot)"
XACRO_IN="${PKG_PREFIX}/share/camera_bot/description/steer.urdf.xacro"
ASSETS="file://${PKG_PREFIX}/share/camera_bot/description/assets"
PARAMS="${PKG_PREFIX}/share/camera_bot/config/controllers.yaml"

xacro "$XACRO_IN" \
  mesh_dir:="${ASSETS}" \
  ros_namespace:=ugv \
  rsp_node:=/ugv/robot_state_publisher \
  params_file:="${PARAMS}" \
  > /tmp/steer.urdf
~~~

### 3.2 Build `/tmp/rsp.yaml` from the URDF
~~~bash
rm -f /tmp/rsp.yaml
printf "/ugv/robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n" > /tmp/rsp.yaml
tail -n +2 /tmp/steer.urdf | sed 's/^/      /' >> /tmp/rsp.yaml
~~~

---

## 4) Start Gazebo and spawn the robot

Gazebo:
~~~bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r ~/camera_bot/worlds/world.sdf"
~~~

Spawn in separate terminal:
~~~bash
ros2 run ros_gz_sim create -name ugv -file /tmp/steer.urdf -z 0.3
~~~

---

## 5) Run RSP

~~~bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -r __ns:=/ugv --params-file /tmp/rsp.yaml
~~~

---

## 6) Controllers

Create minimal Joint State Broadcaster (JSB) params:
~~~bash
cat > /tmp/jsb.yaml <<'YAML'
joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster
    interfaces: [position, velocity]
YAML
~~~

Spawn JSB:
~~~bash
ros2 run controller_manager spawner joint_state_broadcaster \
  -c /ugv/controller_manager --param-file /tmp/jsb.yaml --controller-manager-timeout 20.0
~~~

Set controller types:
~~~bash
ros2 param set /ugv/controller_manager steering_controller.type position_controllers/JointGroupPositionController
ros2 param set /ugv/controller_manager rear_wheels_controller.type velocity_controllers/JointGroupVelocityController
~~~

Ensure controllers are loaded:
~~~bash
ros2 control load_controller -c /ugv/controller_manager steering_controller    2>/dev/null || true
ros2 control load_controller -c /ugv/controller_manager rear_wheels_controller 2>/dev/null || true
~~~

Provide required params:
~~~bash
ros2 param set /ugv/steering_controller    joints "['steer_fl','steer_fr']"
ros2 param set /ugv/rear_wheels_controller joints "['spin_rl','spin_rr']"
~~~

Configure and activate:
~~~bash
ros2 control set_controller_state -c /ugv/controller_manager steering_controller inactive
ros2 control set_controller_state -c /ugv/controller_manager steering_controller active
ros2 control set_controller_state -c /ugv/controller_manager rear_wheels_controller inactive
ros2 control set_controller_state -c /ugv/controller_manager rear_wheels_controller active
~~~

Verify:
~~~bash
ros2 control list_controllers -c /ugv/controller_manager
ros2 control list_hardware_interfaces -c /ugv/controller_manager | sed -n '/command interfaces/,$p'
~~~

---

## 7) Manual command test (optional)

~~~bash
# steering angles (radians) for both front knuckles
ros2 topic pub -1 /ugv/steering_controller/commands std_msgs/msg/Float64MultiArray "data: [0.2, 0.2]"

# rear wheels angular velocity (rad/s)
ros2 topic pub -1 /ugv/rear_wheels_controller/commands std_msgs/msg/Float64MultiArray "data: [4.0, 4.0]"
~~~

---

## Troubleshooting

- Package not visible:
  ~~~bash
  source /opt/ros/$ROS_DISTRO/setup.bash
  source install/setup.bash
  ros2 pkg list | grep camera_bot
  ~~~
- Cache/source mismatch errors: clean the workspace root
  ~~~bash
  rm -rf build/ install/ log/
  colcon build --packages-select camera_bot --symlink-install
  ~~~
- No subscribers on `commands`: ensure `/ugv` namespace is used and controllers are active.


