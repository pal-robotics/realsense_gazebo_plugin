# Intel RealSense Gazebo ROS plugin

This package is a Gazebo ROS plugin for the Intel D435 realsense camera.

This fork adds gazebo plugins for the IMU found in the Intel D435i realsense camera.

## Usage

Download this package and put it into your catkinws src folder and then compile the catkin package.
This will generate shared libraries called librealsense_gazebo_plugin.so,  for you which is used in the files you gonna download in the next steps.

Download the two xacro files [```_d435.gazebo.xacro```](https://raw.githubusercontent.com/pal-robotics-forks/realsense/upstream/realsense2_description/urdf/_d435.gazebo.xacro) and [```_d435.urdf.xacro```](https://raw.githubusercontent.com/pal-robotics-forks/realsense/upstream/realsense2_description/urdf/_d435.urdf.xacro) (they are from [pal-robotics-forks/realsense](https://github.com/pal-robotics-forks/realsense/tree/upstream/realsense2_description/urdf)).
Put those two files in the urdf folder of the package where you want your realsense to be simulated.

**In the file ```_d435.urdf.xacro``` change:**

**line 13** from:

```xml
<xacro:include filename="$(find realsense2_description)/urdf/urdf_d435.gazebo.xacro"/>
```

to: (note: replace ```packagename``` the name of your package)

```xml
<xacro:include filename="$(find packagename)/urdf/urdf_d435.gazebo.xacro"/>
```

**lines 144-145** from:

```xml
<!-- Realsense Gazebo Plugin -->
<xacro:gazebo_d435 camera_name="${name}" reference_link="${name}_link" topics_ns="${topics_ns}" depth_optical_frame="${name}_depth_optical_frame" color_optical_frame="${name}_color_optical_frame" infrared1_optical_frame="${name}_left_ir_optical_frame" infrared2_optical_frame="${name}_right_ir_optical_frame"/>
```

to:

```xml
<!-- camera accel joints and links -->
    <joint name="${name}_accel_joint" type="fixed">
      <origin xyz="-0.00174 0.00552 0.0176" rpy="0 0 0" />
      <parent link="${name}_link" />
      <child link="${name}_accel_frame" />
    </joint>
    <link name="${name}_accel_frame"/>

    <joint name="${name}_accel_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="${-M_PI/2} 0 ${-M_PI/2}" />
      <parent link="${name}_accel_frame" />
      <child link="${name}_accel_optical_frame" />
    </joint>
    <link name="${name}_accel_optical_frame"/>

    <!-- camera gyro joints and links -->
    <joint name="${name}_gyro_joint" type="fixed">
      <origin xyz="-0.00174 0.00552 0.0176" rpy="0 0 0" />
      <parent link="${name}_link" />
      <child link="${name}_gyro_frame" />
    </joint>
    <link name="${name}_gyro_frame"/>

    <joint name="${name}_gyro_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="${-M_PI/2} 0 ${-M_PI/2}" />
      <parent link="${name}_gyro_frame" />
      <child link="${name}_gyro_optical_frame" />
    </joint>
    <link name="${name}_gyro_optical_frame"/>

    <!-- Realsense Gazebo Plugin -->
    <xacro:gazebo_d435i camera_name="${name}"
                        reference_link="${name}_link"
                        topics_ns="${topics_ns}"
                        depth_optical_frame="${name}_depth_optical_frame"
                        color_optical_frame="${name}_color_optical_frame"
                        infrared1_optical_frame="${name}_left_ir_optical_frame"
                        infrared2_optical_frame="${name}_right_ir_optical_frame"
                        accel_optical_frame="${name}_accel_optical_frame"
                        gyro_optical_frame="${name}_gyro_optical_frame"
    />
```

**In the file ```_d435.urdf.xacro``` change:**

**line 12** from:

```xml
<xacro:macro name="gazebo_d435" params="camera_name reference_link topics_ns depth_optical_frame color_optical_frame infrared1_optical_frame infrared2_optical_frame" >
```

to:

```xml
<xacro:macro name="gazebo_d435i" params="camera_name reference_link topics_ns depth_optical_frame color_optical_frame infrared1_optical_frame infrared2_optical_frame accel_optical_frame gyro_optical_frame" >
```

**line 116-117** from:

```xml
  </sensor>
</gazebo>
```

to:

```xml
  </sensor>
  <sensor name="{camera_name}accel" type="imu">
    <always_on>true</always_on>
    <update_rate>300</update_rate>
    <topic>${camera_name}/accel/sample</topic>
    <plugin name="${topics_ns}_accel_plugin" filename="librealsense_gazebo_accel_sensor.so">
      <topicName>${camera_name}/accel/sample</topicName>
      <bodyName>imu_link</bodyName>
      <updateRateHZ>300.0</updateRateHZ>
      <gaussianNoise>0.1</gaussianNoise>
      <xyzOffset>0 0 0</xyzOffset>
      <rpyOffset>0 0 0</rpyOffset>
      <frameName>${accel_optical_frame}</frameName>
    </plugin>
    <pose>0 0 0 0 0 0</pose>
  </sensor>
  <sensor name="{camera_name}gyro" type="imu">
    <always_on>true</always_on>
    <update_rate>300</update_rate>
    <topic>${camera_name}/gyro/sample</topic>
    <plugin name="${topics_ns}_gyro_plugin" filename="librealsense_gazebo_gyro_sensor.so">
      <topicName>${camera_name}/gyro/sample</topicName>
      <bodyName>imu_link</bodyName>
      <updateRateHZ>300.0</updateRateHZ>
      <gaussianNoise>0.1</gaussianNoise>
      <xyzOffset>0 0 0</xyzOffset>
      <rpyOffset>0 0 0</rpyOffset>
      <frameName>${gyro_optical_frame}</frameName>
    </plugin>
    <pose>0 0 0 0 0 0</pose>
  </sensor>
</gazebo>
```

### Finally

In the urdf file where you want to use the simulated realsense add the following code, while again replacing ```packagename```,```tf_prefix```, and ```topic_prefix``` with you are using.

```xml
<xacro:include filename="$(find packagename)/urdf/_d435.urdf.xacro" />
<xacro:sensor_d435 parent="parent_link" name="tf_prefix" topics_ns="topic_prefix"/>
```

## Acknowledgement

This is a continuation of work done by [SyrianSpock](https://github.com/SyrianSpock) for a Gazebo ROS plugin with RS200 camera.

This package also includes the work developed by Intel Corporation with the ROS model fo the [D435](https://github.com/intel-ros/realsense) camera.
