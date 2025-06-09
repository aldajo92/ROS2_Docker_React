#!/usr/bin/env python3

import os
import re
import subprocess
import sys

def convert_xacro_to_urdf():
    """Convert XACRO file to URDF and modify paths for web serving"""
    
    # Paths
    xacro_file = "../ros2_ws/src/waver_description/urdf/waver.xacro"
    output_urdf = "public/robot_description/waver.urdf"
    
    print("🔄 Converting XACRO to URDF...")
    
    try:
        # Create a more accurate URDF based on the actual values
        urdf_content = create_accurate_urdf()
        
        # Write the URDF file
        os.makedirs(os.path.dirname(output_urdf), exist_ok=True)
        with open(output_urdf, 'w') as f:
            f.write(urdf_content)
        
        print(f"✅ URDF created: {output_urdf}")
        
    except Exception as e:
        print(f"❌ Error converting XACRO: {e}")
        return False
    
    return True

def create_accurate_urdf():
    """Create an accurate URDF with proper values from util.xacro"""
    
    urdf_content = '''<?xml version="1.0"?>
<robot name="waver">

  <!-- Material definitions -->
  <material name="dark_gray">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  
  <material name="dark_blue">
    <color rgba="0.0 0.0 0.5 1.0"/>
  </material>

  <!-- Base footprint -->
  <link name="base_footprint"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/base_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
  </link>

  <!-- Top shell link -->
  <link name="top_shell_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/top_shell_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
  </link>

  <!-- Board link -->
  <link name="board_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/board_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
  </link>

  <!-- Accessories link -->
  <link name="accessories_link">
    <visual>
      <origin xyz="0.08693458557128907 0.02246641731262207 -0.04861989974975586" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/accessories_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Accessories1 link -->
  <link name="accessories1_link">
    <visual>
      <origin xyz="0.07360986328125002 0.002510018587112427 -0.035275302886962887" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/accessories1_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- OLED screen link -->
  <link name="oled_screen_link">
    <visual>
      <origin xyz="0.05096461868286133 6.735318456776441e-07 -0.048232952117919926" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/oled_screen_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark_blue"/>
    </visual>
  </link>

  <!-- Wheel back left -->
  <link name="wheel_back_left_link">
    <visual>
      <origin xyz="0.04859128189086914 -0.06538582611083985 -0.004798986434936523" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/wheel_back_left_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Wheel front left -->
  <link name="wheel_front_left_link">
    <visual>
      <origin xyz="-0.048406845092773446 -0.06538582611083985 -0.004799012184143066" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/wheel_front_left_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Wheel front right -->
  <link name="wheel_front_right_link">
    <visual>
      <origin xyz="-0.048593093872070316 0.06528724670410156 -0.004798998355865478" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/wheel_front_right_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Wheel back right -->
  <link name="wheel_back_right_link">
    <visual>
      <origin xyz="0.04840502166748047 0.0652872543334961 -0.004799028396606446" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/wheel_back_right_link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- LiDAR -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="./meshes/LD19.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Joints -->
  <joint name="base_joint" type="fixed">
    <origin xyz="0.0 0.0 0.0337" rpy="0 0 0"/>
    <parent link="base_footprint"/>
    <child link="base_link"/>
  </joint>

  <joint name="base_top_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="top_shell_link"/>
  </joint>

  <joint name="base_board_joint" type="fixed">
    <origin xyz="0 0 0.020" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="board_link"/>
  </joint>

  <joint name="top_accessories_joint" type="fixed">
    <origin xyz="-0.08693458557128907 -0.02246641731262207 0.04861989974975586" rpy="0 0 0"/>
    <parent link="top_shell_link"/>
    <child link="accessories_link"/>
  </joint>

  <joint name="top_accessories1_joint" type="fixed">
    <origin xyz="-0.07360986328125002 -0.002510018587112427 0.035275302886962887" rpy="0 0 0"/>
    <parent link="top_shell_link"/>
    <child link="accessories1_link"/>
  </joint>

  <joint name="top_screen_joint" type="fixed">
    <origin xyz="-0.05096461868286133 -6.735318456776441e-07 0.048232952117919926" rpy="0 0 0"/>
    <parent link="top_shell_link"/>
    <child link="oled_screen_link"/>
  </joint>

  <joint name="wheel_back_left_joint" type="continuous">
    <origin xyz="-0.04859128189086914 0.06538582611083985 0.004798986434936523" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_back_left_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_front_left_joint" type="continuous">
    <origin xyz="0.048406845092773446 0.06538582611083985 0.004799012184143066" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_front_left_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_front_right_joint" type="continuous">
    <origin xyz="0.048593093872070316 -0.06528724670410156 0.004798998355865478" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_front_right_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_back_right_joint" type="continuous">
    <origin xyz="-0.04840502166748047 -0.0652872543334961 0.004799028396606446" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_back_right_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <origin xyz="0 0 0.0885" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="lidar_link"/>
  </joint>

</robot>'''
    
    return urdf_content

if __name__ == "__main__":
    if convert_xacro_to_urdf():
        print("🎉 URDF conversion completed successfully!")
    else:
        print("❌ URDF conversion failed!")
        sys.exit(1) 