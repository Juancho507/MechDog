#!/usr/bin/env python
import rospy
import os
import yaml
from string import Template


class ScenarioBuilder:
    OBSTACLE_TEMPLATES = {
        'box': '''    <model name="$name">
      <static>true</static>
      <pose>$px $py $pz $roll $pitch $yaw</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>$sx $sy $sz</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>$sx $sy $sz</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>$color</name></script></material>
        </visual>
      </link>
    </model>''',

        'cylinder': '''    <model name="$name">
      <static>true</static>
      <pose>$px $py $pz $roll $pitch $yaw</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>$radius</radius><length>$length</length></cylinder></geometry></collision>
        <visual name="visual">
          <geometry><cylinder><radius>$radius</radius><length>$length</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>$color</name></script></material>
        </visual>
      </link>
    </model>''',

        'sphere': '''    <model name="$name">
      <static>true</static>
      <pose>$px $py $pz $roll $pitch $yaw</pose>
      <link name="link">
        <collision name="collision"><geometry><sphere><radius>$radius</radius></sphere></geometry></collision>
        <visual name="visual">
          <geometry><sphere><radius>$radius</radius></sphere></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>$color</name></script></material>
        </visual>
      </link>
    </model>'''
    }

    WORLD_TEMPLATE = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="${world_name}">

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>

${obstacles}

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-${cam_x} ${cam_y} ${cam_z} 0 ${cam_pitch} ${cam_yaw}</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>'''

    def __init__(self):
        self.scenarios_dir = rospy.get_param('~scenarios_dir', '')

    def build_from_yaml(self, config_path, output_dir):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        scenarios = config.get('scenarios', {})
        world_files = []

        for name, scenario in scenarios.items():
            world_content = self.build_world(name, scenario)
            world_path = os.path.join(output_dir, f'scenario_{name}.world')
            with open(world_path, 'w') as f:
                f.write(world_content)
            world_files.append(world_path)
            rospy.loginfo("Generated world: %s (%d obstacles)", name, len(scenario.get('obstacles', [])))

        return world_files

    def build_world(self, name, scenario):
        obstacles = scenario.get('obstacles', [])
        obstacle_sdf = []
        for i, obs in enumerate(obstacles):
            obs_type = obs.get('type', 'box')
            pose = obs.get('pose', [0, 0, 0, 0, 0, 0])
            color = obs.get('color', 'Gazebo/Red')
            obs_name = f"obstacle_{obs_type}_{i+1}"

            if obs_type == 'box':
                size = obs.get('size', [0.3, 0.3, 0.3])
                t = Template(self.OBSTACLE_TEMPLATES['box'])
                obstacle_sdf.append(t.safe_substitute(
                    name=obs_name,
                    px=pose[0], py=pose[1], pz=pose[2],
                    roll=pose[3], pitch=pose[4], yaw=pose[5],
                    sx=size[0], sy=size[1], sz=size[2],
                    color=color
                ))
            elif obs_type == 'cylinder':
                radius = obs.get('radius', 0.2)
                length = obs.get('length', 0.4)
                t = Template(self.OBSTACLE_TEMPLATES['cylinder'])
                obstacle_sdf.append(t.safe_substitute(
                    name=obs_name,
                    px=pose[0], py=pose[1], pz=pose[2],
                    roll=pose[3], pitch=pose[4], yaw=pose[5],
                    radius=radius, length=length,
                    color=color
                ))
            elif obs_type == 'sphere':
                radius = obs.get('radius', 0.15)
                t = Template(self.OBSTACLE_TEMPLATES['sphere'])
                obstacle_sdf.append(t.safe_substitute(
                    name=obs_name,
                    px=pose[0], py=pose[1], pz=pose[2],
                    roll=pose[3], pitch=pose[4], yaw=pose[5],
                    radius=radius,
                    color=color
                ))

        map_size = scenario.get('map_size', [12, 12])
        obstacles_str = '\n'.join(obstacle_sdf)

        t = Template(self.WORLD_TEMPLATE)
        return t.safe_substitute(
            world_name=f"mechdog_objects_{name}",
            obstacles=obstacles_str,
            cam_x=max(3, map_size[0] * 0.3),
            cam_y=3,
            cam_z=map_size[0] * 0.7,
            cam_pitch=0.4,
            cam_yaw=0.3
        )


if __name__ == '__main__':
    rospy.init_node('scenario_builder', anonymous=True)
    builder = ScenarioBuilder()

    pkg_path = rospy.get_param('~pkg_path', '/app/catkin_ws/src/mechdog_experiments')
    config_path = os.path.join(pkg_path, 'config', 'scenarios.yaml')
    worlds_dir = os.path.join(pkg_path, 'worlds')

    if rospy.has_param('~config_path'):
        config_path = rospy.get_param('~config_path')
    if rospy.has_param('~output_dir'):
        worlds_dir = rospy.get_param('~output_dir')

    if not os.path.exists(worlds_dir):
        os.makedirs(worlds_dir)

    rospy.loginfo("Building scenarios from: %s", config_path)
    files = builder.build_from_yaml(config_path, worlds_dir)
    rospy.loginfo("Generated %d world files", len(files))
