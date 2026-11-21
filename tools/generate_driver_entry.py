"""Generate the standalone ROS component entry and ament component resource."""

import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('template', type=Path)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    replacements = {
        '@node@': 'livox_ros_driver2_node',
        '@executor@': 'SingleThreadedExecutor',
        '@library_name@': 'liblivox_ros_driver2.so',
        '@component@': 'livox_ros::DriverNode',
    }
    text = args.template.read_text()
    for before, after in replacements.items():
        text = text.replace(before, after)
    (args.out / 'livox_ros_driver2_node.cpp').write_text(text)
    resource = args.out / 'share/ament_index/resource_index/rclcpp_components/livox_ros_driver2'
    resource.parent.mkdir(parents=True, exist_ok=True)
    resource.write_text('livox_ros::DriverNode;lib/liblivox_ros_driver2.so\n')


if __name__ == '__main__':
    main()
