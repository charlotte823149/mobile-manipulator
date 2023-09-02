from setuptools import setup

setup(
    name='human_and_pose',
    version='1.0.0',
    packages=['human_and_pose'],
    scripts=['src/human_and_pose_node.py'],
    install_requires=['rospy', 'std_msgs'],
    package_dir={'': 'src'}
)

