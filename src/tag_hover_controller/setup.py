from setuptools import find_packages, setup

package_name = 'tag_hover_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/lockon_backbone.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mars',
    maintainer_email='mars@todo.todo',
    description='TODO: Package description, Hover and yaw-search controller backbone for MAVROS',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'hover_yaw_search = tag_hover_controller.hover_yaw_search:main',
        	'hover_yaw_search_v1 = tag_hover_controller.hover_yaw_search_v1:main',
			'apriltag_pnp_broadcaster = tag_hover_controller.apriltag_pnp_broadcaster:main',
			'apriltag_tf_broadcaster = tag_hover_controller.apriltag_tf_broadcaster:main', 
	],
    },
)
