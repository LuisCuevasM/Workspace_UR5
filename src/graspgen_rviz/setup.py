from setuptools import setup

package_name = 'graspgen_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='GraspGen inference + RViz publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Tu nodo original
            'graspgen_rviz_infer = graspgen_rviz.graspgen_rviz_infer_node:main',
            # ¡Tu nuevo nodo de filtrado!
            'voxel_filter_node = graspgen_rviz.voxel_filter_node:main',
        ],
    },
)