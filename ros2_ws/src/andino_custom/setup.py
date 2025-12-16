from setuptools import setup

package_name = 'andino_custom'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/andino_gz.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Custom Andino launch + config for Nav2',
    license='Apache-2.0',
)
