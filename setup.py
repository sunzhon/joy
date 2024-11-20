from setuptools import setup

package_name = 'joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='suntao.hn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    data_files =[
    (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    entry_points={
        'console_scripts': [
            'joy_remap_node = joy.joy_remap:main'
        ],
    },
)
