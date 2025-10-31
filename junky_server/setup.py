from setuptools import setup

package_name = 'junky_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/server_launch.py']),
        ('share/' + package_name + '/templates', ['junky_server/static/css/style.css']),
        ('share/' + package_name + '/templates', ['junky_server/static/js/script.js']),
        ('share/' + package_name + '/templates', ['junky_server/templates/base.html']),
        ('share/' + package_name + '/templates', ['junky_server/templates/index.html']),
        ('share/' + package_name + '/templates', ['junky_server/templates/control_pc.html']),
        ('share/' + package_name + '/templates', ['junky_server/templates/control_phone.html']),
        ('share/' + package_name + '/templates', ['junky_server/templates/sensors.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Robot Junky control and server package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = junky_server.web_server:main',
            'control_node = junky_server.control_node:main',
            'camera_node = junky_server.camera_node:main',
        ],
    },
)
