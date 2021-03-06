from setuptools import setup

package_name = 'bot_manager'

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
    maintainer='heidt',
    maintainer_email='nathanielheidt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_test = bot_manager.quick_test:main',
            'basic_multiagent_test = bot_manager.basic_multiagent_test:main',
        ],
    },
)
