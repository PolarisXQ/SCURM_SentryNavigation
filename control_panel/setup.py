from setuptools import setup

package_name = 'control_panel'

setup(
    name=package_name,
    version='2.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Polaris Xia',
    author_email='1367256900@qq.com',
    maintainer='Polaris Xia',
    maintainer_email='1367256900@qq.com',
    description=(
        "no description"
    ),
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_panel = control_panel.gui:main',
        ],
    },
)
