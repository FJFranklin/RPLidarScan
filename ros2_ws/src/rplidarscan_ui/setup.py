from setuptools import find_packages, setup

package_name = 'rplidarscan_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/fonts',     ['rplidarscan_ui/fonts/NotoSerifCJKjp-Medium.otf']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/arrow-left-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/arrow-left-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/clock-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/clock-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/home-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/home-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/pause-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/pause-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/play-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/play-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/settings-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/settings-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/system-shut-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/system-shut-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/zoom-in-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/zoom-in-80x80.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/zoom-out-100x100.png']),
        ('share/' + package_name + '/icons/png', ['rplidarscan_ui/icons/png/zoom-out-80x80.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francis James Franklin',
    maintainer_email='fjf@alinameridon.com',
    description='A simple user interface for the SlamTec RPLidar C1 intended for small touchscreens.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui = rplidarscan_ui.ui:main'
        ],
    },
)
