from setuptools import setup

package_name: str = 'dummy_total_publisher'
node_package_name: str = package_name + '.node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, node_package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reidlo',
    maintainer_email='201840103@email.daelim.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_total_publisher = dummy_total_publisher.main:main'
        ],
    },
)
