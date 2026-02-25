from setuptools import find_packages, setup

package_name = 'ml_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource',
            ['resource/puppy.jpeg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='eduardo.laruta@gmail.com',
    description='Object detection inference node using PyTorch and torchvision',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'detector_node = ml_inference.detector_node:main',
            'test_image_publisher = ml_inference.test_image_publisher:main',
        ],
    },
)
