import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'image_to_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Skill YAML 文件安装到 share 目录
        (os.path.join('share', package_name, 'skills'),
            glob(os.path.join('image_to_llm', 'skills', '*.yaml'))),
    ],
    package_data={
        'image_to_llm': ['skills/*.yaml'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuko',
    maintainer_email='dc22897@um.edu.mo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_to_llm_node = image_to_llm.image_to_llm_node:main',
            'image_conversion = image_to_llm.image_conversion:main',
        ],
    },
)
