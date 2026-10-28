import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'barrier_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch, urdf, models 폴더 빌드 경로 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 두 가지 노드를 모두 사용할 수 있도록 등록
            'yolo_barrier = barrier_sim.yolo_barrier:main',
            'yolo_ocr_barrier = barrier_sim.yolo_ocr_barrier:main'
        ],
    },
)