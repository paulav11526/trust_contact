from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trust_contact'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 1. Grab only the files directly in models (scene.xml, etc.)
        (os.path.join('share', package_name, 'models'), 
         [f for f in glob('trust_contact/models/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'pointcloud'), 
         [f for f in glob('trust_contact/pointcloud/*') if os.path.isfile(f)]),
        # 2. Grab only the files inside assets (STLs, OBJs, etc.)
        (os.path.join('share', package_name, 'models/assets'), 
         [f for f in glob('trust_contact/models/assets/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools', 'messages'],
    zip_safe=True,
    maintainer='paulav',
    maintainer_email='paulita-villamil@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simulation = trust_contact.simulation:main',
            'residual_node = trust_contact.residual_node:main',
            'fsm_node = trust_contact.fsm_node:main',
            'contact_classifier_node = trust_contact.contact_classifier_node:main',
            'speed_predictor_node = trust_contact.speed_predictor_node:main'

        ],
    },
)
