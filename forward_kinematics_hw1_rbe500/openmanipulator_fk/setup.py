from setuptools import setup

setup(
    name='openmanipulator_fk', version='0.1.0',
    packages=['openmanipulator_fk'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/openmanipulator_fk']),
                ('share/openmanipulator_fk', ['package.xml', 'README.md'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Student', maintainer_email='student@example.com',
    description='Forward kinematics using the supplied standard DH table',
    license='MIT',
    entry_points={'console_scripts': ['fk_node = openmanipulator_fk.fk_node:main']},
)
