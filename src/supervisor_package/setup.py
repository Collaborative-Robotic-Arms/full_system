from setuptools import setup

package_name = 'supervisor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=['supervisor_logic'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marwan',
    maintainer_email='2100771@eng.asu.edu.eg',
    description='Supervisor node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'supervisor = supervisor_logic.supervisor:main',
        ],
    },
)
