from setuptools import setup,find_packages
setup(name='ltaom_pipeline',version='0.1.0',packages=find_packages(),data_files=[('share/ament_index/resource_index/packages',['resource/ltaom_pipeline']),('share/ltaom_pipeline',['package.xml','LICENSE'])],install_requires=['setuptools'],entry_points={'console_scripts':['live_pipeline = ltaom_pipeline.live:main']})
