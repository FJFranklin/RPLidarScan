This uses (via pip install):
* RPLidarC1 from https://github.com/dsaadatmandi/rplidarc1
* DearPyGui from https://github.com/hoffstadt/DearPyGui (availability depends on platform)

If using python 3.10 (as is the case on the Jetson Orin Nano), you also need (via pip install):
* taskgroup from https://github.com/graingert/taskgroup

For the Jetson Orin Nano, you can build DearPyGui from source; instructions here:
* https://github.com/hoffstadt/DearPyGui/wiki/Local-Wheel
but for the Jetson:
```bash
git clone --recursive https://github.com/hoffstadt/DearPyGui
cd DearPyGui
python3 -m setup bdist_wheel --plat-name manylinux_2_35_aarch64 --dist-dir dist
pip install dist/dearpygui-2.1.1-cp310-cp310-manylinux_2_35_aarch64.whl 
```
