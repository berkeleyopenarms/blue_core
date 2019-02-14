from distutils.core import setup, Extension

extension_mod = Extension("_blue_interface", ["_blue_interface.cc", "../../src/BLDCControllerClient.cpp"])

setup(name="blue_interface", ext_modules=[extension_mod])
