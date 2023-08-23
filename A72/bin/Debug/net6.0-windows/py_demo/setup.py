"""simple setup script for jbd 4020 uLED API."""
import setuptools

with open("requirements.txt") as rf:
  requirements = [r for r in rf.readlines() if r and not r.startswith("#")]

setuptools.setup(name="jbd_uled_4020",
                 version="0.0.1",
                 packages=setuptools.find_packages(),
                 install_requires=requirements)
