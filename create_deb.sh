#!/bin/bash

bloom-generate rosdebian --os-name ubuntu --ros-distro foxy
fakeroot debian/rules binary
