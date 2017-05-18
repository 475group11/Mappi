#!/bin/bash

# Installs the mappi_inverse_laser_transform package
cd `dirname $0`

DEST='/opt/ros/kinetic'
SRC='install'

cp -rf $SRC/lib/mappi_inverse_laser_transform $DEST/lib
cp -rf $SRC/share/mappi_inverse_laser_transform $DEST/share
