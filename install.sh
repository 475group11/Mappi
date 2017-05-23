#!/bin/bash

# Installs the packages in the workspace located in the current directory

DEST='/opt/ros/kinetic'
SRC='install'

cp -rf $SRC/include/* $DEST/include
cp -rf $SRC/lib/* $DEST/lib
cp -rf $SRC/share/* $DEST/share
