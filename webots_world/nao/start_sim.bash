#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd $SCRIPT_DIR
TARGET_WORLD=$SCRIPT_DIR/worlds/nao_room.wbt
webots --stream=mjpeg --batch $TARGET_WORLD
