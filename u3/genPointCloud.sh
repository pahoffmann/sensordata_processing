#!/bin/sh

MAX_DISPARITY=256
BLOCK_SIZE=9
POINT_CLOUD_FILE="test.txt"
COLOR="--color"
INTRINSICS="intrinsics.yml"
EXTRINSICS="extrinsics.yml"

echo "Params: MAX_DISPARITY=$MAX_DISPARITY, BLOCK_SIZE=$BLOCK_SIZE, POINT_CLOUD_FILE=$POINT_CLOUD_FILE, 
    INTRINSICS=$INTRINSICS, EXTRINSICS=$EXTRINSICS"