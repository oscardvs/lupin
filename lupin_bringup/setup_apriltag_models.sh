#!/bin/bash
# A one-time setup script to fix Gazebo's broken texture parser

echo "Linking Lupin models to Gazebo..."
mkdir -p ~/.gazebo/models
rm -rf ~/.gazebo/models/apriltags
ln -s $(pwd)/src/lupin/lupin_bringup/models/apriltags ~/.gazebo/models/apriltags

echo "Done! Gazebo will now see the AprilTags."
