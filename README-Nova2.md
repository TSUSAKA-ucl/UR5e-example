# DOBOT 6-Axis ROS2 V4 Setup Instructions

Only the parts that differ from the UR5e case are described.

## 1. Install DOBOT 6-Axis ROS2 V4 Package
```
mkdir -p ./ros2/src
cd ./ros2/src
git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nova2_moveit dobot_rviz
. ./install/setup.bash
cd ../
```

## 4. Generate ROS URDF File
```
ros2 run xacro xacro `ros2 pkg prefix nova2_moveit`/share/nova2_moveit/config/nova2_robot.urdf.xacro > nova2_robot.urdf
```

## 12. symlink or copy the mesh files to the meshes folder
```
Nova2Dir=`ros2 pkg prefix dobot_rviz`/share/dobot_rviz
```
```
mkdir -p meshes
cd meshes
for path in "${Meshes[@]}"; do
  path=`echo $path | sed 's|^package://dobot_rviz/||'`
  ln -s "$Nova2Dir/$path" .
done
```

## 1. calculate the bounding box of the DOBOT 6-Axis
```
../s/boundingBox.sh *.STL
```
copy CONVUM_SGE-M5-N's bounding box files to the meshes folder

## 4.Check if the generated collider is acceptable
```
for f in *.STL; do meshlab $f ${f%.*}.bbox.stl; done
```
## 6.edit shapeList.json
```json
[
  [ "table.ply", "base_link.bbox.ply" ],
  [ "Link1.bbox.ply" ],
  [ "Link2.bbox.ply" ],
  [ "Link3.bbox.ply" ],
  [ "Link4.bbox.ply" ],
  [ "Link5.bbox.ply" ],
  [ "Link6.bbox.ply" ],
  [ "CONVUM_SGE-M5-N-body-m.bbox.ply",
    "CONVUM_SGE-M5-N-suction-m.bbox.ply"
  ]
]
```
You can use the same `testPairs.json` as the UR5e case.

## 11. add tool's collider to the end link
```
node ./addToolColliders.js update.json Link6 CONVUM_SGE-M5-N-body-m.bbox.gltf CONVUM_SGE-M5-N-suction-m.bbox.gltf
```

