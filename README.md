# making Universal Robots Description for `robot-loader`

## Creating joint information and link visualization information used in `robot-loader` and `ik-worker`

This section describes how to create the `urdf.json`, `linkmap.json`,
`update.json` files and the mesh files needed to use Universal Robots
(UR) robots with `robot-loader` and `ik-worker`.

0. create your ros2 workspace if you don't have one
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   ```
1. clone UR git repository into ros2 workspace `src` folder
   ```
   cd src
   git clone  https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
   cd -
   ```
2. build ros2 workspace
   ```
   colcon build
   . install/setup.bash
   ```
3. launch `view_ur.launch.py` to verify the UR robot's configuration (OPTIONAL)
   ```
   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
   ```
   you can move the robot in RViz using Joint State Publisher.
   it is better to check the zero position of the robot joints
   and the direction of each joint movement.
4. create ROS URDF file using xacro
   ```
   ros2 run xacro xacro `ros2 pkg prefix ur_description`/share/ur_description/urdf/ur.urdf.xacro name:=ur ur_type:=ur5e > ur5e_robot.urdf
   ```
5. return to this working directory, and clone scripts repository if you haven't done yet
   ```
   cd <this_working_directory>
   ./clone.sh
   ```
5. split the URDF tree into separate chains using `splitUrdfTree.sh` script
   ```
   ./a/splitUrdfTree.sh ur5e_robot.urdf
   ```
6. Select the `chain_X.urdf` file you need by examining the corresponding `chain_X.json`(summary) file.
   `ls -l chain_*.json` to see the size of each chain file, and `view chain_1.json` to see 
   the details of longest chain. Usually, the longest chain corresponds to the main robot body.
   If it includes the root link and the end-effector link you need, then select it.
7. generate the joint map file(`urdf.json`), the link map
   file(`links.json`), and the modifier file(`update.json`) from the
   selected `chain_X.urdf` file using `extract-joint-and-link-tag.sh`
   example, if `chain_1.urdf` is selected, run
   ```
   ./a/extract-joint-and-link-tag.sh chain_1.urdf
   ```
   this will generate `urdfmap.json`, `linkmap.json`, and
   `update-stub.json` files.
8. cut off the unnecessary parts of the joint map from the
   `urdfmap.json` using `./a/cut-joint-map.sh` script.
   for example, if `chain_1.urdf` is selected, run
   ```
   ./a/cut-joint-map.sh urdfmap.json --from base_link_inertia --to flange
   ```
   `--from` and `--to` options specify the root **LINK** and
   the end-effector **link** of the selected chain.
   this will generate `urdfmap_cut.json` file.
9. regenerate the link map file using the cut joint map file
   ```
   ./a/extract-joint-and-link-tag.sh chain_1.urdf -j urdfmap_cut.json
   ```
   this is not strictly necessary, but it will generate
   smaller `linkmap.json` and `update-stub.json` files,
   and make easier to edit `update-stub.json` file.
   If you don't want to visualize colliders shapes, you can use `-n` option.
10. finding the shape data needed for visualization and collision from the
    link map file
    ```
    Meshes=(`grep filename linkmap.json |sed 's/^\s*//'|sort -u | sed -e 's/^[^:]*:\s*//' -e 's/"//g'`)
    for path in "${Meshes[@]}"; do echo $path; done
    ```
    this will list up all the mesh files' ROS2 paths used in the selected chain.
    if you had built the robot's description package in ros2 workspace, 
    the paths will be findable under the `install` folder. If not, you can usually
    find them in the source tree you cloned in step 1.
11. symlink or copy the mesh files to the `meshes` folder.
	for example,
	```
	mkdir -p meshes
	cd meshes
	```
    ```
    URDir=`ros2 pkg prefix ur_description`/share/ur_description
    ```
    or you can use the source tree path like below:
    ```
    URDir=../Universal_Robots_ROS2_Description
    ```
	```
	for path in "${Meshes[@]}"; do
	  path=`echo $path | sed 's|^package://ur_description/||'`
	  ln -s $URDir/$path .
	done
	```
12. convert meshes to glTF format using `convert-to-gltf.sh` tool
	```
	for file in *.STL *.stl *.DAE *.dae; do
	  ../a/convert-to-gltf.sh "$file"
	done
	cd ..
	```
	this will generate glTF files(`.gltf` and `.bin`) for each mesh file
	under `out` folder.
13. make a compact `update.json` using `json-pretty-compact.sh` tool
	```
	./a/json-pretty-compact.sh update-stub.json -o update.json -c 90
	```
	and edit `update.json` if necessary.

14. finally, rename `urdfmap_cut.json` to `urdf.json` and move
	`urdf.json`, `linkmap.json`, `update.json` and files in `meshes/out/` folder
	to `public/ur5e/` folder or any other desired folder.

Now you can use the created files with `robot-loader` and `ik-worker`.

## Creating colliders information and their visualization information

This section describes how to create the `shapes.json` file that defines
the colliders used in `cd-worker`, and the visualization information.

`shapes.json` file defines only vertices of convex shapes and their composition.

There are two ways to create the `shapes.json` file. One is 
from bounding boxes of link meshes, and the other is more complex, where
the link meshes are decimated and decomposed into convex parts.

For the former way, please refer to [`HowToMakeShapes_json_file2.md`(in Japanese)](https://github.com/TSUSAKA-ucl/gjk_worker/blob/main/docs/HowToMakeShapes_json_file2.md). For the latter way, please refer to [`HowToMakeShapes_json_file3.md`(in Japanese)](https://github.com/TSUSAKA-ucl/gjk_worker/blob/main/docs/HowToMakeShapes_json_file3.md).

UR robots has the links with simple shapes, so the former way is usually sufficient.
