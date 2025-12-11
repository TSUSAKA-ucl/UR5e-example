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
   this will generate `urdfmap.json` (or `urdfsorted.json`), `linkmap.json`,
   and `update-stub.json` files.
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

Now you can use the created files(`urdf.json`, `linkmap.json`, `update.json`)
and glTF mesh files  with `robot-loader` and `ik-worker`.

## Creating colliders information and their visualization information

This section describes how to create the `shapes.json` file that defines
the colliders used in `cd-worker`, and the visualization information.

`shapes.json` file defines only vertices of convex shapes and their composition.

There are two ways to create the `shapes.json` file. One is 
from bounding boxes of link meshes, and the other is more complex, where
the link meshes are decimated and decomposed into convex parts.

For the former way, please refer to [`HowToMakeShapes_json_file2.md`(in Japanese)](https://github.com/TSUSAKA-ucl/gjk_worker/blob/main/docs/HowToMakeShapes_json_file2.md). For the latter way, please refer to [`HowToMakeShapes_json_file3.md`(in Japanese)](https://github.com/TSUSAKA-ucl/gjk_worker/blob/main/docs/HowToMakeShapes_json_file3.md).

UR robot has the links with simple shapes, so the former way is usually sufficient.

1. UR5e's description includes DAE files for visualization and STL files for collision.
   You can use the DAE files to create colliders from bounding boxes. DAE files are easier
   to understand their shapes than STL files.
   So first, calculate the bounding boxes from DAE files.
   ```
   cd meshes/
   ../s/boundingBox.sh *.dae
   ```
   this creates `.bbx` files for each DAE file.
2. edit bounding box files if necessary to make them tighter.
3. create collider files: STL files, PLY files and glTF files for colliders
   from bounding box files
   ```
   ../s/createBboxAll.sh
   ```
4. Check if the generated collider is acceptable. `meshlab` has problems when
   opening DAE files directly from the command line, so you may need to create
   a project file and open it.
   ```
   for dae in *.dae
   do sed -e "s/TEMPLATE/${dae%.*}/g" template.mlp > "${dae%.*}.mlp"
      meshlab "${dae%.*}".mlp
   done
   ```
   If ROS's visualization uses STL files instead of DAE files, 
   meshlab's project file isn't necessary. You can open two STL files directly.
5. If all colliders are acceptable, create `shapeList.json` file which lists up
   the collider files for each link.
   ```
   ../s/create_shapelist.sh *.bbox.ply > shapeList.json
   ```
6. edit `shapeList.json` to sort the links according to the joint order in `urdf.json`.
   In addition, add tools fixed to the end-link and base plate colliders if necessary.
   This will result in a file like below:
   ```
   [
     [ "table.ply", "base.bbox.ply" ],
     [ "shoulder.bbox.ply" ],
     [ "upperarm.bbox.ply" ],
     [ "forearm.bbox.ply" ],
     [ "wrist1.bbox.ply" ],
     [ "wrist2.bbox.ply" ],
     [ "wrist3.bbox.ply" ],
     [ "CONVUM_SGE-M5-N-body-m.bbox.ply",
       "CONVUM_SGE-M5-N-suction-m.bbox.ply"
     ]
   ]
   ```
7. create `shapes.json` file from `shapeList.json`
   ```
   ../s/ply_loader.js shapeList.json
   cp -p outout.json shapes.json
   ```
   Now you have `shapes.json` file defining colliders for UR5e robot.
8. write `testPairs.json` file for testing collision detection between links.
   This file is handwritten because it is difficult to determine which link pairs
   should be tested for collision automatically. But for many 6-DOF serial robots,
   following pairs are sufficient:
   ```json
   [
     [0,2],[0,3],[0,4],[0,5],[0,6],[0,7],
     [1,3],[1,4],[1,5],[1,6],[1,7],
     [2,4],[2,5],[2,6],[2,7],
     [3,5],[3,6],[3,7]
   ]
   ```
9. finally, move `shapes.json` and `testPairs.json` to `public/ur5e/` folder
   or any other desired folder. These two are all that the `cd-worker` needs.
   But if you want to visualize the colliders in `robot-loader`, you also need
   glTF files for colliders.
10. create glTF files for colliders from the collider STL files
    ```
    cd meshes/
    for file in *.bbox.stl; do
      ../a/convert-to-gltf.sh "$file"
    done
    ```
    STLs have no color information, so add a color and opacity to the
    glTF files using `set-gltf-color.mjs` tool
    ```
	cd out
    for f in *.bbox.gltf; do
      node ../../s/set-gltf-color.mjs "$f" --color '#ff0000' --opacity 0.2
    done
    cd ../..
    ```
	move the generated glTF files and bin files in `meshes/out/` folder 
	to `public/ur5e/` folder or any other desired folder.	
11. modify `update.json` to draw the tool's collider if necessary.
	```
	node ./addToolColliders.js
	```
	or 
	```
	node ./addToolColliders.js update.json wrist_3_link CONVUM_SGE-M5-N-body-m.bbox.gltf CONVUM_SGE-M5-N-suction-m.bbox.gltf
	```
	These commands create the same `update_with_tool.json` file.
	Then overwrite `public/ur5e/update.json` file with it.  
	**NOTE:**  
	Tools are not defined in `linkmap.json`, so they are written into
	`shapes.json` and `update.json` as shapes **in the coordinate system
	of the LINK** to which they are attached, **not in the glTF visual's origin**
	of the link.

Now you can use the created `shapes.json`, `testPairs.json` files and
the collider glTF files with `cd-worker` and `robot-loader`.

If the ROS robot description package does not include DAE files for visualization
but for visualization only STL files, you can add colors to the collider glTF files
using `set-gltf-color.mjs` tool as described above.
