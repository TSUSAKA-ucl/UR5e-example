# RT sciurs17

1. download ros2 packages
   ```
   cd ./ros2/src
   git submodule update --init --recursive
   ```
2. build ros2 workspace
   ```
   cd ../
   colcon build --symlink-install
   ```
3. confirm sciurs17 urdf file is loaded
   ```
   . install/setup.bash
   ros2 launch sciurs17_description sciurs17.launch.py
   ```
4. generate urdf file
   ```
   cd `git rev-parse --show-toplevel`
   xacro $(ros2 pkg prefix sciurs17_description)/share/sciurs17_description/urdf/sciurs17.urdf.xacro > ./sciurs17.urdf
   ```
5. split URDF tree
   ```
   ./a/splitUrdfTree.sh sciurus17.urdf
   ```
6. select chains for both arms
   * left arm: `chain_7.urdf` ( be sure to check the contens yourself )
	 ```
	 mkdir left_arm
	 cd left_arm
	 ../a/extract-joint-and-link-tag.sh ../chain_7.urdf
	 cd -
	 ```
   * right arm: `chain_4.urdf` ( be sure to check the contens yourself )
   	 ```
	 mkdir right_arm
	 cd right_arm
	 ../a/extract-joint-and-link-tag.sh ../chain_4.urdf
 	 cd -
	 ```
7. cut off the unnecessary parts from the base urdf file
   * left arm including the body
	 ```
 	 cd left_arm
	 ```
	 ```
	 ../a/cut-joint-map.sh urdfmap.json --from base_link --to l_link7
	 ../a/extract-joint-and-link-tag.sh ../chain_7.urdf -j urdfmap_cut.json
	 mkdir meshes && cd meshes && \
	 ../../resolve_ros2_paths.sh ../linkmap.json |\
	 grep -v '\/collision\/' |\
	 while read f; do ln -s "$f" .; done && \
	 for file in *.STL *.stl *.DAE *.dae;\
	 do ../../a/convert-to-gltf.sh "$file";\
	 done && cd ..
	 ```
	 ```
	 cd ..
	 ```
   * right arm
   	 ```
	 cd right_arm
 	 ```
	 ```
	 ../a/cut-joint-map.sh urdfmap.json --from body_link --to r_link7
	 ../a/extract-joint-and-link-tag.sh ../chain_4.urdf -j urdfmap_cut.json
	 mkdir meshes && cd meshes && \
	 ../../resolve_ros2_paths.sh ../linkmap.json |\
	 grep -v '\/collision\/' |\
	 while read f; do ln -s "$f" .; done && \
	 for file in *.STL *.stl *.DAE *.dae;\
	 do ../../a/convert-to-gltf.sh "$file";\
	 done && cd ..
 	 ```
	 ```
	 cd ..
	 ```

8. create a directory for the combined npm package
   ```
   mkdir sciurs17
   cd sciurs17
   pnpm init
   mkdir public
   cd ..
   ```
9. create gripper urdf files and assets
   `left_gripperB`'s assets are created from `chain_7.urdf`.
   `chain_6.urdf` for `left_gripperA`, `chain_4.urdf` for `right_gripperA` and `chain_3.urdf` for `right_gripperB`. Only the example 
   for `left_gripperB` is shown here.
   ```
   mkdir left_gripperB
   cd left_gripperB/
   ../a/extract-joint-and-link-tag.sh ../chain_7.urdf 
   ../a/cut-joint-map.sh urdfmap.json --from l_link7 
   ../a/extract-joint-and-link-tag.sh ../chain_7.urdf -j urdfmap_cut.json 
   mkdir meshes
   cd meshes/
   ../../resolve_ros2_paths.sh ../linkmap.json  |\
   grep -v '\/collision\/' |\
   while read f; do ln -s "$f" .; done &&\
   for file in *.STL *.stl *.DAE *.dae;\
   do ../../a/convert-to-gltf.sh "$file"; done && cd ..
   cd ..
   ```
9. copy the two arm directories into the package directory
   ```
   mkdir sciurs17/public/sciurus17left/
   ./copy-assets.sh left_arm sciurs17/public/sciurus17left/
   mkdir sciurs17/public/sciurus17right/
   ./copy-assets.sh right_arm sciurs17/public/sciurus17right/
   (cd sciurs17/public; mkdir sciurus17lgripperA sciurus17lgripperB sciurus17rgripperA sciurus17rgripperB)
   ./copy-assets.sh right_gripperA/ sciurus17/public/sciurus17rgripperA/
   ./copy-assets.sh right_gripperB/ sciurus17/public/sciurus17rgripperB
   ./copy-assets.sh left_gripperB/ sciurus17/public/sciurus17lgripperB/
   ./copy-assets.sh left_gripperA/ sciurus17/public/sciurus17lgripperA/
    ```
10. modify package.json
   ```
   cd sciurs17
   ../insert_files.sh 
   vi package.json
   ```
   set "version".
   "name", "author" and "files" are set by `../insert_files.sh`.
11. publish to npm
   ```
   pnpm publish --access public --no-git-checks
   ```
