#!/bin/bash
ThisDir=`dirname "$0"`
PpCmd="$ThisDir"/a/json-pretty-compact.sh
CpCmd=/usr/bin/cp
if [ $# -lt 2 ]; then
    echo usage: "$0" '<src_dir> <dest_dir>' 1>&2
    exit 1
fi
SrcDir="$1"
DstDir="$2"
[ ! -d "$SrcDir" ] || [ ! -d "$DstDir" ] && exit 2

#
#### urdf.json
"$PpCmd" "$SrcDir"/urdfmap_cut.json -o "$DstDir"/urdf.json -c 90
#
#### update.json
if [ -f "$SrcDir"/update.json ]; then
    "$PpCmd" "$SrcDir"/update.json -o "$DstDir"/update.json -c 90
elif [ -f "$SrcDir"/update-with-tools-collider.json ]; then
    "$PpCmd" "$SrcDir"/update-with-tools-collider.json \
	     -o "$DstDir"/update.json -c 90
elif [ -f "$SrcDir"/update-stub.json ]; then
    "$PpCmd" "$SrcDir"/update-stub.json -o "$DstDir"/update.json -c 90
else
    echo 'no update-stub.json file' 1>&2
    exit 1
fi
#
#### linkmap.json
"$CpCmd" "$SrcDir"/linkmap.json "$DstDir"/
#
#### shapes.json
if [ -f "$SrcDir"/shapes.json ]; then
    "$CpCmd" "$SrcDir"/shapes.json "$DstDir"/
else
    if [ -f "$SrcDir"/meshes/output.json ]; then
	ln -s "$SrcDir"/meshes/output.json "$SrcDir"/shapes.json
	"$CpCmd" "$SrcDir"/shapes.json "$DstDir"/
    fi
fi
#
#### testPairs.json
if [ -f "$SrcDir"/testPairs.json ]
then "$CpCmd" "$SrcDir"/testPairs.json "$DstDir"/
fi
#
#### glTF
"$CpCmd" "$SrcDir"/meshes/out/* "$DstDir"/
