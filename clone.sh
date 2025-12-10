#!/bin/bash
git clone --filter=blob:none --no-checkout \
    https://github.com/TSUSAKA-ucl/gjk_worker.git
cd gjk_worker/
git sparse-checkout set scripts nodejs
git checkout
cd -
ln -s gjk_worker/scripts/ s

git clone --filter=blob:none --no-checkout \
    https://github.com/TSUSAKA-ucl/robot-assets.git
cd robot-assets/
git sparse-checkout set scripts
git checkout
cd -
ln -s ./robot-assets/scripts/ a
