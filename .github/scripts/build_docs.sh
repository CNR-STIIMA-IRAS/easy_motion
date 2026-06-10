#!/usr/bin/env bash
set -e

rm -rf docs_build docs_output docs
mkdir -p docs

for pkg in \
  easy_motion \
  easy_motion_cpp \
  easy_motion_behavior_tree \
  easy_motion_msgs
do
  rosdoc2 build --package-path "${pkg}"
done

cp -r docs_output/* docs/
cp .github/pages/index_template.html docs/index.html
touch docs/.nojekyll