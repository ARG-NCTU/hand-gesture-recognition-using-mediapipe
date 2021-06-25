#! /bin/bash

# echo "password: $2"
git checkout master
BRANCH=master
if [ ! -z "$1" ]; then
    echo "pull branch: $1"
    BRANCH=$1
fi

echo "--------------------------------------------------------------------------------------------------------"
echo "---------------------------pull hand-gesture-recognition-using-mediapipe--------------------------------"
echo "--------------------------------------------------------------------------------------------------------"
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in hand-gesture-recognition-using-mediapipe. Aborting"
   return 1
fi

cd ~/hand-gesture-recognition-using-mediapipe
return 0
