#!/bin/bash

git config --global user.name "DiaboloKiat"
git config --global user.email "DiaboloKiat@gmail.com"

git status
git checkout main
echo "Enter your message"
read message
BRANCH=main


# push main
echo "--------------------------------------------------------------------------------------------------------"
echo "---------------------------push hand-gesture-recognition-using-mediapipe--------------------------------"
echo "--------------------------------------------------------------------------------------------------------"
cd ~/hand-gesture-recognition-using-mediapipe/
git add -A
git commit -m "${message}"
git push
