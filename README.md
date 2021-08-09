- Now on the Kiat readme <br>
[[Origin Author readme](https://github.com/DiaboloKiat/hand-gesture-recognition-using-mediapipe/blob/main/origin/README.md)/**Kiat readme**]

- This repo is forked from [Kazuhito00/hand-gesture-recognition-using-mediapipe](https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe).

- In order to make the repo look neat, I put all the original author's files in the [origin](https://github.com/DiaboloKiat/hand-gesture-recognition-using-mediapipe/tree/main/origin) folder.

- This repo will have a ros-package named [hand_gesture](https://github.com/DiaboloKiat/hand-gesture-recognition-using-mediapipe/tree/main/hand_gesture), which is used on a robot named Seadrone.

- Please feel free to contact me if there is anything inappropriate.

<h1 align="center"> Hand Gesture Recognition Using Mediapipe </h1>

## How to run
- Run mediapipe hand pose landmarks (camera: D435)
```
$ roslaunch hand_gesture mediapipe.launch seadrone_cameara:=false
```

- Run hand gesture dataset (camera: D435)
```
$ roslaunch hand_gesture hand_gesture.launch seadrone_cameara:=false
```

## Hand pose landmarks
<h4 align="center"> <img src="https://github.com/DiaboloKiat/hand-gesture-recognition-using-mediapipe/blob/main/images/hand_landmarks.png"/> </h4>

