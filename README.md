# JetBot-from-Scratch

This project will bring the JetBot functionality given by [jetbot.org](http://jetbot.org/master/), away from jupyter-notebook to pure python form. It will also introduce new functionality in the future as well. 

## Ingredients used

* Waveshare JetBot kit [link](https://www.waveshare.com/catalog/product/view/id/3755)
* Original Jetson Nano Image [link](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

## Software setup

* Python Miniconda - `conda create -n <env_name> python=3.7 pip`
* Read Gamepad inputs - [inputs](https://github.com/zeth/inputs)
* Install python packages - `pip install -r requirements.txt`

## Usage

* `jetbot_fs` - contains all functions related to controlling jetbot
* `python basic/robot_basic.py` - Check if the robot wheels are working
* `python basic/gamepad_basic.py` - Check if the Gamepad is working
* `python basic/gamepad_jetbot.py` - Move jetbot using gamepad
    - Y - Go Forward
    - A - Go Backward
    - X - Turn Left
    - B - Turn Right

## To Do

* Make function to direction save gamepad key map to yml file.
