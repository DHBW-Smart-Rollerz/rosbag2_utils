# ROS2 Rosbag Utils Package

[![Build Test](https://github.com/DHBW-Smart-Rollerz/rosbag2_utils/actions/workflows/build-test.yaml/badge.svg)](https://github.com/DHBW-Smart-Rollerz/rosbag2_utils/actions/workflows/build-test.yaml)

This repository contains the rosbag2_utils package with a collection of helpful rosbag nodes.

## Installation

1. Clone the package into the `smarty_workspace/src` directory:
   ```bash
   git clone <repository-url> smarty_workspace/src
   ```
2. Install the required Python dependencies using the `requirements.txt` file:
   ```bash
   pip install -r requirements.txt
   ```
3. Please set the following ENV variable in your .bashrc/.zshrc if not already done:
   ```bash
   PYTHON_EXECUTABLE="/home/$USER/.pyenv/versions/default/bin/python3" # Change this to the python3 executable path of your pyenv
   ```

## Usage

This repository is a collection of standalone nodes.

### RosbagToImages

This node provides functionality for exporting images from a ROS2 bag. It offers optional features to undistort the images and apply the bird's-eye view transformation, utilizing the [camera_preprocessing](https://github.com/DHBW-Smart-Rollerz/camera_preprocessing) package.  
**IMPORTANT!** Proper results require the camera_preprocessing package ithe be calibrated with the same chessboard and camera/lens used during the recording of the ROS bag. Refer to the [camera_preprocessing](https://github.com/DHBW-Smart-Rollerz/camera_preprocessing) documentation for detailed calibration instructions.

Additionally, the interval in which the frames/images are exported can be specified in case not every image is required. The bags are mostly captured in 30fps and therefore it is recommended to use only every 5th frame.

It is also possible to control the number of image extractions by specifying a frame interval if not every image is needed. The rosbags are mostly captured in 30fps and therefore it is recommended to use only every 5th image.

To launch the node, use the following command from the folder the rosbags are saved in:
```bash
ros2 launch rosbag2_utils rosbag_to_images.launch.py
```
The node will find all rosbags recursively and export them into seperate folders. If required, the input and output directory can be changed in the config file.

## Structure

- `config/`: All configurations (most of the time yaml files)
- `launch/`: Contains all launch files. Launch files can start multiple nodes with yaml-configurations
- `resource/`: Contains the package name (required to build with colcon)
- `rosbag2_utils`: Contains all nodes and sources for the ros package
- `test/`: Contains all tests
- `package.xml`: Contains metadata about the package
- `setup.py`: Used for Python package configuration
- `setup.cfg`: Additional configuration for the package
- `requirements.txt`: Python dependencies

## Contributing

Thank you for considering contributing to this repository! Here are a few guidelines to get you started:

1. Fork the repository and clone it locally.
2. Create a new branch for your contribution.
3. Make your changes and ensure they are properly tested.
4. Commit your changes and push them to your forked repository.
5. Submit a pull request with a clear description of your changes.

We appreciate your contributions and look forward to reviewing them!

## License

This repository is licensed under the MIT license. See [LICENSE](LICENSE) for details.
