import glob
import os

import cv2
import cv_bridge
import numpy as np
import rclpy
import rclpy.node
import rclpy.wait_for_message
import rosbag2_py
import sensor_msgs.msg as sensor_msgs
from rclpy.logging import get_logger
from rclpy.serialization import deserialize_message

from camera_preprocessing.transformation import (
    birds_eyed_view,
    coordinate_transform,
    distortion,
)


class RosbagToImagesNode(rclpy.node.Node):
    """Converts ros2 bags to Images."""

    def __init__(self):
        """Initialize the RosbagToImagesNode."""
        super().__init__("rosbag_to_images_node")

        self.load_ros_params()

        self.logger = get_logger("rosbag_to_images")

        self.cv_bridge = cv_bridge.CvBridge()
        self.coord_transform = coordinate_transform.CoordinateTransform()
        self.distortion = distortion.Distortion(self.coord_transform._calib)
        self.bev = birds_eyed_view.Birdseye(
            self.coord_transform._calib, self.distortion
        )

        self.logger.info("RosbagToImages Node initialized")

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        # All command line arguments from the launch file and parameters from the
        # yaml config file must be declared here with a default value.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_dir", "./"),
                ("output_dir", "./rosbag_images/"),
                ("image_topic", "/camera/image_raw"),
                ("frame_interval", 5),
                ("undistort", False),
                ("convert_to_bev", False),
                ("print_status_interval", 100),
                ("debug", False),
            ],
        )

        # Get parameters from the ROS parameter server into a local variable
        self.input_dir = self.get_parameter("input_dir").value
        self.output_dir = self.get_parameter("output_dir").value
        self.image_topic = self.get_parameter("image_topic").value
        self.frame_interval = self.get_parameter("frame_interval").value
        self.undistort = self.get_parameter("undistort").value
        self.convert_to_bev = self.get_parameter("convert_to_bev").value
        self.print_status_interval = self.get_parameter("print_status_interval").value
        self.debug = self.get_parameter("debug").value

    def find_rosbags(self, dir: str) -> list[str]:
        """
        Finds ROS2 bag directories within a given directory.

        Arguments:
            dir: The directory path to search for (nested) ROS2 bags.

        Returns:
            A list of paths to the ROS2 bag directories.
        """

        if not os.path.isdir(dir):
            self.logger.error(f"{dir} is not a directory.")
            return []

        # Find .db3 files in the directory and return its parent directory as the
        # rosbag directory.
        bag_files = glob.glob(dir + "/**/*.db3", recursive=True)
        rosbag_dirs = list(set([os.path.dirname(bag_file) for bag_file in bag_files]))

        return rosbag_dirs

    def load_rosbag(self, rosbag_dir: str) -> rosbag2_py.SequentialReader:
        """
        Loads a ROS2 bag from the given directory.

        Arguments:
            rosbag_dir: The directory containing the ROS2 bag files.

        Returns:
            A SequentialReader object for reading messages from the bag.
        """
        storage_options = rosbag2_py.StorageOptions(
            uri=rosbag_dir,
            storage_id="sqlite3",
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        storage_filter = rosbag2_py.StorageFilter(topics=[self.image_topic])
        reader.set_filter(storage_filter)

        return reader

    def get_image_from_msg(self, image_msg, msg_tpye) -> np.ndarray:
        # Deserialize the message
        if msg_tpye == "sensor_msgs/msg/Image":
            image_msg = deserialize_message(image_msg, sensor_msgs.Image)
        elif msg_tpye == "sensor_msgs/msg/CompressedImage":
            image_msg = deserialize_message(image_msg, sensor_msgs.CompressedImage)
        else:
            self.logger.warn(f"Unsupported topic type: {msg_tpye}")
            return

        # Convert the image message to an opencv image
        image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        return image

    def create_output_dir(self, dir: str):
        """
        Creates the output directory if it does not exist.

        Arguments:
            dir: The directory path to create.
        """
        if not os.path.exists(dir):
            try:
                os.makedirs(dir)
                self.logger.info(f"Created output directory: {dir}")
            except OSError:
                self.logger.error(f"Could not create output directory: {dir}.")
                rclpy.shutdown()
                return

    def rosbag_to_images(self):
        """
        Exports images from the rosbag to the output directory.
        """

        rosbags = self.find_rosbags(self.input_dir)
        self.logger.info(f"Found {len(rosbags)} rosbags in {self.input_dir}")

        for rosbag in rosbags:
            self.logger.info(f"Exporting rosbag: {rosbag}")

            bag_ouput_dir = os.path.join(self.output_dir, os.path.basename(rosbag))
            self.create_output_dir(bag_ouput_dir)

            reader = self.load_rosbag(rosbag)

            topics_and_types = reader.get_all_topics_and_types()
            topic_type_map = {topic.name: topic.type for topic in topics_and_types}

            if self.image_topic not in topic_type_map.keys():
                self.logger.error(f"Could not find topic: {self.image_topic}.")
                return

            metadata: rosbag2_py.BagMetadata = reader.get_metadata()
            topic_info: rosbag2_py.TopicInformation = [
                topic
                for topic in metadata.topics_with_message_count
                if topic.topic_metadata.name == self.image_topic
            ][0]
            message_count = topic_info.message_count // self.frame_interval

            frame_count = 0
            while reader.has_next():
                bag_message = reader.read_next()
                topic_name, image_msg, _ = bag_message

                if topic_name != self.image_topic:
                    continue

                if frame_count % self.frame_interval == 0:
                    image = self.get_image_from_msg(
                        image_msg,
                        topic_type_map[topic_name],
                    )

                    if self.undistort:
                        image = self.coord_transform._calib.downscale_image(image)
                        image = self.distortion.undistort_image(image)

                    if self.convert_to_bev:
                        image = self.bev.transform_img(image)

                    filename = f"{os.path.basename(rosbag)}_frame{frame_count:06}.jpg"
                    cv2.imwrite(os.path.join(bag_ouput_dir, filename), image)

                    num_exports = frame_count // self.frame_interval
                    if num_exports % self.print_status_interval == 0:
                        self.logger.info(
                            f"Exported {num_exports}/{message_count} frames"
                        )

                frame_count += 1

            self.logger.info(
                f"Finished exporting {message_count} frames from rosbag {rosbag}"
            )


def main(args=None):
    """
    Main function to start the RosbagToImagesNode.

    Keyword Arguments:
        args -- Launch arguments (default: {None})
    """
    rclpy.init(args=args)
    node = RosbagToImagesNode()

    try:
        node.rosbag_to_images()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        # Shutdown if not already done by the ROS2 launch system
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
