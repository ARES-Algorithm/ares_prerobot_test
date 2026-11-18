"""Write one controller run to rosbag without a recorder startup race."""
import json
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String


TOPICS = {
    '/prerobot/cmd_vel': 'geometry_msgs/msg/TwistStamped',
    '/prerobot/sent_cmd': 'geometry_msgs/msg/TwistStamped',
    '/prerobot/usb_rx': 'std_msgs/msg/Float32MultiArray',
    '/prerobot/chart_command': 'std_msgs/msg/String',
    '/prerobot/chart_wheel': 'std_msgs/msg/String',
    '/prerobot/chart_current': 'std_msgs/msg/String',
    '/prerobot/recording_event': 'std_msgs/msg/String',
}


class SessionBag:
    def __init__(self, root):
        root = Path(root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.path = root / f"{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:6]}"
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.path), storage_id='sqlite3'),
                         rosbag2_py.ConverterOptions('cdr', 'cdr'))
        for name, msg_type in TOPICS.items():
            fields = dict(name=name, type=msg_type, serialization_format='cdr')
            # Humble has no topic id argument; newer ROS releases require it.
            try:
                metadata = rosbag2_py.TopicMetadata(**fields)
            except TypeError:
                metadata = rosbag2_py.TopicMetadata(id=0, **fields)
            self.writer.create_topic(metadata)

    def write(self, topic, message):
        self.writer.write(topic, serialize_message(message), time.time_ns())

    def sample(self, topic, sample):
        # JSON preserves the named values used by the live charts, including
        # missing derivatives as null; raw ROS packets are recorded separately.
        self.write(topic, String(data=json.dumps(sample, allow_nan=False, separators=(',', ':'))))

    def close(self):
        if self.writer is not None:
            # Humble finalizes the bag when the writer is destroyed.
            if hasattr(self.writer, 'close'):
                self.writer.close()
            self.writer = None
