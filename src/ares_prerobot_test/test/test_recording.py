import json

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from ares_prerobot_test.recording import SessionBag


def test_session_bag_closes_with_readable_samples(tmp_path):
    bag = SessionBag(tmp_path / 'recordings')
    bag.sample('/prerobot/chart_current', {'seq': 7, 'Id': [1, 2, 3, 4], 'Id_mean': 2.5})
    bag.sample('/prerobot/recording_event', {'event': 'stop', 'reason': 'halt_complete'})
    path = bag.path
    bag.close()

    assert (path / 'metadata.yaml').exists()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    samples = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        samples[topic] = json.loads(deserialize_message(raw, String).data)
    assert samples['/prerobot/chart_current']['Id_mean'] == 2.5
    assert samples['/prerobot/recording_event']['reason'] == 'halt_complete'
