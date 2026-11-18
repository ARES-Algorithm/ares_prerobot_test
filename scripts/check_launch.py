#!/usr/bin/env python3
"""Verify the installed launch entry and finalized bag in an isolated ROS domain."""
import signal
import subprocess
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def main():
    rclpy.init(); n=Node('launch_bag_check')
    Path('results').mkdir(exist_ok=True)
    with open('results/launch_check.log','w') as log:
        p=subprocess.Popen(['ros2','launch','ares_prerobot_test','test.launch.py','record:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            c=n.create_client(Trigger,'/prerobot/start'); assert c.wait_for_service(timeout_sec=15)
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=2)
                if f.done() and f.result().success: break
                time.sleep(.1)
            else: raise AssertionError('start failed')
            output=Path(f.result().message)
            deadline=time.monotonic()+12
            while time.monotonic()<deadline and not (output/'outcome.json').exists(): time.sleep(.1)
            assert (output/'outcome.json').exists()
        finally:
            # Signal launch once; launch forwards shutdown to its child processes.
            p.send_signal(signal.SIGINT);p.wait(timeout=10);n.destroy_node();rclpy.shutdown()
    bags=sorted(Path('results/bags').glob('*/metadata.yaml'))
    assert bags,'bag metadata missing'
    info=yaml.safe_load(bags[-1].read_text())['rosbag2_bagfile_information']
    assert info['message_count']>100,info
    names={t['topic_metadata']['name'] for t in info['topics_with_message_count'] if t['message_count']>0}
    assert {'/transformed/pose','/imu/data','/prerobot/cmd_vel'} <= names,names
    text=Path('results/launch_check.log').read_text()
    assert 'Traceback' not in text,text
    assert 'process has died' not in text,text
    print('Launch, clean shutdown and bag passed:',bags[-1],info['message_count'],'messages')
if __name__=='__main__':main()
