#!/usr/bin/env python3
"""Run in an isolated ROS domain: mock plant + real runner + services + report."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ares_prerobot_test.analysis import analyze


def main():
    rclpy.init(); node=Node('pipeline_check')
    processes=[]
    with tempfile.TemporaryDirectory(prefix='prerobot_check_') as tmp:
        logs=[]
        try:
            for exe,extra in [('mock_chassis',[]),('test_runner',['--ros-args','-p','output:='+tmp,'-p','hold:=1.0'])]:
                f=open(Path(tmp)/(exe+'.log'),'w'); logs.append(f)
                processes.append(subprocess.Popen(['ros2','run','ares_prerobot_test',exe,*extra],stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
            client=node.create_client(Trigger,'/prerobot/start')
            assert client.wait_for_service(timeout_sec=15), 'start service absent'
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                future=client.call_async(Trigger.Request()); rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if future.done() and future.result().success: break
                time.sleep(.1)
            else: raise AssertionError('runner never became ready')
            directory=Path(future.result().message)
            deadline=time.monotonic()+15
            while time.monotonic()<deadline and not (directory/'outcome.json').exists(): time.sleep(.1)
            assert (directory/'outcome.json').exists(), 'experiment did not finish'
            report=analyze(directory)
            assert report['outcome']['reason']=='completed',report
            assert .45<report['peak_speed']<.6,report
            assert report['stable_speed'] is not None,report
            assert report['imu']['available'],report
            # Exercise explicit stop and missing-feedback termination in separate runs.
            def start_again():
                f=client.call_async(Trigger.Request()); rclpy.spin_until_future_complete(node,f,timeout_sec=2)
                assert f.result().success,f.result().message
                return Path(f.result().message)
            second=start_again(); time.sleep(.4)
            stop=node.create_client(Trigger,'/prerobot/stop'); assert stop.wait_for_service(timeout_sec=2)
            f=stop.call_async(Trigger.Request()); rclpy.spin_until_future_complete(node,f,timeout_sec=2)
            assert f.result().success
            time.sleep(1.5)
            assert json.loads((second/'outcome.json').read_text())['reason']=='operator_stop'
            third=start_again(); time.sleep(.4)
            os.killpg(processes[0].pid,signal.SIGINT); processes[0].wait(timeout=5)
            time.sleep(1.5)
            assert json.loads((third/'outcome.json').read_text())['reason']=='feedback_lost'
            # Keep a reviewable first-run artifact outside the temporary directory.
            import shutil
            dest=Path('results/verification')
            if dest.exists():
                dest=Path('results')/('verification_'+str(time.time_ns()))
            shutil.copytree(directory,dest)
            print(json.dumps({'passed':['complete_run','operator_stop','feedback_loss','analysis'], 'artifact':str(dest),'peak_speed':report['peak_speed'],'peak_acceleration':report['peak_acceleration'],'peak_jerk':report['peak_jerk']},indent=2))
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=5)
            for f in logs: f.close()
            for p in Path(tmp).glob('*.log'):
                if any(x.returncode not in (0,-signal.SIGINT) for x in processes): print(p.read_text())
            node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
