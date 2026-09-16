from robot import *
import argparse
import logging
import time

parser = argparse.ArgumentParser()
parser.add_argument('--host', default='localhost', help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30001, help='port number (30001)')
parser.add_argument('--samples', type=int, default=100, help='number of samples to record')

args = parser.parse_args()

# When robot throw exception, this function will be called
def robot_exception_cb(robot_exception : RobotException):
    print(f"time stamp: {robot_exception.time_stamp}")
    print(f"source: {robot_exception.exception_source}")
    if robot_exception.exception_source == 10:
        print(f"\tscript line: {robot_exception.script_line}")
        print(f"\tscript line: {robot_exception.script_column}")
        print(f"\tscript line: {robot_exception.description}")
    else:
        print(f"\tcode: {robot_exception.code}")
        print(f"\tsub-code: {robot_exception.subcode}")
        print(f"\tlevel: {robot_exception.level}")
        print(f"\tlevel: {robot_exception.data}")
    


robot = Robot('RobotStateMessage.xlsx', 'v2.6.0', robot_exception_cb)
robot.connect(args.host, args.port)

sample_count = args.samples

# Make a robot runtime exception
exception_script = b"def func():\n\tabcd(123)\nend\n"
robot.send(exception_script)

while sample_count >= 0:
    data = robot.get_data()
    if data == None:
        logging.warning("Data is None")
        continue
    print("actual_joint : ")
    print(data.actual_joint0, end=' ')
    print(data.actual_joint1, end=' ')
    print(data.actual_joint2, end=' ')
    print(data.actual_joint3, end=' ')
    print(data.actual_joint4, end=' ')
    print(data.actual_joint5)
    sample_count -= 1
    time.sleep(0.1)
    
    
    