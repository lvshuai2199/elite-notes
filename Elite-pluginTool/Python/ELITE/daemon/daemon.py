#!/usr/bin/env python3
import time
import sys
import _thread
import re
import socket

from xmlrpc.server import SimpleXMLRPCServer
from http.server import BaseHTTPRequestHandler, HTTPServer

message = 'Hello, world'
ROBOT_IP = "192.168.249.128"
ROBOT_DASH_PORT = 29999
SERVER_RPC_IP = "0.0.0.0"
SERVER_RPC_PORT = 3333

# socket连接
def connectETController(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        return (False, None)

# Dashboard 通用通信
def dashboard_shell(content):
    conSuc, sock = connectETController(ROBOT_IP, ROBOT_DASH_PORT)
    if not conSuc:
        return "connect robot 29999 failed"
    # 清空欢迎信息
    sock.recv(4096)
    send_cmd = str(content) + '\n'
    sock.sendall(bytes(send_cmd, "utf-8"))
    time.sleep(0.05)
    recvData = sock.recv(4096)
    sock.close()
    recvData = recvData.decode("utf-8", errors="ignore")
    return recvData.replace('\n', '').replace('\r', '').strip()

# 上电单步
def powering_on():
    Data = dashboard_shell("robotControl -on")
    print("上电指令返回：", Data)
    time.sleep(0.5)
    return Data

# 循环释放抱闸
def brake_releasing():
    while True:
        Data = dashboard_shell("brakeRelease")
        print("抱闸指令返回：", Data)
        if Data == "Brake is released.":
            break
        time.sleep(0.5)
    return Data

# RPC：机器人上电+抱闸释放
def robot_start(*args):
    power_res = powering_on()
    if power_res != "Powering on":
        return f"上电失败：{power_res}"
    brake_res = brake_releasing()
    return "机器人上电+抱闸释放完成，返回：" + brake_res

# RPC：机器人断电
def robot_stop(*args):
    while True:
        Data = dashboard_shell("robotControl -off")
        print("断电指令返回：", Data)
        if Data == "Powering off":
            break
        time.sleep(0.5)
    return "机器人断电完成"

# RPC：设置http展示消息
def set_message(mes):
    global message
    message = mes
    print("客户端消息：" + mes)
    return message

# ========== 任务相关RPC接口 ==========
# 加载任务文件
def load_task(task_path):
    cmd = f"task -p {task_path}"
    res = dashboard_shell(cmd)
    print(f"加载任务[{task_path}]返回：{res}")
    return f"加载任务 {task_path} 结果：{res}"

# 启动已加载任务
def play_task():
    res = dashboard_shell("play")
    print("启动任务play返回：", res)
    if res == "Failed to execute: play":
        return "任务启动失败，请检查机器人上电/任务是否加载"
    return "任务启动执行，返回：" + res

# 暂停任务
def pause_task():
    res = dashboard_shell("pause")
    print("暂停任务返回：", res)
    return "任务暂停，返回：" + res

# 停止任务
def stop_task():
    res = dashboard_shell("stop")
    print("停止任务返回：", res)
    return "任务已停止，返回：" + res

# ========== MoveJ点位运动RPC接口（封装EliScript下发） ==========
# def robot_moveJ():
#     eli_script = '''
# def move_demo():
#     global u36335u28857_1_p = [0.38408322662661437, -0.14749999998051516, 0.5791660826988158, 3.141590486727266, 2.0510342851484963E-10, -1.570796326794897]
#     global u36335u28857_1_q = [0.0, -1.3553166793699976, -1.5417435069442484, -1.815330960932971, 1.570796327, 0.0]
#     global u36335u28857_2_p = [0.384082379510545, -0.14749999998051516, 0.47530833400077926, -3.141590901884441, 2.051034285150165E-10, -1.5707963267948963]
#     global u36335u28857_2_q = [0.0, -1.327509509546772, -1.8310296704013531, -1.5538480487312125, 1.570796327, 0.0]
#     global u36335u28857_3_p = [0.3840815975778526, 0.06940005726177045, 0.4753079998448488, -3.141591470706301, 7.522785813227031E-7, -1.570795659007012]
#     global u36335u28857_3_q = [0.56630344846347, -1.2721295632916005, -1.870202375828231, -1.5700556394316383, 1.570796327, 0.5663027806760305]
#
#     movej(get_inverse_kin(u36335u28857_1_p, qnear=u36335u28857_1_q), a=1.3962634015954636, v=1.0471975511965976)
#     movej(get_inverse_kin(u36335u28857_2_p, qnear=u36335u28857_2_q), a=1.3962634015954636, v=1.0471975511965976)
#     movej(get_inverse_kin(u36335u28857_3_p, qnear=u36335u28857_3_q), a=1.3962634015954636, v=1.0471975511965976)
# end
# move_demo()
# '''
#     cmd = f'runscript {eli_script}'
#     res = dashboard_shell(cmd)
#     print("执行moveJ脚本返回：", res)
#     return f"MoveJ运动指令下发完成，返回结果：{res}"

def robot_moveJ():
    global u36335u28857_1_p
    u36335u28857_1_p = [0.38408322662661437, -0.14749999998051516, 0.5791660826988158, 3.141590486727266, 2.0510342851484963E-10, -1.570796326794897]
    global u36335u28857_1_q
    u36335u28857_1_q = [0.0, -1.3553166793699976, -1.5417435069442484, -1.815330960932971, 1.570796327, 0.0]
    global u36335u28857_2_p
    u36335u28857_2_p = [0.384082379510545, -0.14749999998051516, 0.47530833400077926, -3.141590901884441, 2.051034285150165E-10, -1.5707963267948963]
    global u36335u28857_2_q
    u36335u28857_2_q = [0.0, -1.327509509546772, -1.8310296704013531, -1.5538480487312125, 1.570796327, 0.0]
    global u36335u28857_3_p
    u36335u28857_3_p = [0.3840815975778526, 0.06940005726177045, 0.4753079998448488, -3.141591470706301, 7.522785813227031E-7, -1.570795659007012]
    global u36335u28857_3_q
    u36335u28857_3_q = [0.56630344846347, -1.2721295632916005, -1.870202375828231, -1.5700556394316383, 1.570796327, 0.5663027806760305]
    movej(get_inverse_kin(u36335u28857_1_p, qnear=u36335u28857_1_q), a=1.3962634015954636, v=1.0471975511965976)
    movej(get_inverse_kin(u36335u28857_2_p, qnear=u36335u28857_2_q), a=1.3962634015954636, v=1.0471975511965976)
    movej(get_inverse_kin(u36335u28857_3_p, qnear=u36335u28857_3_q), a=1.3962634015954636, v=1.0471975511965976)
    return "robot_MoveJ"

# ========== HTTP网页服务 ==========
class RequestHandler(BaseHTTPRequestHandler):
    global message
    Page = '''\
        <html>
        <body>
        <p>{message} </p>
        </body>
        </html>
    '''
    def do_GET(self):
        print("do_GET")
        print("do_GET: " + message)
        mes = self.Page.replace("{message}", message)
        self.send_response(200)
        self.send_header("Content-Type", "text/html;charset=utf-8")
        self.send_header("Content-Length", str(len(mes.encode("utf-8"))))
        self.end_headers()
        self.wfile.write(mes.encode("utf-8"))

# RPC服务启动
def rpc_server(threadName, delay):
    sys.stdout.write("MyDaemon RPC daemon started\n")
    sys.stderr.write("MyDaemon RPC daemon started\n")
    server = SimpleXMLRPCServer((SERVER_RPC_IP, SERVER_RPC_PORT), allow_none=True, logRequests=True)
    # 注册所有接口
    server.register_function(set_message, "set_message")
    server.register_function(robot_start, "robot_start")
    server.register_function(robot_stop, "robot_stop")
    server.register_function(load_task, "load_task")
    server.register_function(play_task, "play_task")
    server.register_function(pause_task, "pause_task")
    server.register_function(stop_task, "stop_task")
    server.register_function(robot_moveJ, "robot_moveJ")
    server.serve_forever()

# HTTP服务启动
def http_server(threadName, delay):
    serverAddress = ('0.0.0.0', 5555)
    server = HTTPServer(serverAddress, RequestHandler)
    server.serve_forever()

# 主线程入口
if __name__ == '__main__':
    try:
        _thread.start_new_thread(http_server, ("Thread-1", 1, ))
        _thread.start_new_thread(rpc_server, ("Thread-2", 2,))
    except:
        print("Error: 无法启动线程")
    while True:
        time.sleep(1)