import socket
import time

#建立socket连接
def connectETController(ip, port):
    #ip, port：IP和端口号
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        # 修复bug：异常返回二元组，否则 conSuc, sock = xxx 会解包报错
        return (False, None)


#29999端口 dashboard_shell
def dashboard_shell(content):
    robot_ip = "192.168.249.128"
    port = 29999
    conSuc, sock = connectETController(robot_ip, port)
    if not conSuc or sock is None:
        return "connect fail"
    #清空缓存
    sock.recv(4096)
    #发送指令
    sock.sendall(bytes(str(content + '\n'),"utf-8"))
    recvData = sock.recv(4096)
    sock.close()
    recvData = recvData.decode("utf-8","ignore")
    return recvData.replace('\n', '').replace('\r', '')


# 30001端口：下发脚本执行
def send_script_30001(script_text):
    """
    30001脚本端口发送脚本
    :param script_text: 完整脚本字符串
    :return: (ok:bool, resp:str)
    """
    robot_ip = "192.168.249.128"
    port = 30001
    conSuc, sock = connectETController(robot_ip, port)
    if not conSuc or sock is None:
        return False, "30001端口连接失败"

    try:
        # 脚本结尾务必加换行
        send_bytes = bytes(script_text + "\n", "utf-8")
        sock.sendall(send_bytes)
        resp = sock.recv(8192).decode("utf-8","ignore")
        return True, resp
    except Exception as e:
        return False, str(e)
    finally:
        sock.close()


#利用29999端口打开电源
def powering_on():
    Data = dashboard_shell("robotControl -on")
    print(Data)
    if Data == 'Powering on':
        print('上电成功')
    else:
        print(Data)
    time.sleep(0.5)

#利用29999端口释放抱闸
def brake_releasing():
    while True:
        Data = dashboard_shell("brakeRelease")
        if Data == 'Brake is released.':
            print('抱闸释放成功')
            break
        else:
            print(Data)
        time.sleep(0.5)

#利用29999端口运行任务
def play():
    Data = dashboard_shell("play")
    if Data == 'Starting task':
        print('已启动任务')
    else:
        print('运行失败：'+Data)
    time.sleep(1)

#利用29999端口查询任务状态
def task():
    Data = dashboard_shell("task -r")
    if Data == 'Task is running':
        print('任务正在运行')
    else:
        print('当前运行状态为：'+Data)
    time.sleep(1)


if __name__ == "__main__":
    #打开电源
    powering_on()
    #释放抱闸
    brake_releasing()

    # ---------------------- 要发送的脚本 a() ----------------------
    script = """def a():
 movej([-0.005,-1.564,-1.587,-1.474,1.571,-0.005],a=1.4,v=0.5,t=0,r=0)
end
"""
    #发送30001执行
    ok, ret = send_script_30001(script)
    # print(f"30001发送结果 ok={ok}, response:\n{ret}")

    # play()
    # task()