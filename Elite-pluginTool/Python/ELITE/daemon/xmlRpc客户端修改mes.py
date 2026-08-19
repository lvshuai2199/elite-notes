import xmlrpc.client

proxy = xmlrpc.client.ServerProxy("http://192.168.249.128:4444")
# proxy = xmlrpc.client.ServerProxy("http://127.0.0.1:4444")
res = proxy.robot_start11()
print("调用返回：",res)