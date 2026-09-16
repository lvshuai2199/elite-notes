# 一、依赖
本示例依赖 pandas 库，用于读取 'RobotStateMessage.xlsx' 表格。可以使用 pip 安装该库:  `pip install pandas`



# 二、'RobotStateMessage.xlsx' 表格
该表格由 '机器人状态报文-中文' 修改而来。详细可看该表格的"说明"sheet

# 三、Robot 类
1. Robot(excel, sheet, robot_exception_hook)
   * 说明：创建Robot类
   * 参数：
     * excel: excel 表格名称
     * sheet: 表格里所选用的sheet名称 
     * robot_exception_hook: 机器人异常回调函数
   * 返回：Robot类的实例
2. connect(ip, port)
    * 说明：连接到控制器
    * 参数：
      * ip：控制器IP地址
      * port：端口缺省值为30001
    * 返回：无
3. get_data()
   * 说明：接收并解析报文
   * 参数：无
   * 返回：RobotData 对象，该对象的成员变量是通过excel表格解析得出的