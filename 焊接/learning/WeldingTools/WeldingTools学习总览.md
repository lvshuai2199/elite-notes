# WeldingTools 学习总览

## 1. 文档定位

WeldingTools 不是一个独立运行的桌面应用，而是运行在 ELITE/Elibot 机器人环境中的焊接工艺插件。

项目主要负责：

1. 在机器人任务树中提供焊接相关节点。
2. 提供焊机、焊接工艺和摆动参数配置。
3. 保存 Pose、JointPositions 和工艺数据。
4. 将焊接段转换为普通运动命令或离散摆动轨迹。
5. 生成机器人脚本，并与本地 WeldingDaemon 通信。

推荐把整个项目理解为以下数据流：

```text
UI View
    -> TaskNodeContribution
        -> DataModelWrapperExtension
            -> Pose / JointPositions / 工艺参数
                -> BaseCommand 或 TrajectoryManager
                    -> ScriptWriter
                        -> 机器人脚本

带摆动的轨迹：
TaskNodeContribution
    -> TrajectoryManager
        -> JSON
            -> XmlRpcDaemonFacade
                -> WeldingDaemon
                    -> NodeContext 离散点
                        -> MoveP / MoveL / MoveC
                            -> ScriptWriter
```

## 2. 推荐阅读顺序

建议按以下顺序阅读：

1. `README.md`
2. `pom.xml`
3. `src/main/java/.../impl/Activator.java`
4. `src/main/java/.../task/base/command/WeldCommandAbstract.java`
5. `src/main/java/.../task/base/weld/WeldTaskNodeAbstract.java`
6. `src/main/java/.../task/base/point/WeldPointTaskNodeAbstract.java`
7. `src/main/java/.../task/base/linear/WeldLinearTaskNodeAbstract.java`
8. `src/main/java/.../task/weld/WeldTaskNodeContribution.java`
9. `src/main/java/.../task/weldstart/WeldStartTaskNodeContribution.java`
10. `src/main/java/.../task/weldshape/WeldShapeTaskNodeContribution.java`
11. `src/main/java/.../domain/TrajectoryManager.java`
12. `src/main/java/.../domain/NodeContext.java`
13. `src/main/java/.../task/weldmultipass/WeldMultiPassTaskNodeContribution.java`
14. `src/main/java/.../utils/MultiPathUtils.java`
15. `src/main/java/.../daemon/WeldingDaemonServiceImpl.java`
16. `src/test/java/.../GenerateTrajectoryTest.java`

## 3. 项目分层

### 3.1 插件入口层

主要目录：

```text
impl/Activator.java
```

`Activator.java` 是 OSGi 插件入口，负责注册：

- 本地化资源。
- WeldingDaemon 服务。
- Swing 任务节点服务。
- 配置节点服务。
- 导航栏入口。

### 3.2 任务节点层

任务节点通常由三部分组成：

```text
xxxServiceImpl
xxxContribution
xxxView
```

主要节点：

| 目录 | 作用 |
| --- | --- |
| `task/weldtool` | 焊接工具总入口 |
| `task/weld` | 普通焊接根节点 |
| `task/weldrun` | 多层多道中的单道焊接段 |
| `task/weldmultipass` | 多层多道根节点 |
| `task/weldpoint` | 接近点、结束点等普通点位 |
| `task/weldstart` | 起弧点和焊接段入口 |
| `task/weldshape` | 直线段和圆弧段 |
| `task/weldbindpoint` | 绑定点 |
| `task/weldparameters` | 焊机工艺参数 |
| `task/tack` | 点焊 |
| `task/touchsense` | 接触感知 |
| `task/touchsearch` | 搜索点 |
| `task/tracking` | 跟踪 |
| `task/arctracking` | 电弧跟踪 |

### 3.3 基础抽象层

| 类 | 职责 |
| --- | --- |
| `WeldCommandAbstract` | 脚本、命令缓存、整体偏移、任务树操作 |
| `WeldTaskNodeAbstract` | 启用状态、等待时间、对子树整体偏移 |
| `WeldPointTaskNodeAbstract` | 普通点位示教、Pose、移动类型 |
| `WeldLinearTaskNodeAbstract` | 直线/圆弧点位示教和逆运动学 |
| `BaseCommand` | 运动命令统一抽象 |
| `CommandDomain` | Pose、速度、加速度和偏移表达式 |

### 3.4 配置和参数层

主要目录：

```text
impl/configure/**
impl/task/weldparameters/**
```

焊机适配器包括：

- `AotaiWelder`
- `GysWelder`
- `KolarcWelder`
- `LorchWelder`
- `MerkleWelder`
- `MegmeetCanWelder`
- `MegmeetModbusWelder`
- `KemppidcmWelder`
- `AxMigOneWelder`
- `ScriptCBWelder`
- `GenericIOWelder`
- `EmptyWelder`

`BaseMachineAbstract` 和各品牌 machine contribution 负责把不同焊机参数页面统一到同一套任务节点结构中。

### 3.5 轨迹、数学和 daemon 层

| 目录/类 | 作用 |
| --- | --- |
| `domain/TrajectoryManager.java` | 摆动轨迹 JSON、daemon 调用、离散结果 |
| `domain/NodeContext.java` | daemon 返回的离散运动点 |
| `domain/MultiPathManager.java` | 多道路径中间模型 |
| `utils/MultiPathUtils.java` | 多道线段/圆弧偏移 |
| `utils/InverseKinematicsUtils.java` | 逆运动学请求和结果解析 |
| `utils/MathOffsetUtils.java` | Pose 和坐标变换 |
| `XmlRpcDaemonFacade.java` | XML-RPC 通信 |
| `daemon/WeldingDaemonServiceImpl.java` | daemon 安装和启动 |
| `math/*` | 三维、向量、圆弧和坐标系计算 |

## 4. 任务树结构

### 4.1 单直线焊缝

创建入口：

```java
WeldToolTaskNodeContribution.addWeldNode(true)
```

任务树：

```text
WeldToolTaskNodeContribution
└── WeldTaskNodeContribution
    ├── WeldPointTaskNodeContribution       // 接近点/进枪点
    ├── WeldStartTaskNodeContribution       // 起弧点
    │   └── WeldLinearTaskNodeContribution  // 直线焊接点
    └── WeldPointTaskNodeContribution       // 结束点/退枪点
```

创建位置：

```text
task/weldtool/WeldToolTaskNodeContribution.java
addWeldNode(boolean)
```

主要流程：

1. 创建普通焊接根节点。
2. 创建接近点。
3. 创建起弧点。
4. 在起弧点下创建直线段。
5. 创建结束点。
6. 设置父子关系并加入任务树。

### 4.2 多层多道焊缝

创建入口：

```java
WeldToolTaskNodeContribution.addMultiPassNode()
```

初始任务树：

```text
WeldToolTaskNodeContribution
└── WeldMultiPassTaskNodeContribution
    └── WeldRunTaskNodeContribution       // 第一道
        ├── WeldPointTaskNodeContribution // 接近点
        ├── WeldStartTaskNodeContribution // 起弧点
        │   └── WeldLinearTaskNodeContribution
        └── WeldPointTaskNodeContribution // 结束点
```

完成多道创建后：

```text
WeldMultiPassTaskNodeContribution
├── WeldRunTaskNodeContribution // 第一道
├── WeldRunTaskNodeContribution // 第二道
├── WeldRunTaskNodeContribution // 第三道
└── ...
```

每一道仍然复用同样的点位类，区别是放在不同的 `WeldRunTaskNodeContribution` 下，后续焊道通常由基础焊道偏移复制得到。

## 5. 点位类与常用接口

### 5.1 点位对应关系

| 业务含义 | 具体类 | 数据基类 | Pose 字段 |
| --- | --- | --- | --- |
| 接近点/进枪点 | `WeldPointTaskNodeContribution` | `WeldPointTaskNodeAbstract` | `movePoseKey` |
| 起弧点 | `WeldStartTaskNodeContribution` | `WeldPointTaskNodeAbstract` | `movePoseKey` |
| 直线点 | `WeldLinearTaskNodeContribution` | `WeldLinearTaskNodeAbstract` | `linearMovePoseKey` |
| 圆弧通过点 | `WeldCircularTaskNodeContribution` | `WeldLinearTaskNodeAbstract` | `circularPassMovePoseKey` |
| 圆弧结束点 | `WeldCircularTaskNodeContribution` | `WeldLinearTaskNodeAbstract` | `circularEndMovePoseKey` |
| 结束点/退枪点 | `WeldPointTaskNodeContribution` | `WeldPointTaskNodeAbstract` | `movePoseKey` |

接近点和结束点使用同一个具体类，通过 `isApproach` 区分：

```text
isApproach = true  -> 接近点/进枪点
isApproach = false -> 结束点/退枪点
```

起弧点虽然是独立的 `WeldStartTaskNodeContribution`，但继承自 `WeldPointTaskNodeAbstract`，因此仍使用 `movePoseKey`。

### 5.2 普通点位接口

核心基类：

```java
WeldPointTaskNodeAbstract
```

常用接口：

| 接口 | 作用 |
| --- | --- |
| `setPoint()` | 从机器人当前 TCP 和关节位置示教 |
| `getMovePose()` | 获取普通点位 Pose |
| `setMovePose(Pose)` | 写入 Pose，并请求逆运动学 |
| `setPointPose(Pose)` | 只写入 Pose |
| `getMoveJoint()` | 获取关节位置 |
| `setPointJoint(JointPositions)` | 写入关节位置 |
| `moveHere()` | 移动机器人到目标点 |
| `getMoveType()` | 获取移动类型 |
| `setMoveType(String)` | 设置移动类型 |
| `setTitleAndApproach(String, boolean)` | 设置名称和接近/结束属性 |

示教调用链：

```text
setPoint()
    -> RobotMovementService.requestUserToSetPosition()
        -> getCurrentTCPPose()
        -> getCurrentJointPositions()
            -> 写入 movePoseKey / moveJointKey
```

### 5.3 直线和圆弧点位接口

核心基类：

```java
WeldLinearTaskNodeAbstract
```

示教接口：

```java
setLinearPoint()
setCircularPassPoint()
setCircularEndPoint()
```

直接写入接口：

```java
setLinearMovePose(Pose)
setCircularPassMovePose(Pose)
setCircularEndMovePose(Pose)
```

这些 `set...MovePose()` 方法会保留已有的近似关节位置，并通过 `InverseKinematicsUtils` 重新请求关节角。

### 5.4 逆运动学接口

入口：

```java
InverseKinematicsUtils.getInverseKinematicsReq(
    Pose poseRPY,
    JointPositions nearPosition
)
```

流程：

```text
Pose
    -> 转换为当前脚本模式
    -> 拼接 get_inverse_kin(...)
    -> CommandRequestService.sendMessageReq(...)
    -> 解析返回字符串
    -> JointPositions
```

`nearPosition` 用于生成 `qnear`，尽量保持机器人姿态连续。

## 6. 数据模型和节点生命周期

### 6.1 DataModelDomain

所有可持久化字段通常使用类型化数据键：

```java
DataModelDomain<Pose>
DataModelDomain<JointPositions>
DataModelDomain<Boolean>
DataModelDomain<Double>
DataModelDomain<String>
```

典型数据字段：

```java
movePoseKey
moveJointKey
linearMovePoseKey
circularPassMovePoseKey
circularEndMovePoseKey
moveTypeKey
isApproach
effectiveKey
waitTimeKey
```

### 6.2 DataModelWrapperExtension

常用读写接口：

```java
setDataModel(key)
setDataModel(key, value)
getDataModel(key)
```

### 6.3 UndoRedoManager

用户可见的节点树和数据修改一般包装在：

```java
taskApiProvider.getUndoRedoManager()
    .recordChanges(() -> {
        // 修改数据模型或任务树
    });
```

适用场景：

- 设置 Pose 和关节位置。
- 添加或删除节点。
- 修改焊接参数。
- 创建多层多道副本。
- 修改多道偏移。

### 6.4 节点生命周期

| 方法 | 作用 |
| --- | --- |
| `onInserted` | 首次插入时初始化名称和父子关系 |
| `loadComplete` | 加载完成后恢复父子关系和状态 |
| `setTaskNodeContributionViewProvider` | 关联 View 和参数组件 |
| `onViewOpen` | 刷新 UI、注册示教监听 |
| `onViewClose` | 清理监听 |
| `generateScript` | 输出机器人脚本 |
| `isDefined` | 判断节点是否配置完整 |

## 7. 脚本和运动命令

### 7.1 脚本生成层级

```text
WeldTaskNodeContribution.generateScript()
    -> scriptWriter.writeChildren()

WeldRunTaskNodeContribution.generateScript()
    -> scriptWriter.writeChildren()
    -> sleep(waitTime)

WeldStartTaskNodeContribution.generateScript()
    -> 起弧前移动
    -> createCommand()
    -> scriptWriter.writeChildren()
    -> arcOff()
```

### 7.2 WeldCommandAbstract

核心基类：

```java
WeldCommandAbstract
```

常用接口：

| 接口 | 作用 |
| --- | --- |
| `createCommands(ArrayList<BaseCommand>)` | 缓存焊接段的运动命令 |
| `destroyCommands()` | 清除命令缓存 |
| `setBaseWelder(BaseWelder)` | 设置当前焊机实现 |
| `generateScript(ScriptWriter)` | 输出焊机起弧和运动命令 |
| `offset(Transform3D, poseKey, jointKey)` | 对点位应用空间变换 |
| `getOffsetTaskNodeDomain()` | 导出节点及数据模型用于复制 |
| `setBaseNextWeldNode()` | 选中下一个焊接节点 |
| `setBasePreviousWeldNode()` | 选中上一个焊接节点 |
| `deleteBaseWeldNode()` | 删除当前焊接节点 |

默认命令输出逻辑：

```text
1. 如果配置了焊机，输出 welder.arcOn()
2. 依次执行 BaseCommand.command()
```

### 7.3 运动命令

| 类 | 当前脚本输出 | 用途 |
| --- | --- | --- |
| `MoveJCommand` | `movej(...)` | 关节运动 |
| `MoveLCommand` | 当前实现输出 `movep(...)` | 直线/笛卡尔点运动 |
| `MovePCommand` | `movep(...)` | 离散轨迹点 |
| `MoveCCommand` | `movec(...)` | 圆弧运动 |

注意：`MoveLCommand` 的类名是 MoveL，但当前 `command()` 实现拼接的是 `movep(...)`，实际行为应以实现为准。

常见接口：

```java
command(ScriptWriter scriptWriter)
setAllowableOffset(boolean)
setFramePose(Pose)
```

命令构造前通常需要转换单位：

```text
速度：m/s
加速度：m/s²
距离：m
角度：rad
等待时间：s
```

## 8. 核心轨迹生成

### 8.1 普通模式和摆动模式

项目存在两条轨迹生成路径。

普通模式：

```text
WeldShapeTaskNodeContribution
    -> WeldStartTaskNodeContribution.getBaseCommandList(...)
        -> MoveLCommand 或 MoveCCommand
            -> ScriptWriter
```

摆动模式：

```text
WeldShapeTaskNodeContribution
    -> TrajectoryManager
        -> JSON
            -> XmlRpcDaemonFacade
                -> WeldingDaemon
                    -> NodeContext
                        -> MoveP / MoveL / MoveC
                            -> ScriptWriter
```

判断依据：

```java
WeldShapeTaskNodeContribution.getBaseOscillation()
```

返回非空时，通常进入 daemon 离散轨迹流程。

### 8.2 WeldStartTaskNodeContribution.createCommand()

这是焊接段轨迹生成的核心入口：

```java
WeldStartTaskNodeContribution.createCommand()
```

内部流程：

```text
1. 创建 TrajectoryManager。
2. 遍历起弧点下的焊接形状节点。
3. 收集带摆动的直线或圆弧段。
4. Linux 环境下调用 generateTrajectory()。
5. 读取 getDiscretePoint()。
6. 按 moveTypeId 创建运动命令。
7. 按 taskPointId 将命令分配回原始焊接段。
```

### 8.3 直线段收集

接口：

```java
TrajectoryManager.addLinePoint(
    BaseOscillation oscillation,
    Pose beginPoint,
    Pose endPoint,
    double radius,
    AngleData angleData,
    double speed
)
```

流程：

1. 如果启用了 `AngleData`，对起点和终点进行角度偏移。
2. 将 Pose 转换成 m/rad 格式。
3. 写入摆动类型、速度和摆动参数。
4. 写入直线段关键点。

### 8.4 圆弧段收集

接口：

```java
TrajectoryManager.addCirclePoint(
    BaseOscillation oscillation,
    Pose beginPoint,
    Pose auxiliaryPoint,
    Pose endPoint,
    double arcAngle,
    double radius,
    AngleData angleData,
    double speed,
    boolean arcFixedMode
)
```

三个点的含义：

```text
beginPoint     -> 圆弧起点
auxiliaryPoint -> 圆弧通过点
endPoint       -> 圆弧结束点
```

### 8.5 daemon 离散化

生成入口：

```java
TrajectoryManager.generateTrajectory()
```

流程：

```text
json2String()
    -> XmlRpcDaemonFacade.setDiscreteData(...)
        -> XML-RPC setDiscreteData
            -> daemon 返回 JSON
                -> parseRetData(...)
                    -> NodeContext 列表
```

当前 XML-RPC 地址：

```text
127.0.0.1:4444/RPC2
```

### 8.6 NodeContext

`NodeContext` 表示 daemon 返回的一个离散运动点：

| 字段 | 含义 |
| --- | --- |
| `pose` | 当前运动点 |
| `poseVia` | 圆弧通过点 |
| `moveTypeId` | 0=`MoveP`，1=`MoveL`，2=`MoveC` |
| `moveSpeed` | 速度，m/s |
| `moveAcc` | 加速度，m/s² |
| `blendRadius` | 转接半径，m |
| `sleepTime` | 等待时间，s |
| `taskPointId` | 所属原始焊接段 ID |
| `framePose` | 点位平移坐标系 |

转换逻辑：

```java
moveTypeId == 0
    -> new MovePCommand(...)

moveTypeId == 1
    -> new MoveLCommand(...)

moveTypeId == 2
    -> new MoveCCommand(poseVia, pose, ...)
```

之后：

```java
baseCommand.setFramePose(nodeContext.getFramePose());
baseCommand.setAllowableOffset(isAllowSearchOffset());
```

最后通过 `taskPointId` 将命令归属到对应的 `WeldShapeTaskNodeContribution`。

## 9. TrajectoryManager 和 JSON

### 9.1 JSON 的整体结构

`TrajectoryManager.json2String()` 生成的数据结构大致是：

```json
{
  "trajectory": [
    {
      "type": 1,
      "speed": 0.05,
      "parameters": {
        "period": 2.0,
        "amplitude": 0.003
      },
      "radius": 0.001,
      "points": [
        [0.1, 0.2, 0.3, 0, 0, 0],
        [0.4, 0.2, 0.3, 0, 0, 0]
      ]
    }
  ]
}
```

### 9.2 公共字段

| 字段 | 含义 |
| --- | --- |
| `type` | 摆动类型 ID |
| `speed` | 焊接速度 |
| `parameters` | 摆动参数 |
| `radius` | 混合或转接半径 |
| `points` | 轨迹关键 Pose |
| `arcAngle` | 圆弧角度，rad |
| `isFixed` | 固定圆弧模式 |

### 9.3 摆动类型

| 类型 | 主要参数 |
| --- | --- |
| `ZIGZAG` | 周期、振幅、左右停留、摆动角、中间停留 |
| `TRAPEZOID` | 周期、振幅、左右停留、混合半径 |
| `CRESCENT` | 深度、左右停留 |
| `CIRCLE` | 宽度、方向 |

单位转换在 `TrajectoryManager.oscillation2Json()` 中完成：

```text
Pose：m / rad
振幅：m
频率：通常转换为 1/s
角度：rad
```

梯形摆动的频率处理和其他类型不同，修改时要保留该特殊分支。

## 10. 多层多道路径偏移

### 10.1 偏移流程

```text
WeldMultiPassTaskNodeContribution.newMultiPass(...)
    -> setChildrenOffset(...)
        -> createOffsetExtensionNode(...)
        -> createMultiPathManager(...)
        -> MultiPathManager.offsetPath(...)
        -> setMultiPathManager(...)
            -> setPoseMultiPath(...)
                -> 写回各焊道 Pose
```

### 10.2 MultiPathManager

`MultiPathManager` 是几何偏移中间模型，不负责生成机器人脚本。

常用接口：

```java
addStartPoint(Pose)
addLinePoint(Pose)
addCirclePoint(Pose passPoint, Pose endPoint)
offsetPath(Pose userFrame, double offsetDist, double zOffset)
getPoseList()
getMultiPathNodes()
```

内部节点：

```text
START_NODE
    -> startNodePnt

LINE_NODE
    -> lineEndPnt

ARC_NODE
    -> arcPassPnt
    -> arcEndPnt
```

需要注意：`START_NODE` 更接近“普通路径点”的概念，不一定只表示真正的起弧点。`createMultiPathManager()` 会把 `WeldPointTaskNodeAbstract` 类型的点统一加入为 `START_NODE`。

### 10.3 MultiPathUtils

主要入口：

```java
MultiPathUtils.welding_path_offset_by_section(...)
```

算法会把路径拆成：

```text
TRANSITION_SECTION
LINE_SECTION
ARC_SECTION
```

随后完成：

1. 线段或圆弧自身偏移。
2. 相邻线段交点计算。
3. 线段与圆弧交点计算。
4. 圆弧与圆弧交点计算。
5. 起始、结束过渡点修正。
6. 将偏移后的点重新组装成 `MultiPathNode`。

需要重点验证：

- 直线连接直线。
- 直线连接圆弧。
- 圆弧连接直线。
- 圆弧连接圆弧。
- 左右偏移方向。
- `up` 和 `out` 偏移。
- 起始和结束过渡段。

## 11. 坐标和单位转换

### 11.1 常见单位

```text
点位数据：距离常以 mm 表示
轨迹 JSON：距离为 m，角度为 rad
机器人脚本：速度 m/s，加速度 m/s²
```

常用转换：

```java
pose.toArray(Length.Unit.M, Angle.Unit.RAD)
pose.getPosition().toArray(Length.Unit.MM)
```

### 11.2 AngleData 和 OffsetAngle

焊接段启用工作角、前进角时，使用：

```java
AngleData
OffsetAngle
AngleTransUtils.lineOffset(...)
AngleTransUtils.circleOffset(...)
```

直线段会对起点和终点统一偏移。

圆弧段会对起点、通过点和终点统一偏移。

### 11.3 MathOffsetUtils

常用接口：

| 接口 | 用途 |
| --- | --- |
| `offsetPoint(Transform3D, Pose)` | 对 Pose 应用空间变换 |
| `offsetPoint(Pose, Vector3d, Pose)` | 在用户坐标系平移点位 |
| `calOffsetTransform(...)` | 根据点或特征计算变换 |
| `getVecInUserFrame(...)` | 将基坐标系向量转换到用户坐标系 |

## 12. Daemon 启动和通信

### 12.1 WeldingDaemonServiceImpl

主要职责：

1. 根据系统架构安装 daemon 资源。
2. 获取对应架构的 `WeldingDaemon`。
3. 自动启动 daemon。

### 12.2 XmlRpcDaemonFacade

常用接口：

```java
start()
isStarted()
setDiscreteData(String data)
```

轨迹生成使用的 XML-RPC 方法：

```text
setDiscreteData
```

## 13. 常用扩展位置

### 13.1 新增一种运动段

通常需要修改：

1. 新增 Service 和 Contribution。
2. 增加 Pose 和 JointPositions 数据字段。
3. 在 `WeldStartTaskNodeContribution.createTrajectoryManager()` 中收集。
4. 在 `getBaseCommandList()` 中支持普通模式。
5. 在 `setLinearCommands()` 中支持离散结果。
6. 更新 `Node3DDomain` 三维预览。
7. 更新任务树插入规则和偏移复制逻辑。

### 13.2 新增一种摆动类型

通常需要修改：

1. 新增 `BaseOscillation` 子类。
2. 扩展 `OscillationType`。
3. 在 `TrajectoryManager.oscillation2Json()` 中加入 JSON 参数。
4. 增加 UI 参数面板和校验。
5. 确认 daemon 支持新的 `type` 和 `parameters`。

### 13.3 修改多道偏移

重点文件：

```text
WeldMultiPassTaskNodeContribution
MultiPathManager
MultiPathUtils
MultiPathSection
MathOffsetUtils
```

## 14. 调试排查清单

### 14.1 任务树问题

检查：

```text
canInsertToTargetParent()
canInsertAChild()
onInserted()
loadComplete()
setParentContributionOnChildren()
```

### 14.2 点位问题

检查：

```text
Pose 是否为零 Pose
JointPositions 是否有效
setPoint() 是否成功回调
setMovePose() 是否触发逆运动学
isDefined() 是否通过
```

### 14.3 轨迹问题

建议按顺序检查：

1. `getBaseOscillation()` 是否返回空。
2. `TrajectoryManager.json2String()` 是否完整。
3. daemon 是否启动在 `127.0.0.1:4444`。
4. `setDiscreteData()` 是否返回 `status=ok`。
5. `NodeContext.moveTypeId` 是否正确。
6. `taskPointId` 是否能映射回原始焊接段。
7. `MoveP/MoveL/MoveCCommand.command()` 是否生成预期脚本。

### 14.4 单位问题

优先确认：

```text
mm 是否误当成 m
deg 是否误当成 rad
mm/s 是否正确转换为 m/s
角度偏移是否重复应用
```

## 15. 核心文件索引

| 文件 | 重点 |
| --- | --- |
| `impl/Activator.java` | 插件注册入口 |
| `task/base/command/WeldCommandAbstract.java` | 命令缓存、脚本、偏移、节点操作 |
| `task/base/weld/WeldTaskNodeAbstract.java` | 启用状态、等待和整体偏移 |
| `task/base/point/WeldPointTaskNodeAbstract.java` | 普通点位示教 |
| `task/base/linear/WeldLinearTaskNodeAbstract.java` | 直线/圆弧点位 |
| `task/weldtool/WeldToolTaskNodeContribution.java` | 创建焊接任务树 |
| `task/weld/WeldTaskNodeContribution.java` | 普通焊接根节点 |
| `task/weldstart/WeldStartTaskNodeContribution.java` | 起弧、轨迹组装、命令分配 |
| `task/weldshape/WeldShapeTaskNodeContribution.java` | 直线/圆弧焊接段 |
| `task/weldmultipass/WeldMultiPassTaskNodeContribution.java` | 多道复制和偏移 |
| `domain/TrajectoryManager.java` | 摆动轨迹 JSON 和 daemon |
| `domain/NodeContext.java` | 离散点结果 |
| `domain/MultiPathManager.java` | 多道路径中间模型 |
| `utils/MultiPathUtils.java` | 多道偏移算法 |
| `utils/InverseKinematicsUtils.java` | 逆运动学 |
| `utils/MathOffsetUtils.java` | 坐标和 Pose 变换 |
| `XmlRpcDaemonFacade.java` | XML-RPC 通信 |
| `daemon/WeldingDaemonServiceImpl.java` | daemon 安装和启动 |

## 16. 最简调用链

### 普通直线焊接

```text
WeldStartTaskNodeContribution
    -> getBaseCommandList()
        -> MoveLCommand
            -> command()
                -> ScriptWriter
```

### 带摆动直线焊接

```text
WeldStartTaskNodeContribution
    -> createTrajectoryManager()
        -> TrajectoryManager.addLinePoint()
            -> json2String()
                -> XmlRpcDaemonFacade.setDiscreteData()
                    -> NodeContext
                        -> MoveP/MoveL/MoveCCommand
                            -> ScriptWriter
```

### 多层多道

```text
WeldMultiPassTaskNodeContribution
    -> createMultiPathManager()
        -> MultiPathManager.offsetPath()
            -> MultiPathUtils.welding_path_offset_by_section()
                -> setPoseMultiPath()
                    -> 写回各焊道 Pose
                        -> 每一道按普通焊接流程生成脚本
```

## 17. 文档归并说明

本项目学习目录中的文档建议按以下方式管理：

| 原文档 | 处理方式 |
| --- | --- |
| `LEARNING_GUIDE.md` | 内容已吸收到本文，作为项目总览和阅读入口 |
| `焊缝创建流程与点位类说明.md` | 内容已吸收到本文第 4、5、8、10 节 |
| `常用接口及核心轨迹生成说明.md` | 内容已吸收到本文第 5-16 节 |

`FullFunctionWelding` 目录下的文档属于另一个项目，不建议和本文合并：

- `FullFunctionWelding/docs/常用接口归并建议.md`
- `FullFunctionWelding/docs/多层多道左右旋转偏差修复说明.md`

如果后续需要继续拆分，建议只保留两个文件：

1. `WeldingTools学习总览.md`：项目结构、任务树、常用接口和完整调用链。
2. `WeldingTools轨迹算法详解.md`：`TrajectoryManager`、`MultiPathUtils`、坐标变换和 daemon 算法细节。

