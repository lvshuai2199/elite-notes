# TrajectoryManager 轨迹数据流说明

## 1. 说明范围

本文说明 WeldingTools 项目中 `TrajectoryManager` 的完整数据流：

- `points2Json` 为什么没有返回值，数据存放在哪里。
- 轨迹 JSON 如何从构建、发送、解析到最终生成运动命令。
- 涉及的 XML-RPC 守护进程交互（`127.0.0.1:4444`）。

一句话总结：`points2Json` 不返回数据，而是通过**引用传递**把每条轨迹追加到成员变量 `jsonObject` 的 `trajectory` 数组中；该数组在 `generateTrajectory()` 时被序列化发送给守护进程计算离散点，结果解析进 `poses` 列表，最终由 `WeldStartTaskNodeContribution.setLinearCommands()` 消费生成运动命令。

## 2. 相关核心类

| 作用 | 类 |
| --- | --- |
| 轨迹数据管理（本文主角） | `TrajectoryManager` |
| 轨迹发送入口 | `WeldStartTaskNodeContribution` |
| XML-RPC 客户端 | `XmlRpcDaemonFacade` |
| 摆动基类 | `BaseOscillation` |
| 摆动类型 | `ZigzagOscillation`、`TrapezoidOscillation`、`CrescentOscillation`、`CircleOscillation` |
| 角度偏移计算 | `OffsetAngle`、`AngleTransUtils` |
| 离散点数据模型 | `NodeContext` |

## 3. TrajectoryManager 的两个关键成员变量

`src/main/java/cn/elibot/plugin/processes/weldingtools/impl/domain/TrajectoryManager.java`

```java
// 第 45 行：轨迹 JSON 的累积容器（构建阶段）
private final JsonObject jsonObject = new JsonObject();
// 第 48 行：守护进程计算后的离散点列表（消费阶段）
private final List<NodeContext> poses = new ArrayList<>();
```

两个成员变量对应两个阶段：

- `jsonObject`：**构建阶段**使用，`points2Json` 往这里累积原始轨迹数据。
- `poses`：**消费阶段**使用，`parseRetData` 往这里填充守护进程算出的离散点。

## 4. points2Json 为什么不返回

`points2Json` 是 `void` 方法（第 134 行），但它操作的 `object` 参数就是成员变量 `jsonObject`。

关键逻辑（`TrajectoryManager.java:139-160`）：

```java
JsonArray array = object.getAsJsonArray(TRAJECTORY);
if (array == null || array.isJsonNull()) {
    array = new JsonArray();
    object.add(TRAJECTORY, array);        // 把数组挂到 jsonObject 上
}
...
JsonObject pointObj = new JsonObject();
oscillation2Json(oscillation, pointObj, speed / 1000);
...
pointObj.add(POINTS, poseArray);
array.add(pointObj);                      // 追加一条轨迹（第 160 行）
```

理解要点：

1. **不是没有存数据**，而是通过引用把数据写进了 `jsonObject` 这个成员变量。
2. `array` 取自 `jsonObject` 的 `"trajectory"` 键，不存在时新建并 `add` 回去，所以后续调用拿到的都是同一个数组。
3. 每次 `addLinePoint()` / `addCirclePoint()` 调用，就往 `trajectory` 数组追加一个 `pointObj`。调用 N 次，数组里就有 N 条轨迹。
4. 因此 `points2Json` 不需要返回值——它是"收集状态"而非"纯计算"函数。

## 5. 完整数据流

```
addLinePoint() / addCirclePoint()           WeldStartTaskNodeContribution.createTrajectoryManager()
        │
        ▼
points2Json()                               累积进 jsonObject.trajectory（成员变量，不返回）
        │
        ▼
generateTrajectory()                        ① 启动守护进程（如需）
        │  json2String() → xmlRpcDaemonFacade.setDiscreteData(...)  发送到 127.0.0.1:4444
        ▼
（守护进程计算离散点并返回 JSON）
        │
        ▼
parseRetData()                              解析返回，填充成员变量 poses (List<NodeContext>)
        │
        ▼
getDiscretePoint()                          返回 poses
        │
        ▼
setLinearCommands()                         遍历 poses → MoveP/MoveL/MoveCCommand
        │
        ▼
各 WeldShapeTaskNodeContribution.createCommands() → generateScript() 生成机器人脚本
```

## 6. 各阶段详解

### 6.1 阶段一：数据构建

**入口：** `WeldStartTaskNodeContribution.createTrajectoryManager()`（约第 399-448 行）

遍历任务树子节点，对每个 `WeldShapeTaskNodeContribution`：

- 直线段：`manager.addLinePoint(baseOscillation, startPose, linearMovePose, blend, angleData, travelSpeed)`（第 413 行）。
- 圆弧段：`manager.addCirclePoint(baseOscillation, startPose, passMovePose, endMovePose, arcAngle, blend, angleData, travelSpeed, arcFixedMode)`（第 421 行）。

`TrajectoryManager.addLinePoint()`（第 51-69 行）内部：

1. 构造 `poses` 列表，加入起点、终点。
2. 若 `angleData` 有坐标系，通过 `createOffsetAngle()` + `AngleTransUtils.lineOffset()` 计算角度偏移，替换为偏移后的 Pose。
3. 调用 `points2Json(...)`。

`TrajectoryManager.addCirclePoint()`（第 77-103 行）同理，poses 加入三点（起点、辅助点、终点），并多出 `arcAngle` 参数。

**构建结果**：`jsonObject.trajectory` 数组，每个元素结构：

```json
{
  "type": "zigzag",                    // 摆动类型 id
  "speed": 0.005,                      // 速度，m/s
  "parameters": {                      // 摆动参数，随类型不同
    "period": 0.5,
    "amplitude": 0.003,
    "leftStayTime": 0.1,
    "rightStayTime": 0.1,
    "angle": 1.5708,
    "centerStayTime": 0.0
  },
  "radius": 0.003,                     // 可选，半径 >= 0 时写入（m）
  "isFixed": true,                     // 可选，圆弧固定模式
  "arcAngle": 3.1416,                  // 可选，poses.size() > 2 时写入（rad）
  "points": [                          // 点位数组
    [x, y, z, rx, ry, rz],            // Pose，单位 m / rad
    [x, y, z, rx, ry, rz]
  ]
}
```

注意单位换算：

- 速度：`speed / 1000`（mm/s → m/s）。
- 半径：`radius / 1000D`（mm → m）。
- 幅度：`amplitude / 1000D`（mm → m）。
- 圆弧角：`arcAngle / 180D * PI`（度 → 弧度）。

### 6.2 阶段二：发送守护进程

`generateTrajectory()`（第 114-132 行）：

```java
if (!xmlRpcDaemonFacade.isStarted()) {
    xmlRpcDaemonFacade.start();
}
retStr = xmlRpcDaemonFacade.setDiscreteData(json2String());  // 第 121 行
```

- `json2String()`（第 308-310 行）返回 `jsonObject.toString()`，即整个 `trajectory` 数组。
- 通过 XML-RPC 发送到 `127.0.0.1:4444` 的守护进程，由守护进程负责离散化计算（把带摆动的轨迹切成实际运动点）。

### 6.3 阶段三：解析返回

`parseRetData()`（第 236-255 行）：

1. `poses.clear()` 清空上次结果。
2. 返回 JSON 含 `"error"` 键 → 失败。
3. 返回 JSON 含 `"status":"ok"` → 取出 `"points"` 数组，逐个 `readNode()` 解析为 `NodeContext` 存入 `poses`。

`readNode()`（第 257-306 行）解析每个点：

- 通用字段：`acc`、`index`、`moveType`、`radius`、`sleepTime`、`speed`、`pose`、`offsetFrame`。
- `moveType == 2`（圆弧）时额外读取 `poseVia` 中间点，构造带 via 点的 `NodeContext`。

### 6.4 阶段四：生成运动命令

`WeldStartTaskNodeContribution.setLinearCommands()`（约第 203-235 行）：

```java
if (SystemUtils.checkSystemIsLinux()) {
    manager.generateTrajectory();              // 第 205 行
}
List<NodeContext> poses = manager.getDiscretePoint();  // 第 209 行
for (NodeContext pose : poses) {
    // moveTypeId == 0 → MovePCommand
    // moveTypeId == 1 → MoveLCommand
    // moveTypeId == 2 → MoveCCommand（带 poseVia）
}
```

按 `taskPointId` 分组存入 `commandMap` / `node3DMap`，后续由各 `WeldShapeTaskNodeContribution.createCommands()` 绑定摆动命令，最终经 `generateScript()` 输出机器人脚本。

注意：`generateTrajectory()` 只在 Linux 环境调用（第 204 行 `checkSystemIsLinux()`），示教器运行环境是 Linux；非 Linux 环境（如 Windows 开发调试）下 `poses` 保持为空。

## 7. 调用关系速查

| 方法 | 位置 | 作用 |
| --- | --- | --- |
| `addLinePoint(...)` | `TrajectoryManager.java:51` | 添加直线轨迹 |
| `addCirclePoint(...)` | `TrajectoryManager.java:77` | 添加圆弧轨迹 |
| `points2Json(...)` | `TrajectoryManager.java:134` | 累积到 `jsonObject.trajectory` |
| `generateTrajectory()` | `TrajectoryManager.java:114` | 发送 + 解析 |
| `parseRetData()` | `TrajectoryManager.java:236` | 解析离散点到 `poses` |
| `getDiscretePoint()` | `TrajectoryManager.java:312` | 返回离散点列表 |
| `json2String()` | `TrajectoryManager.java:308` | 序列化轨迹 JSON |

| 调用点 | 位置 |
| --- | --- |
| `createTrajectoryManager()` 构建轨迹 | `WeldStartTaskNodeContribution.java:399` |
| `setLinearCommands()` 消费离散点 | `WeldStartTaskNodeContribution.java:203` |
| 多层多道里的轨迹构建 | `WeldMultiPassTaskNodeContribution.java:579` |
| 单元测试示例 | `test/.../GenerateTrajectoryTest.java:22` |

## 8. 关键结论

1. **`points2Json` 不返回数据**：它通过引用把数据写入成员变量 `jsonObject.trajectory`，这是"收集状态"的写法，不是纯函数。
2. **轨迹 JSON 只在 `generateTrajectory()` 时被序列化发送**，在此之前数据只存在于 `jsonObject` 中。
3. **守护进程返回的离散点**解析进 `poses`，通过 `getDiscretePoint()` 暴露给上层。
4. **最终消费者是 `setLinearCommands()`**，它把离散点转成 `MoveP/MoveL/MoveCCommand`，进而生成 EliScript 机器人脚本。
