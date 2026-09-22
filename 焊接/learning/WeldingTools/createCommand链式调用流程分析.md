# createCommand 链式调用流程分析

> 分析对象：`WeldStartTaskNodeContribution.createCommand()` 及其内部调用链
> 核心方法：`createTrajectoryManager()` + `setLinearCommands()`
> 分析日期：2026-08-25

---

## 一、代码位置

| 类 / 方法 | 文件路径 | 行号 |
|---|---|---|
| `createCommand()` | `src/.../task/weldstart/WeldStartTaskNodeContribution.java` | `:190-195` |
| `getContributionTreeNode()` | 同上 | `:198-201` |
| `createTrajectoryManager()` | 同上 | `:399-448` |
| `setLinearCommands()` | 同上 | `:203-261` |
| `getBaseCommandList()` | 同上 | `:319-397` |
| `getBaseNode3DList()` | 同上 | `:263-312` |
| `TrajectoryManager` | `src/.../domain/TrajectoryManager.java` | 全文件 |
| `WeldCommandAbstract.generateScript()` | `src/.../task/base/command/WeldCommandAbstract.java` | `:229-236` |
| `WeldCommandAbstract.createCommands()` | 同上 | `:57-59` |
| `WeldCommandAbstract.destroyCommands()` | 同上 | `:62-64` |

---

## 二、入口方法

```java
// WeldStartTaskNodeContribution.java:190-195
private void createCommand() {
    TrajectoryManager manager = new TrajectoryManager();
    List<TreeNode> treeNodes = getContributionTreeNode();
    createTrajectoryManager(manager, treeNodes);
    setLinearCommands(manager, treeNodes);
}
```

三步执行：**获取子节点 → 收集几何信息 → 离散化并分发指令**。

---

## 三、完整调用链路图

```
createCommand()                                      // :190-195  入口
  │
  ├─ ① getContributionTreeNode()                    // :198-201  获取子节点列表
  │      └─ rootNode.getChildren()
  │
  ├─ ② createTrajectoryManager(manager, treeNodes)  // :399-448  收集子节点几何信息→组装JSON
  │      │
  │      └─ 遍历子节点 (WeldShapeTaskNodeContribution)
  │           ├─ 有摆动 (baseOscillation != null)
  │           │    ├─ 直线焊缝 → manager.addLinePoint(...)
  │           │    │    └─ TrajectoryManager.points2Json()      // 序列化: 位姿+摆动参数+半径+速度 → JSON
  │           │    │
  │           │    └─ 圆弧焊缝 → manager.addCirclePoint(...)
  │           │         └─ TrajectoryManager.points2Json()      // 同上，额外加 arcAngle
  │           │
  │           └─ 无摆动 → 跳过，不加入 TrajectoryManager
  │
  └─ ③ setLinearCommands(manager, treeNodes)         // :203-261  离散化→构建指令→注入子节点
         │
         ├─ [仅Linux] manager.generateTrajectory()   // TrajectoryManager:114-132
         │      │
         │      ├─ xmlRpcDaemonFacade.setDiscreteData(json)  // XML-RPC 调用本地 daemon (127.0.0.1:4444)
         │      │
         │      └─ parseRetData(retStr)               // :236-255  解析 daemon 返回的离散点
         │           └─ readNode(json)                // :257-306  逐点解析为 NodeContext
         │                └─ poses.add(nodeContext)   // 存入内部列表
         │
         ├─ manager.getDiscretePoint()               // :312-314  返回所有离散点 List<NodeContext>
         │
         ├─ 遍历离散点，按 moveTypeId 构建 BaseCommand
         │      │  并按 taskPointId 分组到 commandMap / node3DMap
         │      │
         │      ├─ moveTypeId == 0 → new MovePCommand(...)   // 过渡运动
         │      ├─ moveTypeId == 1 → new MoveLCommand(...)   // 直线运动
         │      └─ moveTypeId == 2 → new MoveCCommand(...)   // 圆弧运动
         │
         └─ 遍历子节点，注入指令 ──────────────────
                │
                ├─ 有摆动的子节点:
                │    createCommands(commandMap.get(index))     // 注入离散化后的多条指令
                │    setWeldNode(node3DMap.get(index))          // 注入3D显示数据
                │    index++
                │
                └─ 无摆动的子节点:
                     createCommands(getBaseCommandList(contribution))  // 注入单条直接指令
                     setWeldNode(getBaseNode3DList(contribution))       // 注入3D显示数据

                → setBaseWelder(getBaseWelder())   // 注入焊机实例（两个分支都执行）
```

---

## 四、逐步详解

### 4.1 getContributionTreeNode() — 获取子节点列表

```java
// WeldStartTaskNodeContribution.java:198-201
private List<TreeNode> getContributionTreeNode() {
    TreeNode rootNode = this.taskApiProvider.getTaskModel().getContributionTreeNode(this);
    return rootNode.getChildren();
}
```

获取当前起始点节点下的所有子节点（直线焊缝、圆弧焊缝、绑定点等）。

### 4.2 createTrajectoryManager() — 收集几何信息并组装JSON

```java
// WeldStartTaskNodeContribution.java:399-448
private void createTrajectoryManager(TrajectoryManager manager, List<TreeNode> treeNodes) {
    Pose startPose = null;
    for (TreeNode child : treeNodes) {
        // ... 获取 WeldShapeTaskNodeContribution
        BaseOscillation baseOscillation = nodeContribution.getBaseOscillation();
        if (baseOscillation != null) {
            if (startPose == null) {
                startPose = PoseUtils.newPose(nodeContribution.getStartPose());
            }
            if (nodeContribution.getIsLinear()) {
                // 直线焊缝 → addLinePoint
                manager.addLinePoint(baseOscillation, startPose,
                    nodeContribution.getLinearMovePose(),
                    nodeContribution.getBlend(),
                    nodeContribution.getAngleData(),
                    nodeContribution.getTravelSpeed());
                startPose = null;
            } else {
                // 圆弧焊缝 → addCirclePoint
                manager.addCirclePoint(baseOscillation, startPose,
                    nodeContribution.getCircularPassMovePose(),
                    nodeContribution.getCircularEndMovePose(),
                    nodeContribution.getOverrideArcAngle() ? nodeContribution.getArcAngle() : -1,
                    nodeContribution.getBlend(),
                    nodeContribution.getAngleData(),
                    nodeContribution.getTravelSpeed(),
                    nodeContribution.getArcFixedMode());
                // 自定义圆弧角度时，下一个估计起点需要重新计算
                if (nodeContribution.getOverrideArcAngle()) {
                    // ... AngleTransUtils.mathGenArcBy3P 计算圆弧终点
                    startPose = 更新后的终点;
                } else {
                    startPose = null;
                }
            }
        }
    }
}
```

**关键逻辑**：
- `startPose` 在多个焊缝段之间传递，表示当前段的起点 = 上一段的终点
- 只有 `baseOscillation != null`（有摆动）的节点才加入 `TrajectoryManager`
- `addLinePoint` / `addCirclePoint` 内部会先做 `AngleData` 偏移修正，再调用 `points2Json` 序列化为 JSON

### 4.3 TrajectoryManager 内部 — JSON 组装与 daemon 通信

```java
// TrajectoryManager.java

// addLinePoint → points2Json (直线)
public void addLinePoint(BaseOscillation oscillation, Pose beginPoint, Pose endPoint,
                         double radius, AngleData angleData, double speed) {
    // 1. AngleData 偏移修正
    // 2. points2Json(poses, oscillation, jsonObject, radius, radius, speed, false)
}

// addCirclePoint → points2Json (圆弧)
public void addCirclePoint(BaseOscillation oscillation, Pose beginPoint, Pose auxiliaryPoint,
                           Pose endPoint, double arcAngle, double radius,
                           AngleData angleData, double speed, boolean arcFixedMode) {
    // 1. AngleData 偏移修正
    // 2. points2Json(poses, oscillation, jsonObject, radius, arcAngle, speed, isFixed)
}

// JSON 组装
private void points2Json(List<Pose> poses, BaseOscillation oscillation, JsonObject object,
                         double radius, double arcAngle, double speed, boolean isFixed) {
    // 将位姿、摆动参数、半径、速度、圆弧角度等组装为 JsonObject
    // 添加到 jsonObject 的 "trajectory" 数组中
}

// 发送给 daemon 做轨迹离散化
public boolean generateTrajectory() {
    String retStr = xmlRpcDaemonFacade.setDiscreteData(json2String());  // XML-RPC → daemon
    return parseRetData(retStr);
}

// 解析 daemon 返回的离散点
private boolean parseRetData(String ret) {
    // 解析 JSON 返回值
    // 每个 "points" 数组元素 → readNode() → NodeContext
    // 存入 poses 列表
}

// 返回离散点列表
public List<NodeContext> getDiscretePoint() {
    return poses;
}
```

### 4.4 setLinearCommands() — 离散化、构建指令、分发注入

```java
// WeldStartTaskNodeContribution.java:203-261
private void setLinearCommands(TrajectoryManager manager, List<TreeNode> treeNodes) {
    // 1. [仅Linux] 触发 daemon 离散化
    if (SystemUtils.checkSystemIsLinux()) {
        manager.generateTrajectory();
    }

    // 2. 获取离散点列表
    List<NodeContext> poses = manager.getDiscretePoint();

    // 3. 逐点构建 BaseCommand，按 taskPointId 分组
    Map<Integer, ArrayList<BaseCommand>> commandMap = new HashMap<>();
    Map<Integer, ArrayList<Node3DDomain>> node3DMap = new HashMap<>();
    for (NodeContext pose : poses) {
        int taskPointId = pose.getTaskPointId();
        BaseCommand baseCommand;
        if (pose.getMoveTypeId() == 0) {
            baseCommand = new MovePCommand(...);   // 过渡运动
        } else if (pose.getMoveTypeId() == 1) {
            baseCommand = new MoveLCommand(...);   // 直线运动
        } else {
            baseCommand = new MoveCCommand(...);   // 圆弧运动
        }
        baseCommand.setFramePose(pose.getFramePose());
        baseCommand.setAllowableOffset(this.isAllowSearchOffset());
        commandMap.computeIfAbsent(taskPointId, k -> new ArrayList<>()).add(baseCommand);
        node3DMap.computeIfAbsent(taskPointId, k -> new ArrayList<>()).add(node3DDomain);
    }

    // 4. 遍历子节点，注入指令
    int index = 1;
    for (TreeNode child : treeNodes) {
        // ... 获取 WeldShapeTaskNodeContribution
        BaseOscillation baseOscillation = contribution.getBaseOscillation();
        if (baseOscillation != null) {
            // 路径A：有摆动 → 注入离散化后的多条指令
            contribution.createCommands(commandMap.get(index));
            contribution.setWeldNode(node3DMap.get(index));
            index++;
        } else {
            // 路径B：无摆动 → 注入单条直接指令
            contribution.createCommands(getBaseCommandList(contribution));
            contribution.setWeldNode(getBaseNode3DList(contribution));
        }
        contribution.setBaseWelder(getBaseWelder());  // 两个分支都注入焊机实例
    }
}
```

---

## 五、两条分发路径

### 路径 A：有摆动（`baseOscillation != null`）

```
子节点几何信息 → TrajectoryManager JSON 组装
  → daemon 轨迹离散化 → 返回大量离散点
  → 每个离散点变成一个 MoveP/MoveL/MoveC 指令
  → 按 taskPointId 分组
  → createCommands(commandMap.get(index)) 注入对应子节点
```

一个焊缝段会被离散成几十上百个运动指令，实现摆动焊接轨迹（如 Z字形、梯形、月牙形等摆动模式）。

### 路径 B：无摆动（`baseOscillation == null`）

```
子节点位姿 → getBaseCommandList() 直接构建单条指令
  → createCommands(单条指令列表) 注入子节点
```

不走 daemon，不离散化，直接从节点自身的 `startPose` 和 `linearMovePose` / `circularPassMovePose` + `circularEndMovePose` 构建一条 `MoveLCommand` 或 `MoveCCommand`。

### getBaseCommandList() 内部偏移处理

```
getBaseCommandList(contribution)                    // :319-397
  │
  ├─ 直线焊缝:
  │    ├─ 有 AngleData → lineOffset 偏移修正 → new MoveLCommand(偏移后位姿, ...)
  │    └─ 无 AngleData → new MoveLCommand(原位姿, ...)
  │
  └─ 圆弧焊缝:
       ├─ 有 AngleData → circleOffset 三点偏移修正 → new MoveCCommand(偏移后位姿, ...)
       └─ 无 AngleData → new MoveCCommand(原位姿, ...)
```

---

## 六、数据流总结

```
                    ┌─────────────────────────────────────┐
                    │      createTrajectoryManager        │
                    │  遍历子节点 → 收集几何+摆动参数      │
                    │  → TrajectoryManager 内部 JSON 组装  │
                    └───────────────┬─────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────┐
                    │       manager.generateTrajectory()   │
                    │  JSON → XML-RPC → daemon (4444端口)   │
                    │  → 离散化 → 返回 List<NodeContext>    │
                    └───────────────┬─────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────┐
                    │         setLinearCommands           │
                    │  离散点 → BaseCommand (按ID分组)     │
                    │  → createCommands() 注入各子节点      │
                    └───────────────┬─────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────┐
                    │     scriptWriter.writeChildren()     │
                    │  子节点 generateScript 读取 baseCommands │
                    │  → 逐条 command(scriptWriter) 写脚本  │
                    └───────────────┬─────────────────────┘
                                    │
                                    ▼
                    ┌─────────────────────────────────────┐
                    │         destroyCommand()              │
                    │  清空所有子节点的 baseCommands         │
                    └─────────────────────────────────────┘
```

---

## 七、NodeContext 字段说明

daemon 返回的每个离散点 `NodeContext` 包含以下字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `pose` | `Pose` | 目标位姿 |
| `poseVia` | `Pose` | 圆弧中间点（仅 moveTypeId==2） |
| `framePose` | `Pose` | 偏移坐标系（电弧跟踪用） |
| `moveTypeId` | `int` | 0=MoveP, 1=MoveL, 2=MoveC |
| `moveSpeed` | `double` | 运动速度 |
| `moveAcc` | `double` | 加速度 |
| `blendRadius` | `double` | 融合半径 |
| `sleepTime` | `double` | 延时时间 |
| `taskPointId` | `int` | 所属焊缝段编号（用于分组） |

---

## 八、与 generateScript 的时序关系

```
WeldStartTaskNodeContribution.generateScript()      // :150-169
  │
  ├─ weldingIndicator(True)              // 焊接指示信号开
  ├─ getCommand().command(scriptWriter)   // 写入起始点运动指令
  ├─ createCommand()                      // ★ 本文档分析对象
  │    ├─ createTrajectoryManager()       //   收集 → JSON
  │    ├─ [Linux] generateTrajectory()     //   daemon 离散化
  │    └─ setLinearCommands()              //   分发指令到子节点
  ├─ scriptWriter.writeChildren()          // 子节点生成脚本（消费 baseCommands）
  ├─ destroyCommand()                     // 清空子节点的 baseCommands
  ├─ arcOff()                             // 关弧
  └─ weldingIndicator(False)             // 焊接指示信号关
```

---

_本文档由代码静态分析生成，供后续开发参考。_
