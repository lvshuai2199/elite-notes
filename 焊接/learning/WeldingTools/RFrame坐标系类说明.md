# RFrame 坐标系类说明

## 1. 说明范围

本文说明 WeldingTools 数学库中 `RFrame` 类：

- `RFrame` 是什么，成员结构对应什么数学概念。
- 为什么三维运算需要它（对比机器人 SDK 的 `Pose` / 欧拉角）。
- 核心变换 API 及数学原理。
- 在焊接插件（多层多道偏移、角度偏移、特征点定位、几何求交）中的实际用途。

一句话总结：**`RFrame` 是机器人学里的"参考坐标系"，等价于一个 4×4 齐次变换矩阵。它把姿态从"位置+欧拉角"（Pose）转换成"原点+三个正交单位轴方向向量"，让三维几何运算更直观、更稳定。**

## 2. RFrame 是什么

文件：`src/main/java/cn/elibot/plugin/processes/weldingtools/impl/math/RFrame.java`

### 2.1 成员结构

```java
public class RFrame {
    private Point3d origin;    // 坐标系原点位置
    private Vector3d xDir;     // X 轴单位方向向量
    private Vector3d yDir;     // Y 轴单位方向向量
    private Vector3d zDir;     // Z 轴单位方向向量
    private double scale;      // 缩放因子（本工程恒为 1）
}
```

### 2.2 等价于 4×4 齐次变换矩阵

`RFrame` 等价于如下矩阵（`Math3D.mathFrameToTransform3D()`，`Math3D.java:232`）：

```
| xDir.x  yDir.x  zDir.x  origin.x |
| xDir.y  yDir.y  zDir.y  origin.y |
| xDir.z  yDir.z  zDir.z  origin.z |
|   0       0       0      scale   |
```

- 左上 3×3：旋转矩阵，三个**列**向量就是 x / y / z 轴方向（也是旋转矩阵 R 的列）。
- 右侧一列：平移 t（原点位置）。
- 因此 `RFrame` 与 Java3D `Transform3D` 可以无损互转。

### 2.3 默认构造与正交重建

- 默认构造（`RFrame.java:16-22`）：原点 `(0,0,0)`，三轴分别对齐世界坐标轴，scale = 1，即**单位坐标系**。
- `rebuildFrame()`（`RFrame.java:94-113`）：
  1. 用 `zDir = xDir × yDir` 叉积算 Z 轴；
  2. 若 zDir 长度过小说明 x/y 平行或共线，返回 false；
  3. 归一化 xDir 和 zDir，再用 `yDir = zDir × xDir` 恢复 Y 轴；
  4. 最终保证三个轴**单位且两两正交**。

这是"施密特正交化"思路——即使外部传入的 x/y 方向不完全正交，也能重建出一个标准正交坐标系。欧拉角表示做不到这种数值自愈。

## 3. 为什么需要 RFrame（与 Pose 对比）

机器人 SDK 的 `Pose` 是 `[x, y, z, rx, ry, rz]`（位置 mm + 欧拉角 rad）。直接拿 Pose 做三维几何运算有几个问题：

| 问题 | 说明 |
| --- | --- |
| 欧拉角奇异性（万向锁） | 某些姿态下 rx/ry/rz 无法唯一描述或剧烈跳变 |
| 姿态插值/旋转不直观 | 绕任意轴旋转、姿态叠加都要先转成矩阵 |
| 单位混乱 | 位置用 mm、角度用 rad，运算时常忘记换算 |
| 缺少坐标系"框架"概念 | 用户坐标系、工具坐标系、焊缝坐标系都需要一个完整的 frame 描述 |

`RFrame` 把姿态统一成"原点 + 三个正交单位轴"，几何意义直接对应机器人的坐标系，运算稳定，还能方便地做**坐标系间变换**。

## 4. 核心变换 API（Math3D）

`src/main/java/cn/elibot/plugin/processes/weldingtools/impl/math/Math3D.java`

### 4.1 Pose ↔ RFrame

| 方法 | 作用 |
| --- | --- |
| `mathEulerToFrame(double[] euler)` | Pose `[x,y,z,rx,ry,rz]` → RFrame（第 377 行） |
| `mathFrameToEuler(RFrame)` | RFrame → Pose（第 308 行） |
| `mathFrameToEulerArray(RFrame)` | RFrame → 6 个 double（第 335 行） |

欧拉转 RFrame 用的是**固连坐标系 Z-Y-X 欧拉角**构造旋转矩阵：

```java
RFrame.getxDir().setX(c[2] * c[1]);                    // 第 392 行
RFrame.getyDir().setX(c[2] * s[1] * s[0] - s[2] * c[0]);
RFrame.getzDir().setX(c[2] * s[1] * c[0] + s[2] * s[0]);
```

反向提取欧拉角（`mathFrameToEuler`，第 324-330 行）：

```java
double Beta  = Math.atan2(-r31, Math.sqrt(r11 * r11 + r21 * r21));
double Alpha = Math.atan2(r21 / Math.cos(Beta), r11 / Math.cos(Beta));
double Gamma = Math.atan2(r32 / Math.cos(Beta), r33 / Math.cos(Beta));
```

### 4.2 点/矢量：世界 ↔ 局部

| 方法 | 数学含义 |
| --- | --- |
| `mathTransWorldPoint3d(frame, worldPt)` | 世界点 → 局部点：先减原点，再乘旋转矩阵（第 27 行） |
| `mathTransLocalPoint3d(frame, localPt)` | 局部点 → 世界点：先乘旋转矩阵，再加原点（第 98 行） |
| `mathTransWorldVector3d(frame, worldVec)` | 世界矢量 → 局部矢量：只旋转不平移（第 53 行） |
| `mathTransLocalVector3d(frame, localVec)` | 局部矢量 → 世界矢量：只旋转不平移（第 127 行） |

以 `mathTransWorldPoint3d` 为例：

```java
transPoint = worldPoint - frame.origin;                 // 平移
localPoint.x = transPoint · frame.xDir;                 // 旋转（点积=投影）
localPoint.y = transPoint · frame.yDir;
localPoint.z = transPoint · frame.zDir;
```

**矢量变换没有平移项**（矢量无位置，只有方向），所以只做旋转。

### 4.3 坐标系：世界 ↔ 局部

| 方法 | 数学含义 |
| --- | --- |
| `mathTransWorldFrame(localFrame, inWorldFrame)` | 世界坐标系 → 局部坐标系（第 74 行） |
| `mathTransLocalFrame(localFrame, inLocalFrame)` | 局部坐标系 → 世界坐标系（第 156 行） |

原理：把目标坐标系的原点当点变换、三个轴当矢量变换，得到新坐标系的 4 个要素。

### 4.4 其它工具

| 方法 | 作用 |
| --- | --- |
| `mathRotateFrame(pivot, axis, angle, frame)` | 坐标系绕任意轴旋转（第 276 行） |
| `mathFrameToTransform3D(frame)` | RFrame → Java3D Transform3D（第 232 行） |
| `mathTransform3DToFrame(t3d)` | Transform3D → RFrame（第 216 行） |
| `mathCalTransformByOXY(origin, xAxis, yDir)` | 用原点 + X 轴点 + Y 轴点构造坐标系（第 179 行） |

## 5. 在焊接插件中的实际业务场景

### 5.1 场景一：用户坐标系下的焊缝偏移（多层多道核心）

文件：`MultiPathUtils.weldingPolygonPathOffset()`（`MultiPathUtils.java:33`）

```text
1. 用户坐标系 Pose → RFrame（mathEulerToFrame）
2. 世界坐标路径点 → 用户坐标系局部点（mathTransWorldPoint3d）
3. 投影到用户坐标系 XY 平面（zDir = (0,0,1)）
4. 在平面内做左右偏移（offsetLineByFrame，用 frame.zDir 叉积求偏移方向）
5. 补偿 Z 高度（zOffsetDist）
6. 偏移后的局部点 → 世界坐标（mathTransLocalPoint3d）
```

关键代码（`MultiPathUtils.java:44`、`92`）：

```java
pt_list_frame.add(Math3D.mathTransWorldPoint3d(userFrame, ptList.get(i)));  // 世界 → 用户
...
Point3d point3d = Math3D.mathTransLocalPoint3d(userFrame, proj_offset_pt_list_in_frame.get(i)); // 用户 → 世界
```

这就是"把焊缝往用户坐标系左右偏一定距离"生成新焊道的标准流程。`MultiPathSection.offsetSectionPoints()`（`MultiPathSection.java:125`）里也用了同样模式。

### 5.2 场景二：工作角/前进角姿态偏移

文件：`OffsetAngle.java`

`OffsetAngle` 把示教点位和用户坐标系都转成 RFrame：

```java
this.orgPoint = Math3D.mathEulerToFrame(orgPoint.toArray(...));  // OffsetAngle.java:19
this.frame    = Math3D.mathEulerToFrame(frame.toArray(...));     // OffsetAngle.java:20
```

然后在用户坐标系下按 `workAng`（工作角）、`forwardAng`（前进角）旋转出偏移后的姿态 `offsetPoint`，最后 `Math3D.mathFrameToEuler(offsetPoint)`（`OffsetAngle.java:51`）转回 Pose 供运动指令使用。

（`TrajectoryManager` 中 `createOffsetAngle()` + `AngleTransUtils.lineOffset()/circleOffset()` 就是消费这个偏移姿态的地方。）

### 5.3 场景三：三点定位 / 特征点点位搬移

文件：`MathOffsetUtils.featurePointOffset()`（`MathOffsetUtils.java:50`）

```text
1. 原三个特征点 → orgFrame（calFrameBy3p）
2. 新三个特征点 → newFrame
3. 待搬移点 world → orgFrame 局部（mathTransWorldFrame 后取局部坐标）
4. 局部点 → newFrame 世界（mathTransLocalFrame）
```

这实现"同一特征结构从位置 A 搬到位置 B，点位跟着走"。`calOffsetTransform()`（`MathOffsetUtils.java:66`）则是把两个坐标系转成 `Transform3D` 求增量变换：

```java
Transform3D offsetTrans = new Transform3D();
offsetTrans.mul(newTrans, orgTrans);   // 新坐标系 × 旧坐标系⁻¹
```

### 5.4 场景四：几何求交（圆/直线/平面）

文件：`MathIntersection.java`

- `mathIntCirLin(RFrame pLF, double r, ...)`：`pLF` 是圆的局部坐标系（原点=圆心，XY 平面=圆平面），求直线与圆交点（第 183 行）。
- `mathIntCirPln(RFrame pLF, double r, ...)`：求平面与圆交点（第 236 行）。
- `mathIntCir(RFrame pLF1, r1, RFrame pLF2, r2, ...)`：两个圆各自用 RFrame 表示，求两圆交点（第 270 行）。

圆弧的几何特征也是用 RFrame 承载：`AngleTransUtils.ArcFeatur` 内部字段 `private RFrame lf`（`AngleTransUtils.java:394`），记录圆弧所在坐标系。

### 5.5 场景五：矢量在用户坐标系下的方向换算

文件：`MathOffsetUtils.java`

- `getVecInUserFrame()`（第 153、166 行）：把基座坐标系下的前进矢量转成用户坐标系下的矢量。
- `calOffsetTransform(Pose userFrame, Vector3d vectorInUser)`（第 119 行）：把用户坐标系下的偏移矢量转回基座后构造 Transform3D。

这些用于"用户输入的方向/偏移量"与"机器人基座坐标"之间的桥接。

## 6. 代码位置速查

| 内容 | 位置 |
| --- | --- |
| `RFrame` 类定义 | `math/RFrame.java` |
| 坐标系变换主工具 | `math/Math3D.java` |
| 圆弧特征（含 RFrame 坐标系） | `utils/AngleTransUtils.java:382`（`ArcFeatur`） |
| 角度偏移（工作角/前进角） | `math/OffsetAngle.java` |
| 三点定位 / 点位搬移 | `utils/MathOffsetUtils.java:50` |
| 用户坐标系下路径偏移 | `utils/MultiPathUtils.java:33` |
| 分段偏移 | `domain/MultiPathSection.java:125` |
| 几何求交（圆/直线/圆） | `math/MathIntersection.java` |

## 7. 关键结论

1. **`RFrame` = 坐标系 = 4×4 齐次变换矩阵**，用"原点 + 三个正交单位轴"表达，比 Pose（欧拉角）更稳定、更直观。
2. **一切三维变换都围绕它展开**：`Math3D` 提供了点、矢量、坐标系三级的双向变换（世界 ↔ 局部）。
3. **单位约定**：`RFrame` 内部的点/向量通常以 mm 为单位；文档中的旋转角用弧度。Pose ↔ RFrame 转换时用 `Length.Unit.MM, Angle.Unit.RAD`。
4. **`rebuildFrame()` 保证坐标系始终正交**，这是 RFrame 相比欧拉角的关键数值优势。
5. 焊接插件里 RFrame 的核心价值在于**"坐标系思维"**：所有偏移（多道、角度、特征点）都是在某个参考坐标系内先变换、再转回基座，而不是直接对裸 Pose 做加减法。
