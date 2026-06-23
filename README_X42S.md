# X42S 双步进电机升降/夹爪控制

## 硬件连接

- **STM32 USART1** → X42S 驱动器串口（两个电机并接在同一路 485 总线上）
- **STM32 USART3** → 蓝牙模块（手机 app 通信）

## 电机分配

| 地址 | 轴 | 功能 |
|------|-----|------|
| 1 | X 轴 | 升降（正=上升，负=下降） |
| 2 | Y 轴 | 夹爪开闭（正=张开，负=闭合） |

## 上电自检

上电后各轴正反转 3s 验证接线。量产时改 [main.c](Core/Src/main.c) `#if 1` 为 `#if 0` 关闭。

## APP 蓝牙指令

格式：`[gripper,x_speed,y_speed]`

| 指令 | 效果 |
|------|------|
| `[gripper,200,0]` | X 轴以 200 RPM 上升，Y 轴停止 |
| `[gripper,-200,0]` | X 轴以 200 RPM 下降，Y 轴停止 |
| `[gripper,0,150]` | Y 轴以 150 RPM 张开夹爪 |
| `[gripper,0,-150]` | Y 轴以 150 RPM 闭合夹爪 |
| `[gripper,200,100]` | 同时上升 + 张开 |
| `[gripper,0,0]` | 两轴急停 |

**范围：** `-300 ~ 300` RPM

## 操作示例

```
[gripper,0,100]   → 夹爪张开
...放物料...
[gripper,0,-100]  → 夹爪闭合夹紧
[gripper,200,0]   → 上升
[gripper,0,0]     → 到高度急停
[gripper,-200,0]  → 下降
[gripper,0,0]     → 到底急停
[gripper,0,100]   → 夹爪松开
```

## 代码结构

| 文件 | 说明 |
|------|------|
| [motor_driver_X42S.h](Core/Inc/motor_driver_X42S.h) | 宏定义 + 函数声明 |
| [motor_driver_X42S.c](Core/Src/motor_driver_X42S.c) | 串口控制 + 蓝牙 gripper 解析 |
| [bluetooth.c](Core/Src/bluetooth.c) | 蓝牙收发（队友提供） |
| [app_control.c](Core/Src/app_control.c) | 麦轮/绘图控制（队友提供，不改动） |
