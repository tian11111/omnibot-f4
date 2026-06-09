# 开环控制设计文档

## 问题描述

当前闭环控制存在问题：电机不转，PWM引脚没有电压（用万用表测量）。需要先让电机转起来调试硬件。

## 设计目标

1. 修改为开环控制，让电机立即转起来
2. 保持原有代码结构，便于后续恢复闭环控制
3. 确保安全，防止电机失控

## 设计方案

### 方案选择

采用方案A：修改`app_control.c`中的`Mecanum_SetMotion`函数，直接调用`DC4_Motor_SetSignedSpeed`，绕过PID闭环。

**优点：**
- 修改简单，只需修改几行代码
- 不影响其他模块
- 便于后续恢复闭环控制

**缺点：**
- 没有速度反馈，无法保证速度精度
- 电机特性差异可能导致跑偏

### 具体修改

#### 1. 修改`Mecanum_SetMotion`函数

**文件：** `Core/Src/app_control.c`
**行号：** 103-137

**修改前：**
```c
/* 设置PID目标速度，由MotorClosedLoop_Update()闭环控制 */
MotorClosedLoop_SetTargetSpeed(0, (int32_t)fl);
MotorClosedLoop_SetTargetSpeed(1, (int32_t)fr);
MotorClosedLoop_SetTargetSpeed(2, (int32_t)bl);
MotorClosedLoop_SetTargetSpeed(3, (int32_t)br);
```

**修改后：**
```c
/* 开环控制：直接设置电机PWM，绕过PID闭环 */
DC4_Motor_SetSignedSpeed(0, (int16_t)fl);
DC4_Motor_SetSignedSpeed(1, (int16_t)fr);
DC4_Motor_SetSignedSpeed(2, (int16_t)bl);
DC4_Motor_SetSignedSpeed(3, (int16_t)br);
```

#### 2. 修改`Mecanum_StopAll`函数

**文件：** `Core/Src/app_control.c`
**行号：** 139-145

**修改前：**
```c
void Mecanum_StopAll(void)
{
    MotorClosedLoop_SetTargetSpeed(0, 0);
    MotorClosedLoop_SetTargetSpeed(1, 0);
    MotorClosedLoop_SetTargetSpeed(2, 0);
    MotorClosedLoop_SetTargetSpeed(3, 0);
}
```

**修改后：**
```c
void Mecanum_StopAll(void)
{
    DC4_Motor_SetSignedSpeed(0, 0);
    DC4_Motor_SetSignedSpeed(1, 0);
    DC4_Motor_SetSignedSpeed(2, 0);
    DC4_Motor_SetSignedSpeed(3, 0);
}
```

#### 3. 注释掉主循环中的闭环更新

**文件：** `Core/Src/main.c`
**行号：** 148

**修改前：**
```c
/* 更新闭环控制（10ms周期） */
MotorClosedLoop_Update();
```

**修改后：**
```c
/* 开环控制：不需要闭环更新 */
// MotorClosedLoop_Update();
```

### 安全考虑

1. PWM限幅：`DC4_Motor_SetSignedSpeed`函数内部已有限幅，确保PWM在-100~100范围内
2. 紧急停止：保留蓝牙停止命令功能
3. 调试输出：保留调试信息，便于观察电机状态

### 测试步骤

1. 烧录修改后的程序
2. 发送蓝牙命令`[joystick,0,50,0,0]`让车前进
3. 观察电机是否转动
4. 用万用表测量PWM引脚电压（应有变化）
5. 测试其他方向：后退、平移、旋转

### 后续恢复闭环

调试完成后，恢复闭环控制：
1. 恢复`Mecanum_SetMotion`函数中的`MotorClosedLoop_SetTargetSpeed`调用
2. 恢复`Mecanum_StopAll`函数中的`MotorClosedLoop_SetTargetSpeed`调用
3. 恢复`main.c`中的`MotorClosedLoop_Update()`调用

## 验收标准

1. 电机能够正常转动
2. PWM引脚有电压输出
3. 蓝牙控制响应正常
4. 停止命令有效