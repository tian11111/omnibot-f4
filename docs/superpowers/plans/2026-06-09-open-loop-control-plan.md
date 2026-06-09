# 开环控制实施计划

> **对于智能体工作者：** 必需的子技能：使用superpowers:subagent-driven-development（推荐）或superpowers:executing-plans来逐个任务实施此计划。步骤使用复选框（`- [ ]`）语法进行跟踪。

**目标：** 修改麦克纳姆轮小车的控制代码，从闭环PID控制改为开环PWM控制，让电机能够立即转起来调试硬件。

**架构：** 修改`app_control.c`中的`Mecanum_SetMotion`和`Mecanum_StopAll`函数，直接调用`DC4_Motor_SetSignedSpeed`，绕过PID闭环。注释掉`main.c`中的`MotorClosedLoop_Update()`调用。

**技术栈：** STM32 HAL库，C语言，TB6612电机驱动

---

## 文件结构

**修改的文件：**
- `Core/Src/app_control.c` - 修改运动控制函数，直接调用电机驱动
- `Core/Src/main.c` - 注释掉闭环更新调用

**不需要修改的文件：**
- `Core/Src/motor_closedloop.c` - 保留，便于后续恢复闭环
- `Core/Src/motor_driver_dc4ch.c` - 电机驱动层，不需要修改
- `Core/Inc/motor_closedloop.h` - 头文件保留

---

### 任务1：修改Mecanum_SetMotion函数

**文件：**
- 修改：`Core/Src/app_control.c:103-137`

- [ ] **步骤1：读取当前Mecanum_SetMotion函数**

读取`Core/Src/app_control.c`文件，查看第103-137行的`Mecanum_SetMotion`函数。

- [ ] **步骤2：修改函数实现**

将第127-131行的闭环控制代码：
```c
/* 设置PID目标速度，由MotorClosedLoop_Update()闭环控制 */
MotorClosedLoop_SetTargetSpeed(0, (int32_t)fl);
MotorClosedLoop_SetTargetSpeed(1, (int32_t)fr);
MotorClosedLoop_SetTargetSpeed(2, (int32_t)bl);
MotorClosedLoop_SetTargetSpeed(3, (int32_t)br);
```

修改为开环控制代码：
```c
/* 开环控制：直接设置电机PWM，绕过PID闭环 */
DC4_Motor_SetSignedSpeed(0, (int16_t)fl);
DC4_Motor_SetSignedSpeed(1, (int16_t)fr);
DC4_Motor_SetSignedSpeed(2, (int16_t)bl);
DC4_Motor_SetSignedSpeed(3, (int16_t)br);
```

- [ ] **步骤3：验证修改**

检查修改后的代码：
1. 确认使用了`DC4_Motor_SetSignedSpeed`函数
2. 确认参数类型为`int16_t`
3. 确认注释已更新

- [ ] **步骤4：提交修改**

```bash
git add Core/Src/app_control.c
git commit -m "[功能] 修改Mecanum_SetMotion为开环控制"
```

---

### 任务2：修改Mecanum_StopAll函数

**文件：**
- 修改：`Core/Src/app_control.c:139-145`

- [ ] **步骤1：读取当前Mecanum_StopAll函数**

读取`Core/Src/app_control.c`文件，查看第139-145行的`Mecanum_StopAll`函数。

- [ ] **步骤2：修改函数实现**

将第141-144行的闭环控制代码：
```c
MotorClosedLoop_SetTargetSpeed(0, 0);
MotorClosedLoop_SetTargetSpeed(1, 0);
MotorClosedLoop_SetTargetSpeed(2, 0);
MotorClosedLoop_SetTargetSpeed(3, 0);
```

修改为开环控制代码：
```c
DC4_Motor_SetSignedSpeed(0, 0);
DC4_Motor_SetSignedSpeed(1, 0);
DC4_Motor_SetSignedSpeed(2, 0);
DC4_Motor_SetSignedSpeed(3, 0);
```

- [ ] **步骤3：验证修改**

检查修改后的代码：
1. 确认使用了`DC4_Motor_SetSignedSpeed`函数
2. 确认参数为0

- [ ] **步骤4：提交修改**

```bash
git add Core/Src/app_control.c
git commit -m "[功能] 修改Mecanum_StopAll为开环控制"
```

---

### 任务3：注释掉主循环中的闭环更新

**文件：**
- 修改：`Core/Src/main.c:148`

- [ ] **步骤1：读取当前main.c主循环**

读取`Core/Src/main.c`文件，查看第138-150行的主循环。

- [ ] **步骤2：注释掉闭环更新调用**

将第148行的闭环更新调用：
```c
/* 更新闭环控制（10ms周期） */
MotorClosedLoop_Update();
```

修改为：
```c
/* 开环控制：不需要闭环更新 */
// MotorClosedLoop_Update();
```

- [ ] **步骤3：验证修改**

检查修改后的代码：
1. 确认`MotorClosedLoop_Update()`调用已被注释
2. 确认注释清晰说明了原因

- [ ] **步骤4：提交修改**

```bash
git add Core/Src/main.c
git commit -m "[功能] 注释掉闭环更新调用"
```

---

### 任务4：测试开环控制

**文件：**
- 测试：实际硬件测试

- [ ] **步骤1：编译程序**

使用Keil MDK-ARM或IAR EWARM编译程序，确保没有编译错误。

- [ ] **步骤2：烧录程序**

将编译后的程序烧录到STM32开发板。

- [ ] **步骤3：测试电机转动**

1. 打开蓝牙串口APP
2. 连接HC-05/06蓝牙模块
3. 发送命令`[joystick,0,50,0,0]`让车前进
4. 观察电机是否转动
5. 用万用表测量PWM引脚电压（PE9、PE11、PE13、PE14）

- [ ] **步骤4：测试其他方向**

1. 测试后退：`[joystick,0,-50,0,0]`
2. 测试右平移：`[joystick,50,0,0,0]`
3. 测试原地右转：`[joystick,0,0,50,0]`
4. 测试停止：`[joystick,0,0,0,0]`

- [ ] **步骤5：记录测试结果**

记录每个方向的测试结果：
1. 电机是否转动
2. PWM引脚电压
3. 蓝牙控制响应
4. 停止命令有效性

---

## 验收标准

1. 电机能够正常转动
2. PWM引脚有电压输出
3. 蓝牙控制响应正常
4. 停止命令有效
5. 代码修改清晰，便于后续恢复闭环控制

## 后续步骤

调试完成后，恢复闭环控制：
1. 恢复`Mecanum_SetMotion`函数中的`MotorClosedLoop_SetTargetSpeed`调用
2. 恢复`Mecanum_StopAll`函数中的`MotorClosedLoop_SetTargetSpeed`调用
3. 恢复`main.c`中的`MotorClosedLoop_Update()`调用