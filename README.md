# explorer_26 工程说明

本文档是 `26---main` 工程的总入口说明，用来帮助后续调车、改代码、查问题时快速定位模块。工程基于 STM32F750 + FreeRTOS，核心目标是控制四轮底盘完成探险路线：启动准备、循线行驶、节点识别、障碍处理、陀螺仪直行、平台调头、停车保护等。

## 工程定位

- 主控芯片：STM32F750V8Tx。
- 实时系统：FreeRTOS / CMSIS-RTOS 相关代码由 CubeMX 工程集成。
- 主要控制周期：底盘电机任务按 5ms 周期运行。
- 主要输入：16 路循迹灯、IMU、四路编码器、红外/颜色等辅助信号。
- 主要输出：四路电机 PWM，串口调试输出。
- 推荐开发入口：业务层看 `App/` 和 `Task/`，硬件驱动看 `Motor/`、`Sensor/`、`Driver/`，CubeMX 生成代码尽量不手改。

## 工程保护

- 本工程有三个保护机制
  巡线阶段丢线超过，陀螺仪roll角短时间变换45°(侧翻)，yaw角短时间变化360°(各种意外导致的乱转)
  
## 目录结构

| 路径 | 作用 |
|---|---|
| `Core/` | CubeMX 生成的 HAL 初始化、主函数、外设初始化、中断入口。业务代码只放在 USER CODE 区。 |
| `Drivers/` | STM32 HAL 和 CMSIS 官方驱动。通常不改。 |
| `Middlewares/` | FreeRTOS 源码和 CMSIS-RTOS 适配层。通常不改。 |
| `App/chassis/` | 底盘对外 API 层，封装模式切换、刹车、里程、阻塞转弯、坡道控制、安全保护。 |
| `App/map/` | 地图、节点、路线、路口到达判断、`Cross()` 节点间状态机。 |
| `App/barrier/` | 障碍物流程：准备流程、平台、桥、坡/山地、波浪板等。 |
| `Task/` | FreeRTOS 任务创建、启动初始化、主控任务、电机周期任务、转弯/陀螺仪控制。 |
| `Motor/` | 电机 PWM 输出、编码器读取与里程清零。 |
| `Sensor/` | 16 路循迹灯、IMU、部分传感器适配。 |
| `Driver/` | 通用驱动和工程配置：延时、调试串口、底盘状态数据、常量配置等。 |
| `Math/` | PID、滤波、正弦生成等数学工具。 |
| `MDK-ARM/` | Keil 工程文件和构建产物目录。`explorer_26/` 下多为编译输出。 |
| `cmake/`、`CMakeLists.txt` | GCC/CMake 构建入口，主要给命令行构建、索引和 clangd 使用。 |
| `26_探险.ioc` | CubeMX 配置文件。 |
| `2026探险.txt` | 路线/比赛相关的大型资料文件。 |

## 启动流程

默认启动从 `Core/Src/main.c` 进入：

1. `HAL_Init()` 和 `SystemClock_Config()` 初始化 HAL 与时钟。
2. `MX_GPIO_Init()`、`MX_TIMx_Init()`、`MX_UARTx_Init()` 等初始化 CubeMX 外设。
3. `user_init()` 初始化底盘相关外设：延时、循迹 GPIO、循迹权重、编码器、IMU、电机，并标定 IMU 基准。
4. `Start_task_create()` 创建开始任务。
5. FreeRTOS 调度启动后，`Start_task()` 创建 `main_task` 和 `motor_task`，随后删除自身。

`Core/Src/main.c` 中有两个调试宏：

- `WHEEL_REV_TEST`：单独测试电机方向。
- `PLATFORM_TURN_TEST`：单独测试平台 180 度调头。

默认应保持这些宏为 `0`，需要单项硬件验证时再临时打开。

## 任务分工

| 任务 | 文件 | 职责 |
|---|---|---|
| `Start_task` | `Task/temporary_task.c` | 创建主控任务和电机任务后自删。 |
| `main_task` | `Task/main_task.c` | 地图初始化、执行 `zhunbei()`，循环调用 `Cross()` 推进路线。 |
| `motor_task` | `Task/motor_task.c` | 5ms 周期读取传感器、处理 PID 模式、执行保护、更新目标速度、计算 PID、输出 PWM。 |
| `sin_task` | `Math/sin_generate.c` | 正弦相关辅助任务。 |

主控任务只决定“要做什么”，例如到哪个节点、进入哪个障碍、设置目标速度。电机任务负责“每 5ms 如何把目标变成电机输出”。这两个层级不要互相混写，否则后续调车会很难排查。

## 控制主线

默认比赛流程大致如下：

1. `mapInit()` 初始化节点状态和路线索引。
2. `zhunbei()` 完成启动准备，包含下坡、等待挡板、切换循线。
3. `Cross()` 作为节点间主状态机：
   - 根据当前节点设置循线模式。
   - 根据里程和传感器判断是否接近节点。
   - 到达节点后调用 `map_function()` 分发障碍处理。
   - 根据角度差决定直行通过、行进转弯或停车转弯。
   - 推进 `lastNode / nowNode / nextNode`。
4. `map.routetime == 1` 表示一轮结束，主控任务切入 `is_No` 停车。

`Cross()` 是路线行为的核心入口。需要改节点间逻辑时，优先看 `App/map/map.c`；需要改某个障碍动作时，优先看 `App/barrier/barrier.c`。

## 底盘模式

PID 模式定义在 `Task/motor_task.h`：

| 模式 | 含义 | 主要执行位置 |
|---|---|---|
| `is_No` | 停车/急停，清速度、目标、PID、PWM | `pid_mode_switch()`、`motor_stop_all()` |
| `is_Free` | 自由模式 | `motor_update_pid_mode()` |
| `is_Line` | 循线模式 | `Go_Line()` |
| `is_Turn` | 原地转弯模式 | `Turn_Angle()`、`Stage_turn_Angle()`、`Turn360Step()` |
| `is_Gyro` | 陀螺仪锁航向直行 | `runWithAngle()` |
| `is_sp` | 特殊模式预留 | 视业务代码使用 |
| `is_Remote` | 遥控模式预留 | 视业务代码使用 |

业务层优先使用 `App/chassis/chassis_api.h` 中的接口：

- `Chassis_SetMode(mode)`：统一切换底盘模式。
- `Chassis_MotorControl(mode, lspeed, rspeed, angle)`：设置模式和速度。
- `Chassis_SetTargetSpeed(speed)`：设置循线/陀螺仪目标速度。
- `Chassis_SetGyroAngle_Go(angle)`：设置陀螺仪直行航向。
- `CarBrake()`：停车。
- `Chassis_ClearMileage()` / `Chassis_GetMileage()`：里程管理。
- `Chassis_DriveDistance_Blocking()`：按距离阻塞行驶。
- `Chassis_Turn_By_StopGyro_Blocking()`：阻塞转到指定角度。
- `Chassis_Turn_180_Blocking()`：平台 180 度调头。

除非是底层控制代码，普通业务逻辑不要直接散落调用 `pid_mode_switch()`。明确需要“不继承循线/陀螺仪历史状态”的特殊路径，才使用 `pid_mode_switch_no_inherit()`。

## 模式切换与继承

`Task/motor_task.c` 中的模式切换分两层：

- `pid_mode_switch()` 只负责记录目标模式、必要限幅和 `is_No` 急停清理。
- `motor_update_pid_mode()` 每 5ms 识别真实模式变化，并调用内部 helper 做状态继承或清理。

这样做是为了减少 `is_Line` 和 `is_Gyro` 来回切换时的顿挫。普通切换会尽量继承 PID 输出、速度渐变和目标速度；只有急停或显式无继承切换才会硬清。

维护注意：

- 不要在业务流程里手动清 `line_pid_obj`、`gyroG_pid`、`TC_speed`、`TG_speed`。
- `is_Turn` 进入时不要清循线/陀螺仪速度渐变，否则转弯结束回巡线会重新起步。
- `is_No` 是真正的急停路径，应保持清理完整。

## 转弯与平台调头

转弯核心在 `Task/turn.c`：

- `Turn_Angle(target)`：普通对称原地转弯。
- `Stage_turn_Angle(target)`：平台调头，带右轮倍率和大角度固定方向。
- `Turn360Step()`：360 度转弯单步控制。
- `runWithAngle(target_angle, speed)`：陀螺仪锁航向直行。

平台 180 度调头由 `Chassis_Turn_180_Blocking()` 进入，它会设置 `StageTurn_Flag`，让电机任务在 `is_Turn` 模式下走平台调头算法。不要恢复“根据节点 function 自动分流转弯函数”的隐式逻辑，否则测试流程和正式流程容易走到不同算法。

如果平台调头仍有横移，优先调这些局部参数，而不是重构模式切换：

- `Task/turn.c` 的 `TURN_STAGE_R_RATIO`。
- `App/chassis/chassis_api.c` 中 180 度转弯速度和 PID 临时参数。
- 平台流程中的前进/后退留空间距离。

## 循线与节点识别

循线传感器在 `Sensor/scaner.c`：

- `scaner_gpio_init()` 配置 16 路循迹灯输入。
- `scaner_init()` 初始化权重。
- `getline_error()` 读取并处理主循线数据。
- `Cross_getline()` 读取节点判断用循线数据。
- `Go_Line(speed)` 使用循线误差和 PID 算出左右轮目标速度。

节点和路线在 `App/map/`：

- `map.h` 定义节点编号、节点标志、障碍类型、速度档位、节点结构体。
- `map_message.c` 存放节点表、连接表、路线表等数据。
- `deal_arrive()` 根据节点 flag 和循迹灯 mask 判断是否到达。
- `Cross()` 负责从一个节点推进到下一个节点。

维护注意：

- 巡线控制和到达检测是两个问题：巡线看 `Scaner`，节点检测看 `Cross_Scaner` 和节点 flag。
- 修改节点表时，要同时确认 `route[]`、`ConnectionNum[]`、`Address[]` 与节点编号匹配。
- `ROUTE_END` 用于标记路线结束，不要和真实节点编号混用。

## 障碍流程

障碍处理集中在 `App/barrier/barrier.c`：

| 函数 | 场景 |
|---|---|
| `zhunbei()` | 启动准备流程。 |
| `Stage()` | 通用平台处理，包含上平台、平台内移动、180 度调头、下坡、回循线。 |
| `Stage_P2()` | P2 平台专用流程。 |
| `Barrier_Bridge()` | 过桥流程。 |
| `Barrier_Hill()` | 楼梯/山地流程。 |
| `Barrier_WavedPlate(length)` | 波浪板流程。 |

`map_function(fun)` 根据 `nodesr.nowNode.function` 分发这些障碍流程。障碍流程中可以使用阻塞式底盘 API，但要注意它们必须在任务上下文调用，不能在中断里调用。

## 安全保护

底盘安全保护在 `App/chassis/chassis_api.c`：

- 游龙防护：`Chassis_EnableAntiSnake()` / `Chassis_DisableAntiSnake()`。
- 丢线保护：`Chassis_EnableLineLostProtection()` / `Chassis_DisableLineLostProtection()`。
- 翻车锁定：`Chassis_EnableRollProtection()`、`Chassis_IsTipoverLocked()`、`Chassis_ClearTipoverLock()`。
- 周期更新：`Chassis_Periodic_Update_5ms()`，由 `motor_task` 每 5ms 调用。

这些保护只在底盘周期任务里统一更新。新增保护逻辑时，应继续保持“业务层开关，电机任务周期执行”的结构。

## 电机、编码器与里程

电机输出在 `Motor/motor.c`，对外接口是 `motor_init()` 和 `motor_set_pwm()`。四个电机编号约定：

- `1 = L0`
- `2 = L1`
- `3 = R0`
- `4 = R1`

编码器在 `Motor/encoder.c`：

- TIM1：左前电机。
- TIM2：左后电机。
- TIM3：右前电机。
- TIM5：右后电机。
- TIM11：辅助定时器。

里程累计保存在 `motor_all.Distance`。需要重新计距时使用 `Chassis_ClearMileage()` 或 `encoder_clear()`，业务层优先使用 `Chassis_ClearMileage()`。

## 全局状态

几个经常排查的全局状态：

| 名称 | 位置 | 含义 |
|---|---|---|
| `motor_all` | `Driver/speed_ctrl.h` / `.c` | 底盘速度、最大转向速度、里程、速度渐变参数。 |
| `PIDMode` | `Task/motor_task.h` / `.c` | 当前底盘 PID 模式。 |
| `angle` | `Task/turn.c` | 转弯和陀螺仪直行目标角度。 |
| `nodesr` | `App/map/map.h` / `.c` | 当前路线节点状态。 |
| `map` | `App/map/map.h` / `.c` | 路线索引和轮次。 |
| `Scaner` / `Cross_Scaner` | `Sensor/scaner.h` / `.c` | 主循线数据和节点检测数据。 |
| `imu` | `Sensor/imu.h` / `.c` | IMU yaw/pitch/roll 与补偿值。 |

这些状态大多由周期任务持续更新，调试时要先确认是谁拥有写入权，避免在多个模块里同时改同一个状态。

## 编译方式

### Keil MDK

Keil 工程入口：

- `MDK-ARM/explorer_26.uvprojx`
- 目标名：`explorer_26`

如果使用命令行构建，可用工程内已有的 Keil builder：

- `python C:\Users\LOVECHEN\.agents\skills\build-keil\scripts\keil_builder.py --detect --project E:\CubeMXproject\26---main\MDK-ARM\explorer_26.uvprojx --target explorer_26`

注意：`MDK-ARM/explorer_26/` 下多为 `.o`、`.crf`、`.map`、`.hex` 等构建产物，不应当作为源码补丁提交。

### CMake / GCC

工程也保留了 CMake 入口：

- 配置：`cmake --preset Debug`
- 构建：`cmake --build --preset Debug`

`CMakeLists.txt` 会递归加入 `App/`、`Math/`、`Motor/`、`Sensor/`、`Task/`、`Driver/` 下的用户源码，并移除当前未使用且会和 CubeMX 串口中断冲突的 `Driver/uart.c`。

## 调试建议

串口调试开关在 `Driver/debug_uart.c` 顶部，例如：

- `DBG_YAW`：打印当前 yaw 和目标角。
- `DBG_LINEPID`：打印循线 PID 输出。
- `DBG_GYROPID`：打印陀螺仪直行 PID 输出。
- `DBG_TURNPID`：打印转弯 PID 输出。
- `DBG_DIST`：打印里程。
- `DBG_NODE`：打印当前节点。
- `DBG_MODE`：打印 PID 模式。

硬件问题建议按层拆开验证：

1. 电机方向：打开 `WHEEL_REV_TEST`，确认四轮方向和编号。
2. 编码器：看四路编码器速度是否随轮子方向变化。
3. IMU：看 yaw/pitch/roll 是否稳定，`mpuZreset()` 后角度是否符合预期。
4. 循迹灯：确认 16 路 GPIO bit 位和实际灯位置一致。
5. 单项动作：平台调头、陀螺仪直行、循线、坡道分别测试。
6. 完整路线：再跑 `zhunbei()` + `Cross()` 主流程。

## 开发规范

- 业务层优先使用 `Chassis_*`、`CarBrake()` 等统一接口。
- 不要在多个业务模块里直接清 PID 内部对象。
- 只在明确需要硬清历史状态时使用 `pid_mode_switch_no_inherit()`。
- 不改 CubeMX 生成代码、HAL、FreeRTOS，除非确实是外设配置需要。
- 新增路线或障碍时，优先保持模块边界：路线推进在 `map.c`，障碍动作在 `barrier.c`，底盘动作在 `chassis_api.c`，电机周期控制在 `motor_task.c`。
- 注释写“为什么这么做”，少写“这行代码做了什么”。
- 改硬件参数时记录位置、现象和验证结果，避免后续不知道参数来源。

## 常见排查入口

| 现象 | 优先查看 |
|---|---|
| 模式切换顿一下 | `Task/motor_task.c` 的模式继承 helper 和 `pid_mode_switch()`。 |
| 巡线丢线停车 | `App/chassis/chassis_api.c` 的丢线保护与 `Sensor/scaner.c` 的灯数据。 |
| 平台 180 度横移 | `Task/turn.c` 的 `Stage_turn_Angle()`，以及 `Chassis_Turn_180_Blocking()` 的临时 PID 参数。 |
| 转弯后乱跑 | `PIDMode`、`StageTurn_Flag`、`angle.AngleT`、转弯结束后的 `Chassis_SetMode()`。 |
| 节点误判 | `App/map/map.c` 的 `deal_arrive()`、节点 flag、`Cross_Scaner.detail`。 |
| 障碍结束后状态不对 | `App/barrier/barrier.c` 的收尾函数和 `nodesr.flag`。 |
| 里程异常 | `Motor/encoder.c`、`motor_all.Distance`、编码器缩放参数。 |
| 编译链接缺符号 | 先查 `.h` 声明、`.c` 实现、Keil 工程文件是否包含该源文件。 |

## 推荐阅读顺序

新接手工程时，建议按这个顺序读：

1. `Core/Src/main.c`：看启动入口和测试宏。
2. `Task/temporary_task.c`：看底盘初始化和任务创建。
3. `Task/main_task.c`：看主流程。
4. `App/map/map.h`、`App/map/map.c`、`App/map/map_message.c`：看节点、路线和 `Cross()`。
5. `App/chassis/chassis_api.h`、`App/chassis/chassis_api.c`：看底盘 API 和保护逻辑。
6. `Task/motor_task.c`：看 5ms 电机周期控制。
7. `Task/turn.c`：看转弯和陀螺仪直行。
8. `Sensor/scaner.c`、`Motor/motor.c`、`Motor/encoder.c`：看硬件输入输出。

这套顺序能先建立“路线怎么推进”的上层视角，再进入“电机怎么执行”的底层细节。
