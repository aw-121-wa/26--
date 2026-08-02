# 26-- 项目修改记录

## barrier.c — 障碍物处理

| 修改 | 位置 | 说明 |
|------|------|------|
| P1下坡逻辑 | `Stage()` STAGE_DESCEND | 180°转身后陀螺仪缓速前进5cm，再切巡线下坡 |
| 桥上红线掩码 | `BRIDGE_RED_LEFT/RIGHT_MASK` | 从 0xF800/0x007F 扩大到 0xFC00/0x00FF（+2个传感器） |
| 桥上红线修正kp | `bridge_red_correct()` | 检测到红线时临时降kp（左×0.8，右×0.5），hold结束恢复 |
| 桥接近巡线模式 | `Barrier_Bridge()` | 从强制造中改为按节点flag走边沿巡线 |

## map_message.c — 节点配置

| 修改 | 说明 |
|------|------|
| P1→N1 step 10→30 | 与反向 N1→P1 对齐，防止过早开启到达检测 |
| N1→B2 flag NO→LEFT_LINE\|MORELED | 修复巡线模式为0、到达检测永不触发的问题 |

## map.c — 地图/状态机

| 修改 | 说明 |
|------|------|
| 转弯超时保护 | `cross_run_turn()` 加入1s硬超时 |
| 转弯震荡检测 | 接近目标(12°)后又弹回(>30°)立即刹车 |
| 节点重入保护 | 切换节点后8cm内不检测到达，防止误触 |

## chassis_api.c — 底盘控制

| 修改 | 说明 |
|------|------|
| 游龙保护 | 从 `Scaner.detail & 0xFC3F` 改为 `fabsf(Scaner.error) > 4.0f`，连续值更精确 |

## chassis_api.h

| 修改 | 说明 |
|------|------|
| `CHASSIS_STOP_STALL` | 新增堵转停车枚举值 |

## motor_task.c — 电机任务

| 修改 | 说明 |
|------|------|
| 堵转看门狗 | 新增 `motor_stall_watchdog()`（当前已注释），PWM>1500且编码器<3脉冲/5ms连续500ms触发 |

## main.c — 测试模式

| 修改 | 说明 |
|------|------|
| WHEEL_REV_TEST 斜坡启动 | 0→980占空比分50步每步20ms，防瞬间冲击 |
