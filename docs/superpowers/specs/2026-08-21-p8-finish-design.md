# 珠峰 P8 正式比赛终点设计

## 目标

将第一轮正式比赛路线的终点固定为珠峰 P8。珠峰完成现有上坡、撞板、手势、后退和平台语音后，等待语音实际播放时间并永久停车；不再调头、下珠峰、返程或推进地图。

## 范围

只修改以下两个生产文件：

- `App/map/route_catalog.c`
- `App/barrier/barrier.c`

测试新增一个契约测试文件，验证路线收尾和珠峰终点动作顺序。保留 `ROUTE_P8_RETURN_UPPER`、`BACK_D2/BACK_D3/BACK_D4/BACK_D5` 以及全部第二轮路线定义，不做无关重构。

## 路线设计

`ROUTE_TO_HIGH_SCORE` 和 `ROUTE_HIGH_FROM_N13` 已经以 `... B6, N20, P8` 结束。因此：

```c
static const uint8_t round1_highscore[] = {
    ROUTE_TO_HIGH_SCORE,
    ROUTE_END
};

static const uint8_t round1_highscore_d2[] = {
    ROUTE_HIGH_FROM_N13,
    ROUTE_END
};
```

第一轮四个入口门最终都停在 P8；旧返程宏仍可供第二轮和其他代码使用。

## 珠峰终点流程

在 `Barrier_HighMountain()` 顶部流程原有成功路径中保持以下顺序：

1. 珠峰两段上坡和顶部挡板处理保持不变。
2. `barrier_platform_start_gesture()` 保持现有动作组、顺序、等待时间和调用位置不变。
3. `barrier_reverse_distance(6.0f, 12.0f, heading)` 保持不变。
4. `barrier_play_platform_voice()` 保持调用；当前 P8 映射仍由 `VOICE_INDEX_PLATFORM_P8` 提供 idx=1。
5. 等待 `HIGH_MOUNTAIN_END_VOICE_WAIT_MS`，其值为 `3000u`。
6. 调用 `high_mountain_finish_stop()`：设置 `is_No`、刹车，并在 100ms 周期内持续刹车。

语音之后不再执行 `mpuZreset()`、180°转向、转向校验、重复舵机回中、`high_mountain_descend()`、重新启用丢线保护或 `barrier_complete()`。

## 失败路径与不变量

挡板检测、上坡、后退失败仍走现有 `barrier_fail()` 路径。永久停车 helper 只用于珠峰终点成功路径。南极、通用平台、P2、桥、楼梯、门、PID、循迹参数、语音映射、N13→N18、P3→N3 和 P4→N6 逻辑不修改。

## 验证

契约测试检查：

- 两条第一轮数组只保留主干宏和 `ROUTE_END`，且主干宏最后为 P8。
- 手势、后退 6cm、P8 语音、3秒等待、永久停车按顺序出现。
- 珠峰终点成功路径不再出现 180°转向、下坡或 `barrier_complete()`。
- 旧返程宏仍存在。

完成后运行相关 Python 契约测试、全量可运行测试、`git diff --check` 和 Release CMake 构建；不执行烧录和实车测试。
