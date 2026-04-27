# AGENTS.md

本文件用于帮助后续进入本工作区的 Codex/Agent 快速理解项目背景、代码边界和协作约定。

## 编码与读取要求

本项目源码为 UTF-8，且大量中文注释包含真实硬件意图和调试信息。阅读文件时优先使用 `rg`：

```powershell
rg --files code user
rg -n "关键词" code user
```

如果需要直接查看文件，在 PowerShell 中显式指定 UTF-8：

```powershell
Get-Content -Path code\define.h -Encoding UTF8
```

如果发现中文注释乱码，不要忽略注释继续实现；应先尝试用正确编码读取。若仍无法解决，立即中止当前任务并告知用户存在中文乱码问题。

## 项目背景

这是毕业设计《车载无人机的自主跟踪与降落方法研究》中的无人车端单片机工程。

系统目标：

- 无人车按照预设轨迹自主行驶。
- 无人机自主跟踪无人车，并降落在车载平台上。
- 无人车与无人机/上位机之间通过 WiFi + TCP 通讯，共享位姿、轨迹和动作信息。
- 后续 Ubuntu 无人机端项目会 clone 这个仓库作为只读参考，尤其参考通信协议、坐标系和单位定义。

当前工作区对应无人车端代码，目标芯片为英飞凌 TC387，多核运行环境。

## 工程性质

- 工程类型：AURIX / Eclipse CDT / Infineon AURIX Development Studio 风格工程。
- 目标 MCU：Infineon TC387。
- 底层依赖：Infineon iLLD 与逐飞科技 zf 库。
- 主要语言：C。
- 根目录中的 `.project`、`.cproject`、`Lcf_Tasking_Tricore_Tc.lsl` 等属于工程和链接配置，除非明确要求，不要随意改动。

## 目录职责

重点关注：

- `code/`：用户上层功能代码，是后续主要开发目录。
- `user/`：各核心主函数和中断服务函数，是后续主要开发目录。

一般只读了解：

- `libraries/infineon_libraries/`：Infineon 底层库。
- `libraries/zf_common/`、`libraries/zf_components/`、`libraries/zf_device/`、`libraries/zf_driver/`：逐飞科技中层库。
- `code/wit_c_sdk/`：WIT/HWT606 IMU SDK 相关代码，按第三方/设备 SDK 对待。

除非用户明确要求修底层驱动或中层库，否则不要主动修改 `libraries/`。

## 当前上层模块概览

`code/define.h`：

- 集中定义硬件引脚、PIT、UART、PWM、舵机范围、电机、编码器、遥控器、WiFi SSID/TCP 地址等。
- 修改硬件连接、频率、舵机范围、WiFi 参数时优先看这里。

`code/motion.c/.h`：

- 保存编码器脉冲、车速、电机占空比、舵机占空比等全局状态。
- `encoderSpeedCalculate()` 将编码器脉冲换算为车速，当前采样周期按 5 ms 设计。

`code/ins.c/.h`：

- 当前航位解算主状态为 `CarPoseState carPose`，由 CPU0 更新，CPU2 读取显示。
- 当前方法为“编码器里程计 + 自行车模型 + IMU 航向角互补融合”：编码器提供前向速度，舵机 PWM 估算前轮转角，陀螺 Z 轴提供短期航向角速度，AHRS yaw 只做慢修正。
- 坐标系约定：开机完成 IMU 置零时的位置为原点，开机车头方向为全局 `+X`，车体左侧为全局 `+Y`，yaw 为弧度制且逆时针为正。
- `INS_WHEEL_BASE_M`、`INS_STEER_LEFT_MAX_RAD`、`INS_STEER_RIGHT_MAX_RAD` 以及 `INS_ENCODER_SPEED_SIGN`、`INS_GYRO_Z_SIGN`、`INS_AHRS_YAW_SIGN` 都是实车标定项。
- 修改位姿估计时必须特别核对角度单位、坐标系定义、速度单位、舵机方向、IMU 安装方向和多核共享状态。
- 低价 IMU 加速度不作为位置主积分来源，当前仅保留原始数据读取；不要轻易把加速度双积分加入位置解算。

`code/hwt606.c/.h`：

- HWT606 / WIT IMU 数据接入、SDK 回调注册、更新时间标志、IMU 校准/置零命令。
- `imu_dat` 是当前 IMU 数据的主要全局结构体。
- `hwtUpdate` 是 `volatile uint8` 位标志，用于记录加速度、角速度、欧拉角、四元数、片上时间等寄存器更新。

`code/wifi.c/.h`：

- ATK-MW8266D WiFi 模块 AT 指令驱动。
- 初始化、STA 模式、连接 AP、连接 TCP 服务端、进入透传等流程在这里。
- `atk_mw8266d_info` 保存 WiFi/TCP/透传状态。

`code/wifi_packet.c/.h`：

- 自定义串口/TCP 数据帧协议。
- 帧格式：`A5 5A | Length | Command | Data | CRC16 | FF`。
- 下行 `RecvPacket_t`、上行 `SendPacket_t` 的字段顺序必须与对应 `FieldDesc_t` 描述表完全一致。
- `cmd` 定义、结构体字段顺序、坐标系、单位、CRC 范围和字节序是跨端兼容重点。
- 改通信协议时，需要同步修改结构体、字段描述表、`docs/vehicle_drone_protocol.md` 和 Ubuntu 无人机端/上位机端程序。

`code/gui.c/.h`：

- LED、按键、蜂鸣器、DUMBORC X6F 遥控器通道扫描与映射。
- 遥控器 CH3 当前用于使能遥控控制，CH1/CH2 影响舵机/电机。

## 多核分工

`user/cpu0_main.c`：

- 主要初始化核心。
- 初始化时钟、调试串口、舵机、电机、编码器、IMU、WiFi。
- 处理 IMU 数据更新、`carPoseUpdate()` 航位解算、WiFi 上行数据包发送。
- 当前仅在 IMU 片上时间 `UPDATE_TIME` 更新后推进一次航位解算，避免同一时间戳下重复积分。
- 当前上行包中 `speed`、`distance`、`yaw`、`pos_x`、`pos_y` 来自 `carPose`；`pitch`、`roll` 仍来自 IMU 欧拉角。
- 当前上行发送由 `wifi_tx_timer` 节拍触发，透传成功后约每 10 ms 发送一次。

`user/cpu1_main.c`：

- 初始化蜂鸣器、LED、按键、遥控器。
- 处理按键事件、手动 WiFi 重连、IMU 校准、遥控器控制电机/舵机。

`user/cpu2_main.c`：

- 初始化 IPS200 屏幕。
- 当前主要显示 IMU 数据、时间和 `carPose` 航位解算结果，用于调试观测。

`user/cpu3_main.c`：

- 当前基本空闲，可作为后续任务扩展核，但新增跨核共享状态前要先设计同步策略。

`user/isr.c`：

- 统一存放中断服务函数。
- CCU60_CH1：10 us 遥控器高电平计数扫描。
- CCU61_CH0：5 ms 电机/编码器周期任务。
- CCU61_CH1：2 ms 舵机刷新、按键节拍、WiFi 发送节拍。
- UART0：WiFi 模块接收，初始化阶段按 AT 文本帧处理，透传后按自定义数据包状态机处理。
- UART3：HWT606 IMU 数据输入到 WIT SDK。初始化完成后逐字节交给 `WitSerialDataIn()`。

`user/isr_config.h`：

- 中断优先级配置。TC387 中断优先级不能重复。
- 修改中断配置时必须检查所有优先级唯一性和服务核归属。

## 编码约定

- 保持现有 C 风格：小写函数名为主，宏使用大写，类型名常见 `_t`/`_T` 后缀。
- 公共头文件通常包含 `zf_common_headfile.h`。
- 新增模块一般放在 `code/xxx.c` 和 `code/xxx.h`，并在需要的 `user/*.c` 中显式包含。
- 优先复用逐飞库接口，例如 `gpio_init`、`pwm_init`、`pit_ms_init`、`uart_write_buffer`、`encoder_get_count` 等。
- 不要在中断中加入长时间阻塞、复杂浮点计算、大量打印或 AT 指令等待。
- 对 ISR 和多核共享的变量，必要时使用 `volatile`，并注意读写竞争。
- 修改舵机、电机、刹车或运动控制逻辑时，优先考虑限幅、失控保护和上电初值。
- 修改通信包时，避免直接假设字节序；当前实现使用 `memcpy` 发送 MCU 内存表示，跨平台通信端需要保持一致。
- 项目已有中文注释，可以继续使用中文注释；注释应说明硬件意图、单位、坐标系或时序约束。

## 重要单位与时序

- 电机/编码器周期：5 ms。
- 舵机/PWM 刷新及按键节拍：2 ms。
- 按键扫描实际约 4 ms 执行一次。
- 遥控器 X6F 通道扫描：10 us。
- 编码器速度 `encoder_v`：按当前系数换算为 m/s。
- 舵机 `steer_duty`：PWM duty 计数，语义方向由 `STEER_DUTY_R/M/L` 控制，其中 `STEER_DUTY_R = 1375` 为右侧、`STEER_DUTY_L = 2400` 为左侧；数值限幅应使用 `STEER_DUTY_MIN/MAX`，不要直接把 `STEER_DUTY_L/R` 当作 min/max。
- 电机 `motor_duty`：按百分比语义映射到 PWM，占空比范围应保持受控。
- IMU 欧拉角当前在主循环中由寄存器换算为度；`carPoseUpdate()` 内部会将 AHRS yaw 和 gyro Z 从度/度每秒转换为弧度/弧度每秒。
- `carPose.x/y/distance` 单位为 m，`carPose.yaw` 单位为 rad，`carPose.yaw_deg` 单位为 deg。
- 航位解算积分时间 `dt` 来自 IMU 片上时间戳，并在 `ins.c` 中由 `INS_DT_MAX` 限制异常大步长。

## 后续协作习惯

- 面向功能问题时，先读 `code/` 与 `user/`，只在需要理解底层接口时再查 `libraries/`。
- 保留中文注释和硬件调试线索，不要做无关格式化或大范围重排。

## 验证建议

本工程通常需要在 AURIX Development Studio / 对应 TC387 工具链中编译、下载和上板验证。

修改后至少检查：

- C 语法、头文件声明、全局变量是否重复定义。
- `.h` 中声明与 `.c` 中实现是否一致。
- 新增源文件是否被 Eclipse/CDT 工程纳入构建。
- 中断优先级是否唯一。
- 通信包长度、字段顺序、CRC 范围是否与对端一致。
- 电机和舵机输出是否有合理限幅。
- 上板前先让电机离地或断开动力，确认 PWM 方向和限幅无误。

当前环境里命令行 `git` 可能不在 PATH 中；`rg` 已通过 winget 安装到用户目录，并在 Codex 用户 bin 中配置了 shim。若后续环境异常，可用 PowerShell `Get-ChildItem` / `Select-String` 作为 fallback。
