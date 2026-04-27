# Vehicle-Drone TCP Protocol

本文件记录无人车端 TC387 单片机与后续 Ubuntu 无人机端之间的 WiFi + TCP 透传数据帧。当前实现位于 `code/wifi_packet.c/.h`，上行发送入口位于 `user/cpu0_main.c`。

## Transport

- 物理/链路：ATK-MW8266D WiFi 模块，STA 模式连接 AP 后连接 TCP 服务端。
- 单片机串口：`ATK_MW8266D_UART`，默认 `UART_0`，`115200 bps`。
- TCP 参数：当前在 `code/define.h` 中由 `WIFI_SSID`、`WIFI_PASSWORD`、`TCP_SERVER_IP`、`TCP_SERVER_PORT` 定义。提交公开或跨设备仓库前需要先处理这些敏感/本机网络字段。
- 发送频率：无人车端进入透传并连接成功后，`wifi_tx_timer` 每 2 ms 递增，`user/cpu0_main.c` 中达到 5 后发送一次，约 10 ms 一帧，即约 100 Hz。

## Frame Format

所有 TCP payload 均使用同一帧格式：

| 字段 | 长度 | 当前值/说明 |
| --- | ---: | --- |
| Header 1 | 1 byte | `0xA5` |
| Header 2 | 1 byte | `0x5A` |
| Length | 1 byte | `Data` 区长度，不包含 `Command`、CRC 和 Tail |
| Command | 1 byte | 命令字 |
| Data | `Length` bytes | 按对应命令的字段顺序顺序拼接 |
| CRC16 High | 1 byte | CRC16 高 8 位 |
| CRC16 Low | 1 byte | CRC16 低 8 位 |
| Tail | 1 byte | `0xFF` |

CRC16 校验范围为从 `Header 1` 到 `Data` 最后一个字节，即不包含 CRC 字节和 Tail。

## CRC

当前 CRC 实现见 `code/wifi_packet.c` 中 `CRC16_check()`：

- 初值：`0xFFFF`
- 多项式：`0xA001`
- 逐字节先异或，再右移处理 8 bit
- 发送顺序：先发送 CRC 高 8 位，再发送 CRC 低 8 位
- 接收端同样按高字节、低字节组装后与本地计算值比较

## Byte Order And Binary Representation

当前代码使用 `memcpy()` 直接把 MCU 内存表示写入 `Data` 区，也用 `memcpy()` 从接收缓冲区还原字段：

- `float` 按 32-bit IEEE-754 单精度理解。
- `uint16_t` 按 16-bit 无符号整数理解。
- Ubuntu 端必须按字段描述表逐字段解析，不要按 C struct 直接整包强转。
- 当前 TC387 工程的跨端协议应按小端字段解析；如果后续更换 MCU、编译器或显式改为网络字节序，必须同步更新本文档和对端解析代码。
- 当前协议没有结构体 padding 传输，`Data` 区只包含字段本身按描述表顺序拼接后的字节。

## Command Summary

| Command | 方向 | Payload 长度 | 说明 |
| --- | --- | ---: | --- |
| `0x01` | 无人车 -> 无人机/上位机 | 34 bytes | 无人车状态上行 |
| `0x02` | 无人机/上位机 -> 无人车 | 22 bytes | 无人机/上位机状态或控制下行 |

## Command `0x01`: Vehicle To Drone

代码来源：

- 结构体：`SendPacket_t` in `code/wifi_packet.h`
- 字段描述表：`send_desc[]` in `user/cpu0_main.c`
- 发送调用：`serial_datapacket_send(0x01, send_desc, ARRAY_SIZE(send_desc))`

字段顺序必须与 `send_desc[]` 保持一致：

| 顺序 | 字段 | 类型 | 单位 | 当前来源/含义 |
| ---: | --- | --- | --- | --- |
| 1 | `speed` | `float32` | m/s | `carPose.speed`，滤波后的车体前向速度 |
| 2 | `distance` | `float32` | m | `carPose.distance`，累计行驶距离 |
| 3 | `yaw` | `float32` | deg | `carPose.yaw_deg`，融合航向角，逆时针为正 |
| 4 | `pitch` | `float32` | deg | `imu_dat.pitch`，IMU 欧拉角 |
| 5 | `roll` | `float32` | deg | `imu_dat.roll`，IMU 欧拉角 |
| 6 | `pos_x` | `float32` | m | `carPose.x`，全局 X 坐标 |
| 7 | `pos_y` | `float32` | m | `carPose.y`，全局 Y 坐标 |
| 8 | `pos_z` | `float32` | m | 当前固定为 `0` |
| 9 | `action` | `uint16` | enum/bitfield 待定义 | 当前固定为 `1` |

`Data` 长度计算：8 个 `float32` + 1 个 `uint16` = `8 * 4 + 2 = 34 bytes`。整帧长度为 `34 + 7 = 41 bytes`。

## Command `0x02`: Drone To Vehicle

代码来源：

- 结构体：`RecvPacket_t` in `code/wifi_packet.h`
- 字段描述表：`recv_desc[]` in `code/wifi_packet.c`
- 接收分发：`data_process()` 中仅当 `cmd == 0x02` 时解析到 `recv_packet`

字段顺序必须与 `recv_desc[]` 保持一致：

| 顺序 | 字段 | 类型 | 单位 | 当前含义 |
| ---: | --- | --- | --- | --- |
| 1 | `speed` | `float32` | m/s | 无人机/上位机下发速度或状态量，具体控制语义待上层定义 |
| 2 | `yaw` | `float32` | deg | 无人机/上位机下发航向角或状态量 |
| 3 | `pitch` | `float32` | deg | 无人机/上位机下发俯仰角或状态量 |
| 4 | `roll` | `float32` | deg | 无人机/上位机下发横滚角或状态量 |
| 5 | `distance` | `float32` | m | 无人机/上位机下发距离或状态量 |
| 6 | `action` | `uint16` | enum/bitfield 待定义 | 动作/控制命令，具体语义待定义 |

`Data` 长度计算：5 个 `float32` + 1 个 `uint16` = `5 * 4 + 2 = 22 bytes`。整帧长度为 `22 + 7 = 29 bytes`。

## Coordinate System And Units

当前无人车端全局位姿来自 `code/ins.c/.h`：

- 原点：小车上电并完成 IMU 置零时的位置。
- 全局 `+X`：小车开机时车头方向。
- 全局 `+Y`：小车开机时车体左侧方向。
- `carPose.yaw`：弧度制，逆时针为正，范围约束到 `[-pi, pi]`。
- 上行 `yaw`：当前发送 `carPose.yaw_deg`，单位为度。
- `pos_x`、`pos_y`、`distance`：单位为米。
- `speed`：单位为米每秒。
- `pitch`、`roll`：单位为度，直接来自 IMU 欧拉角。

## Change Rules

修改通信协议时必须同步修改：

- `code/wifi_packet.h` 中的 `RecvPacket_t` / `SendPacket_t`
- `code/wifi_packet.c` 中的 `recv_desc[]` 和 `data_process()`
- `user/cpu0_main.c` 中的 `send_packet` 填充和 `send_desc[]`
- 本文档
- Ubuntu 无人机端或上位机端的解析/组包代码

字段顺序、cmd 定义、单位、坐标系、CRC 范围和字节序属于协议兼容性重点，不能只改单片机端。
