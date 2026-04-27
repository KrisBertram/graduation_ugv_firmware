#include "zf_common_headfile.h"
#include "define.h"
#include "mmath.h"
#include "wifi_packet.h"

// +--------------+--------------+--------------+--------------+--------------+--------------+
// | Header       | Length       | Command      | Data         | CRC16 Check  | Tail         |
// +--------------+--------------+--------------+--------------+--------------+--------------+
// | A5 5A        | XX           | XX           | ... XX ...   | XX XX        | FF           |
// +--------------+--------------+--------------+--------------+--------------+--------------+
// | 2 bytes      | 1 byte       | 1 byte       | n bytes      | 2 bytes      | 1 byte       |
// +--------------+--------------+--------------+--------------+--------------+--------------+
RecvPacket_t recv_packet;
SendPacket_t send_packet;

/* 接收数据描述表 */
const FieldDesc_t recv_desc[] = {
    { &recv_packet.speed,    TYPE_FLOAT32 },
    { &recv_packet.yaw,      TYPE_FLOAT32 },
    { &recv_packet.pitch,    TYPE_FLOAT32 },
    { &recv_packet.roll,     TYPE_FLOAT32 },
    { &recv_packet.distance, TYPE_FLOAT32 },
    { &recv_packet.action,   TYPE_UINT16  }
};

/**
 * @brief   数据处理函数，用于处理解析成功的数据
 * @param   cmd     命令字节，表示接收到的命令
 * @param   datas   包含接收到的数据的字节数组
 * @param   len     数据数组的长度，以字节数表示
 */
static void data_process(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
    const uint8_t *p = datas;
    FieldDesc_t const *desc_table = NULL;
    size_t desc_count = 0;

    if (cmd == 0x02)
    {
        desc_table = recv_desc;
        desc_count = ARRAY_SIZE(recv_desc);
    }

    for(size_t i = 0; i < desc_count; ++i)
    {
        switch(desc_table[i].type)
        {
            case TYPE_UINT8:
            case TYPE_INT8:
                memcpy(desc_table[i].output, p, 1);
                p += 1;
                break;
            case TYPE_UINT16:
            case TYPE_INT16:
                memcpy(desc_table[i].output, p, 2);
                p += 2;
                break;
            case TYPE_UINT32:
            case TYPE_INT32:
            case TYPE_FLOAT32:
                memcpy(desc_table[i].output, p, 4);
                p += 4;
                break;
            case TYPE_UINT64:
            case TYPE_INT64:
            case TYPE_DOUBLE64:
                memcpy(desc_table[i].output, p, 8);
                p += 8;
                break;
            default:
                break;
        }
    }
}

/**
 * @brief   执行 CRC16 校验的函数
 * @param   data    包含要执行 CRC16 校验的数据的字节数组
 * @param   len     数据数组的长度，以字节数表示
 * @return  16 位 CRC16 校验值
 */
static uint16_t CRC16_check(const uint8_t *data, uint8_t len)
{
    // 初始化 CRC16 校验值为 0xFFFF
    uint16_t CRC16 = 0xFFFF;
    uint8_t state, i, j;

    // 遍历输入数据数组
    for (i = 0; i < len; ++i)
    {
        // 异或当前数据字节到 CRC16 校验值
        CRC16 ^= data[i];

        // 循环处理当前数据字节的每一位
        for (j = 0; j < 8; ++j)
        {
            // 获取当前 CRC16 最低位的值
            state = CRC16 & 0x01;

            // 右移 CRC16 校验值一位
            CRC16 >>= 1;

            // 如果最低位为 1，执行异或操作以应用 CRC16 校验算法
            if (state)
            {
                CRC16 ^= 0xA001;
            }
        }
    }

    // 返回计算得到的 CRC16 校验值
    return CRC16;
}

/**
 * @brief 接收数据包并进行解包解析
 * @param bytedata  接收到的单字节数据
 * @note 接收端采用状态机解析数据，需要考虑有几种状态，以及状态之间切换的条件是什么
 *      （状态 0）等待接收帧头第 1 字节 0xA5
 *      （状态 1）等待接收帧头第 2 字节 0x5A
 *      （状态 2）等待接收数据长度字节
 *      （状态 3）等待接收命令字节
 *      （状态 4）等待接收数据字节
 *      （状态 5）等待接收校验字节高 8 位
 *      （状态 6）等待接收校验字节低 8 位
 *      （状态 7）等待接收帧尾字节 0xFF
 */
void serial_datapacket_recv(uint8_t bytedata)
{
    // 状态变量初始化为 0
    static uint8_t step = 0;

    // cnt 缓冲区长度
    // Buf 缓冲区
    // len 帧长
    // cmd 命令字节
    // data_ptr 数据位的头指针
    static uint8_t cnt = 0, Buf[300], len, cmd, *data_ptr;
    static uint16_t crc16; // crc16 校验位

    // 状态机进行解析
    switch(step)
    {
        // 接收帧头 1 状态 0
        case 0:
        {
            if(bytedata == Serial_Frame_Head_1)
            {
                // 解析成功，进入下一状态
                ++step;
                cnt = 0;
                // 缓冲区的第 0 位存入帧头，然后将 cnt 自增
                Buf[cnt++] = bytedata;
            }
            break;
        }
        // 接收帧头 2 状态 1
        case 1:
        {
            if(bytedata == Serial_Frame_Head_2)
            {
                // 解析成功，进入下一状态
                ++step;
                // 缓冲区的第 1 位存入帧头，然后将 cnt 自增
                Buf[cnt++] = bytedata;
            }
            else if(bytedata == 0xA5)
            {
                // 如果收到的数据是帧头 1，则重新回到本状态的开头进行解析
                step = 1;
            }
            else
            {
                // 如果帧头 1 后紧跟的数据不是帧头 2，则重新解析
                step = 0;
            }
            break;
        }
        // 当两个帧头都解析成功，进入接收帧长状态
        case 2:
        {
            // 进入下一状态
            ++step;
            // 缓冲区的第 2 位存入帧长，然后将 cnt 自增
            Buf[cnt++] = bytedata;
            // 储存帧长
            len = bytedata;
            break;
        }
        // 接收命令字节状态
        case 3:
        {
            // 进入下一状态
            ++step;
            // 缓冲区的第 3 位存入命令字节，然后将 cnt 自增
            Buf[cnt++] = bytedata;
            // 储存命令字节
            cmd = bytedata;
            // 记录数据指针首地址
            data_ptr = &Buf[cnt];
            // 如果数据字节长度为 0 则跳过数据接收状态
            if(len == 0)
            {
                ++step;
            }
            break;
        }
        // 接收长度 len 个字节数据状态
        case 4:
        {
            // 缓冲区的第 4 ~ (3 + len) 位存入数据字节
            Buf[cnt++] = bytedata;
            // 利用指针地址偏移判断是否接收完 len 位数据
            if(data_ptr + len == &Buf[cnt])
            {
                ++step;
            }
            break;
        }
        // 接收 crc16 校验高 8 位字节
        case 5:
        {
            ++step;
            crc16 = bytedata;
            break;
        }
        // 接收 crc16 校验低 8 位字节
        case 6:
        {
            crc16 <<= 8;
            crc16 += bytedata;

            // 如果校验正确则进入下一状态
            if(crc16 == CRC16_check(Buf, cnt))
            {
                ++step;
            }
            else if(bytedata == 0xA5)
            {
                // 重新回到状态 1 进行解析
                step = 1;
            }
            else
            {
                // 重新解析
                step = 0;
            }
            break;
        }
        // 接收帧尾状态
        case 7:
        {
            // 如果帧尾接收正确
            if(bytedata == Serial_Frame_Tail)
            {
                // 数据解析
                data_process(cmd, data_ptr, len);
                step = 0;
            }
            else if(bytedata == 0xA5)
            {
                // 重新回到状态 1 进行解析
                step = 1;
            }
            else
            {
                // 重新解析
                step = 0;
            }
            break;
        }
        // 多余状态，正常情况下不可能出现
        default:
        {
            // 重新解析
            step = 0;
            break;
        }
    }
}

/**
 * @brief  发送一帧符合通信协议的数据包
 *
 * @param  cmd         命令字节，用于标识数据类型或指令类型
 * @param  desc_table  指向字段描述表数组，每个元素描述一个字段的数据指针及类型
 * @param  desc_count  字段描述表元素数量（即字段个数）
 *
 * @note   数据帧格式如下：
 *         [0]   帧头 0xA5
 *         [1]   帧头 0x5A
 *         [2]   数据长度 len（仅 payload 的长度，不包含 cmd/CRC/尾）
 *         [3]   命令字节 cmd
 *         [4..] 数据区 payload（由 desc_table 自动填充）
 *         [x]   CRC16 高 8 位（从帧头到 payload 末尾为校验范围）
 *         [x]   CRC16 低 8 位
 *         [x]   帧尾 0xFF
 */
void serial_datapacket_send(uint8_t cmd, const FieldDesc_t *desc_table, size_t desc_count)
{
    uint8_t buffer[256]; // 临时组包缓冲区
    uint8_t offset = 0;

    // 1. 帧头
    buffer[offset++] = Serial_Frame_Head_1;
    buffer[offset++] = Serial_Frame_Head_2;

    // 2. 数据长度占位，稍后填充
    uint8_t len_offset = offset++;

    // 3. 命令字节
    buffer[offset++] = cmd;

    // 4. 遍历字段描述表，把每个字段写入 buffer
    for(size_t i = 0; i < desc_count; ++i)
    {
        switch(desc_table[i].type)
        {
            case TYPE_UINT8:
            case TYPE_INT8:
                buffer[offset++] = *(uint8_t*)desc_table[i].output;
                break;
            case TYPE_UINT16:
            case TYPE_INT16:
                memcpy(&buffer[offset], desc_table[i].output, 2);
                offset += 2;
                break;
            case TYPE_UINT32:
            case TYPE_INT32:
            case TYPE_FLOAT32:
                memcpy(&buffer[offset], desc_table[i].output, 4);
                offset += 4;
                break;
            case TYPE_UINT64:
            case TYPE_INT64:
            case TYPE_DOUBLE64:
                memcpy(&buffer[offset], desc_table[i].output, 8);
                offset += 8;
                break;
        }
    }

    // 2. 回填长度 (payload 长度 = offset - header(2) - len(1) - cmd(1))
    buffer[len_offset] = offset - 4;

    // 5. CRC16（从 header 到 payload 最后）
    uint16_t crc = CRC16_check(buffer, offset);
    buffer[offset++] = (uint8_t)(crc >> 8);   // 高位
    buffer[offset++] = (uint8_t)(crc & 0xFF); // 低位

    // 6. 帧尾
    buffer[offset++] = Serial_Frame_Tail;

    // 7. 发送整包
    uart_write_buffer(ATK_MW8266D_UART, buffer, offset);
}
