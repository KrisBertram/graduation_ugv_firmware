#ifndef CODE_WIFI_PACKET_H_
#define CODE_WIFI_PACKET_H_

// 标识串口通信帧的帧头和长度
#define Serial_Frame_Head_1 (0xA5) // 串口通信帧头 1
#define Serial_Frame_Head_2 (0x5A) // 串口通信帧头 2
#define Serial_Frame_Tail   (0xFF) // 串口通信帧尾

/* 以下结构体应完全按协议顺序排列 */
typedef struct {
    float speed;
    float yaw;
    float pitch;
    float roll;
    float distance;
    uint16_t action;
} RecvPacket_t;

typedef struct {
    float speed;
    float distance;
    float yaw, pitch, roll;
    float pos_x, pos_y, pos_z;
    uint16_t action;
} SendPacket_t;

/* 支持的数据类型 */
typedef enum {
    TYPE_UINT8, TYPE_INT8,
    TYPE_UINT16, TYPE_INT16,
    TYPE_UINT32, TYPE_INT32,
    TYPE_UINT64, TYPE_INT64,
    TYPE_FLOAT32, TYPE_DOUBLE64
} FieldType_t;

/* 字段描述表结构 */
typedef struct {
    void *output;       // 指向目标存放变量
    FieldType_t type;   // 数据类型
} FieldDesc_t;

extern RecvPacket_t recv_packet;
extern SendPacket_t send_packet;

void serial_datapacket_recv(uint8_t bytedata);
void serial_datapacket_send(uint8_t cmd, const FieldDesc_t *desc_table, size_t desc_count);

#endif /* CODE_WIFI_PACKET_H_ */
