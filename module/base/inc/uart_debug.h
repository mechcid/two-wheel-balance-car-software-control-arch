#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define RX_BUFFER_SIZE 64    //定义单条命令缓冲区大小（一条命令最长 63 字符，留一位给'\0'）
#define RX_RING_SIZE 128     //定义环形接收缓冲区大小，比命令缓冲大一些，用于吸收主循环来不及处理的字节
#define TX_CHUNK_SIZE 64     //定义每批交给中断发送的最大字节数，不小于单条消息长度即可
#define TX_RING_SIZE 512     //定义发送环形缓冲区大小，放不下一条消息时整条丢弃
#define MAX_DEBUG_CMDS 20    //定义最大注册数量
#define MAX_UART_INSTANCES 4 //定义最大串口实例数量，初始化超过这个数量时 UART_Debug_Init 返回 NULL

#define FULL 1               //定义命令缓冲区接收到完整命令
#define EMPTY 0              //定义命令缓冲区没有接收到完整命令

#define FLAG_OFF 1           //定义标志位开关的两个宏
#define FLAG_ON 0


typedef enum {
    TYPE_NONE,    // 表示这个命令只置位就行，不需要有数值
    TYPE_INT,
    TYPE_FLOAT,
    TYPE_STR      // 如果需要传递字符串
} VarType_t;

typedef struct {
    const char* cmd;          // 命令字符串，如 "curve"
    VarType_t type;           // 变量类型
    void* var_ptr;            // 变量地址
    VarType_t ret_type;       // 函数返回值类型
    void (*func)(void);       // 命令触发时顺带调用的回调函数，不需要就填 NULL
    int8_t*  flag_ptr;        // 标志位地址 (用于通知外部模块数据已更新)
} UART_CmdEntry_t;

// 串口调试实例：每个实例有自己独立的环形缓冲、命令缓冲和标志位，互不干扰
typedef struct {
    UART_HandleTypeDef *huart;          // HAL 串口句柄
    uint8_t  rx_byte;                   // 本实例中断接收单字节容器
    uint8_t  ring[RX_RING_SIZE];        // 本实例环形接收缓冲（中断里写，主循环里读）
    volatile uint16_t ring_head;        // 写指针：中断处理函数里推进
    volatile uint16_t ring_tail;        // 读指针：主循环里推进
    uint8_t  packet[RX_BUFFER_SIZE];    // 本实例命令缓冲：按 @...\r\n 规则从环形缓冲里拼出一条完整命令
    volatile uint8_t flag;              // 本实例标志位：FULL=有一条完整命令待处理
    uint8_t collecting, saw_cr;
    volatile uint8_t rx_overflow;
    volatile uint32_t rx_dropped, tx_dropped, uart_errors, tx_errors;
    uint8_t  index;                     // 本实例命令缓冲写入指针
    uint8_t  tx_ring[TX_RING_SIZE];     // 本实例发送环形缓冲（主循环写，中断发）
    volatile uint16_t tx_head;          // 发送写指针：主循环推进
    volatile uint16_t tx_tail;          // 发送读指针：tx_start 推进（主循环/发送完成回调都会调）
    uint8_t  tx_chunk[TX_CHUNK_SIZE];   // HAL 中断发送需要连续内存，这里暂存一批待发的字节
    volatile uint8_t tx_busy;           // 发送状态：1=正在中断发送中
} UART_Debug_t;

// 初始化一个串口调试实例，返回实例句柄；实例池已满或参数非法时返回 NULL
// 注意：函数内部会自动调用 HAL_UART_Receive_IT 启动中断接收，外部不要再手动启动
UART_Debug_t* UART_Debug_Init(UART_HandleTypeDef *huart);

// 提供给外部的注册函数（命令表全局共享，所有实例通用）
int8_t UART_Debug_Register(UART_CmdEntry_t *cmd_entry);

// 处理指定实例收到的完整命令：解析并执行，回执打到该实例对应的串口
// 注意：调用后实例内部标志位会自动清空，无需外部手动清除
// 数值支持可选 +/- 和十进制数字；TYPE_FLOAT 还允许一个小数点（如 -.5、5.）。
// 不接受空白、指数、十六进制、NaN/Inf；非法输入不赋值、不触发回调或标志。
int8_t UART_Debug_Process(UART_Debug_t *inst);

// 从环形缓冲里按 @...\r\n 规则拼装命令，返回第一个拼好完整命令的实例（初始化顺序即优先级），没有则返回 NULL
// 建议在主循环里高频调用，频率自行决定
UART_Debug_t* UART_Debug_Poll(void);

//注意，上位机传来的命令格式必须以@开头，以\r\n结尾，中间是命令内容，例如：@LED_ON\r\n

// 仅在主循环调用（不是中断安全的格式化接口）；过长消息整条丢弃。
// 输出给所有串口实例
void UART_Printf(const char *fmt, ...);



#ifdef __cplusplus
}
#endif

#endif
