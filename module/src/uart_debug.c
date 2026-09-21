//这个文件主要用来进行串口相关的操作，重定向，接收发送信息等等，都在这里进行
//解析和打印的频率需要自行决定
#include "uart_debug.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

/* 实例池：编译期固定容量，运行时按需初始化，未初始化的实例 huart 为 NULL */
static UART_Debug_t inst_pool[MAX_UART_INSTANCES];

// 在实例池中按 huart 查找实例，找不到返回 NULL
static UART_Debug_t* find_inst_by_huart(UART_HandleTypeDef *huart) {
    for (int i = 0; i < MAX_UART_INSTANCES; i++) {
        if (inst_pool[i].huart == huart) {
            return &inst_pool[i];
        }
    }
    return NULL;
}

UART_Debug_t* UART_Debug_Init(UART_HandleTypeDef *huart) {
    if (huart == NULL) return NULL;

    // 同一个 huart 重复初始化时直接返回已有实例，避免重复注册
    UART_Debug_t *existing = find_inst_by_huart(huart);
    if (existing != NULL) return existing;

    for (int i = 0; i < MAX_UART_INSTANCES; i++) {
        if (inst_pool[i].huart == NULL) {   // 找到空闲实例
            inst_pool[i].huart = huart;
            inst_pool[i].flag  = EMPTY;
            inst_pool[i].index = 0;
            inst_pool[i].ring_head = 0;
            inst_pool[i].ring_tail = 0;
            inst_pool[i].tx_head = 0;
            inst_pool[i].tx_tail = 0;
            inst_pool[i].tx_busy = 0;
            // 自动启动中断接收，之后由回调函数负责每次接收完重新启动
            if (HAL_UART_Receive_IT(huart, &inst_pool[i].rx_byte, 1) != HAL_OK) {
                inst_pool[i].huart = NULL;
                return NULL;
            }
            return &inst_pool[i];
        }
    }
    return NULL; // 实例池已满
}

// 环形缓冲写入（中断里调用）：写一个字节，缓冲满了就丢这个字节
// 环形缓冲判满：写指针再推一格就追上读指针，说明写满了，干脆牺牲一个格子的容量换取判断简单
static void ring_push(UART_Debug_t *inst, uint8_t byte) {
    uint16_t next = (inst->ring_head + 1) % RX_RING_SIZE;
    if (next != inst->ring_tail) {  // 没满才写
        inst->ring[inst->ring_head] = byte;
        inst->ring_head = next;
    } else {
        inst->rx_dropped++;
        inst->rx_overflow = 1;
    }
}

// 环形缓冲读出（主循环里调用）：取一个字节，空了返回 -1
static int8_t ring_pop(UART_Debug_t *inst, uint8_t *byte) {
    if (inst->ring_head == inst->ring_tail) return -1; // 空
    *byte = inst->ring[inst->ring_tail];
    inst->ring_tail = (inst->ring_tail + 1) % RX_RING_SIZE;
    return 0;
}

// 尝试启动一波发送：把环形缓冲里待发的字节搬一批到连续缓冲，交给中断发送
// 两个地方会调用它：主循环塞完新消息后、上一波发完的发送完成回调里
// tx_busy 挡住"正在发的时候又来启动"，防止重入
/* Short critical sections serialize main-loop producer and completion ISR. */
static void tx_start(UART_Debug_t *inst) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!inst->tx_busy) {
        uint16_t pending = (inst->tx_head - inst->tx_tail + TX_RING_SIZE) % TX_RING_SIZE;
        uint16_t n = pending < TX_CHUNK_SIZE ? pending : TX_CHUNK_SIZE;
        if (n) {
            for (uint16_t i = 0; i < n; ++i)
                inst->tx_chunk[i] = inst->tx_ring[(inst->tx_tail + i) % TX_RING_SIZE];
            inst->tx_busy = 1;
            if (HAL_UART_Transmit_IT(inst->huart, inst->tx_chunk, n) == HAL_OK)
                inst->tx_tail = (inst->tx_tail + n) % TX_RING_SIZE;
            else {
                inst->tx_busy = 0;
                inst->tx_errors++;
            }
        }
    }
    __set_PRIMASK(mask);
}
static void tx_enqueue(UART_Debug_t *inst, uint8_t *data, uint16_t len) {
    if (!len) return;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint16_t pending = (inst->tx_head - inst->tx_tail + TX_RING_SIZE) % TX_RING_SIZE;
    if (len > TX_RING_SIZE - 1 - pending) inst->tx_dropped++;
    else {
        for (uint16_t i = 0; i < len; ++i) {
            inst->tx_ring[inst->tx_head] = data[i];
            inst->tx_head = (inst->tx_head + 1) % TX_RING_SIZE;
        }
    }
    __set_PRIMASK(mask);
    tx_start(inst);
}

// 核心发送函数：inst 非空时发给指定实例，inst 为 NULL 时广播到所有实例
// 只负责"格式化 + 塞进发送环形缓冲"，立刻返回，不阻塞；真正发送由中断逐批完成
static void uart_send(UART_Debug_t *inst, const char *fmt, va_list args) {
    char buf[160]; // 格式化的暂存区，用完即弃，发送中断不碰它
    int len;

    // 使用 vsnprintf 格式化字符串，并且防止溢出
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        if (inst) inst->tx_dropped++;
        else for (int i = 0; i < MAX_UART_INSTANCES; ++i)
            if (inst_pool[i].huart) inst_pool[i].tx_dropped++;
        return;
    }

    if (inst != NULL) {
        tx_enqueue(inst, (uint8_t *)buf, len);
    } else {
        for (int i = 0; i < MAX_UART_INSTANCES; i++) {
            if (inst_pool[i].huart != NULL) {
                tx_enqueue(&inst_pool[i], (uint8_t *)buf, len);
            }
        }
    }
}

// 输出给所有串口实例
void UART_Printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    uart_send(NULL, fmt, args);
    va_end(args);
}

// 发给指定串口实例（命令回执、报错用）
static void print_to_inst(UART_Debug_t *inst, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    uart_send(inst, fmt, args);
    va_end(args);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // 找到这个 huart 对应的实例；不是本模块管理的串口就直接返回
    UART_Debug_t *inst = find_inst_by_huart(huart);
    if (inst == NULL) return;

    // 中断里只做一件事：把字节丢进环形缓冲。拼命令、解析都交给主循环，中断要短
    ring_push(inst, inst->rx_byte);

    // 重要：HAL库的中断是单次触发的，处理完必须手动开启下一次接收
    HAL_UART_Receive_IT(huart, &inst->rx_byte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // 找到这个 huart 对应的实例；不是本模块管理的串口就直接返回
    UART_Debug_t *inst = find_inst_by_huart(huart);
    if (inst == NULL) return;

    inst->tx_busy = 0; // 这一波发完了
    tx_start(inst);    // 环里还有数据就接着发下一波
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    UART_Debug_t *inst = find_inst_by_huart(huart);
    if (!inst) return;
    inst->uart_errors++;
    inst->rx_overflow = 1;
    /* F1 HAL ends RX on overrun before calling us. Nonblocking errors may
       leave RX busy; Poll retries if the receiver becomes READY later. */
    if (huart->RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_IT(huart, &inst->rx_byte, 1);
}

//以上是串口工具基本的收发功能，以下的部分是基于这两个基础功能组合出的进阶功能


static UART_CmdEntry_t cmd_table[MAX_DEBUG_CMDS];
static int cmd_count = 0;

// 注册函数：外部模块在初始化时调用（命令表全局共享，所有实例通用）
int8_t UART_Debug_Register(UART_CmdEntry_t *cmd_entry) {
    if (!cmd_entry || !cmd_entry->cmd || !cmd_entry->cmd[0] ||
        cmd_entry->type < TYPE_NONE || cmd_entry->type > TYPE_FLOAT ||
        (cmd_entry->type != TYPE_NONE && !cmd_entry->var_ptr)) return -1;
    for (int i = 0; i < cmd_count; ++i)
        if (!strcmp(cmd_table[i].cmd, cmd_entry->cmd)) return -1;
    if (cmd_count >= MAX_DEBUG_CMDS) {
        UART_Printf("Error: Command table full\r\n");
        return -1;
    }

    cmd_table[cmd_count].cmd = cmd_entry->cmd;
    cmd_table[cmd_count].type = cmd_entry->type;
    cmd_table[cmd_count].var_ptr = cmd_entry->var_ptr;
    cmd_table[cmd_count].flag_ptr = cmd_entry->flag_ptr;
    cmd_table[cmd_count].ret_type = cmd_entry->ret_type;
    cmd_table[cmd_count].func = cmd_entry->func;

    cmd_count++;
    return 0;
}

// 解析指定实例收到的命令，所有回执/报错都打到该实例对应的串口
int8_t UART_Debug_Process(UART_Debug_t *inst) {
    if (inst == NULL || inst->flag == EMPTY) return -1;

    // 拷贝一份命令到局部缓冲区，然后清空标志（避免后续修改）
    char cmd[RX_BUFFER_SIZE];
    strcpy(cmd, (char *)inst->packet);
    inst->flag = EMPTY;

    char *colon = strchr(cmd, ':'); // 查找冒号，分割命令和数值
    char *cmdStr = cmd;             // 默认命令字符串是整个输入
    char *numStr = NULL;            // 数值字符串，默认为空

    if (colon != NULL){
        // 有冒号：将冒号替换为 '\0'，使 cmdStr 指向命令
        *colon = '\0';
        numStr = colon + 1;         // 如果有数值，就让 numStr 指向数值的起始部分
    }

    for (int i = 0; i < cmd_count; i++) {
        if (strcmp(cmdStr, cmd_table[i].cmd) == 0) {// 匹配成功，根据类型自动赋值
            switch (cmd_table[i].type) {
                case TYPE_NONE:
                    if (numStr != NULL) {
                        print_to_inst(inst, "Error:Command %s get a extra value\r\n", cmd_table[i].cmd);
                        return -1;
                    }
                    break;

                case TYPE_INT:
                    if (numStr != NULL) {
                        char *end;
                        errno = 0;
                        long value = strtol(numStr, &end, 10);
                        if (errno || end == numStr || *end || value < INT_MIN || value > INT_MAX) {
                            print_to_inst(inst, "Error: Invalid integer\r\n");
                            return -1;
                        }
                        *(int*)cmd_table[i].var_ptr = (int)value;
                    }
                    else {
                        print_to_inst(inst, "Error: Value required for command %s\r\n", cmd_table[i].cmd);
                        return -1;
                    }
                    break;
                case TYPE_FLOAT:
                    if (numStr != NULL) {
                        char *end;
                        errno = 0;
                        float value = strtof(numStr, &end);
                        if (errno || end == numStr || *end || !isfinite(value)) {
                            print_to_inst(inst, "Error: Invalid float\r\n");
                            return -1;
                        }
                        *(float*)cmd_table[i].var_ptr = value;
                    }
                    else {
                        print_to_inst(inst, "Error: Value required for command %s\r\n", cmd_table[i].cmd);
                        return -1;
                    }
                    break;
                case TYPE_STR:
                    // 字符串类型的命令暂不支持数值部分，后续再补充
                    break;

                default:
                    print_to_inst(inst, "Error: Unknown type for command %s\r\n", cmd_table[i].cmd);
                    return -1;
            }
            // 顺带触发回调函数：必须按声明的返回值类型转回对应类型的函数指针再调用
            // （返回值类型不同，调用约定可能不同，用 void(*)(void) 直接调用别的类型属于未定义行为）
            if (cmd_table[i].func != NULL) {
                switch (cmd_table[i].ret_type) {
                    case TYPE_INT:
                        (void)((int (*)(void))cmd_table[i].func)();
                        break;
                    case TYPE_FLOAT:
                        (void)((float (*)(void))cmd_table[i].func)();
                        break;
                    case TYPE_NONE:
                    default:
                        ((void (*)(void))cmd_table[i].func)();
                        break;
                }
            }
            // 自动置位标志位
            if (cmd_table[i].flag_ptr != NULL) {
                *(cmd_table[i].flag_ptr) = FLAG_ON;
            }

            print_to_inst(inst, "%s has set\r\n\r\n", cmd_table[i].cmd);
            return 0;
        }
    }

    print_to_inst(inst, "Error: Unknown Command %s\r\n", cmdStr);
    return -1;
}

// 从环形缓冲里按 @...\r\n 规则拼装命令，返回第一个拼好完整命令的实例（初始化顺序即优先级），没有则返回 NULL
UART_Debug_t* UART_Debug_Poll(void) {
    for (int i = 0; i < MAX_UART_INSTANCES; ++i) {
        UART_Debug_t *inst = &inst_pool[i];
        if (!inst->huart) continue;
        tx_start(inst); /* Retry queued data after HAL_BUSY. */
        if (inst->huart->RxState == HAL_UART_STATE_READY)
            HAL_UART_Receive_IT(inst->huart, &inst->rx_byte, 1);
        if (inst->rx_overflow) {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            inst->ring_tail = inst->ring_head;
            inst->rx_overflow = 0;
            inst->collecting = inst->saw_cr = inst->index = 0;
            __set_PRIMASK(mask);
        }
        if (inst->flag == FULL) return inst;
        uint8_t byte;
        while (ring_pop(inst, &byte) == 0) {
            if (byte == '@') {
                inst->collecting = 1;
                inst->saw_cr = inst->index = 0;
            } else if (inst->collecting) {
                if (inst->saw_cr) {
                    inst->collecting = inst->saw_cr = 0;
                    if (byte == '\n' && inst->index) {
                        inst->packet[inst->index] = 0;
                        inst->index = 0;
                        inst->flag = FULL;
                        return inst;
                    }
                    inst->index = 0;
                } else if (byte == '\r') inst->saw_cr = 1;
                else if (byte < 32 || byte > 126 || inst->index >= RX_BUFFER_SIZE - 1) {
                    inst->collecting = inst->index = 0;
                } else inst->packet[inst->index++] = byte;
            }
        }
    }
    return NULL;
}
