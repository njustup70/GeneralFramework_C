#include "Monitor.hpp"
#include "stdarg.h"
#include "stdio.h"
#include "System.hpp"

const static char error_head[7] = "[ERR]:";
const static char warning_head[7] = "[WRN]:";
const static char log_head[7] = "[LOG]:";

const static uint8_t LogPort = 0x01;    // 日志使用的端口号
const static uint8_t WatchPort = 0x02;  // 监视使用的端口号
const static uint8_t TrackPort = 0x03;  // 跟踪使用的端口号


template<typename T>
static void ConcatToBuf(byte* buf, size_t& used_bytes, void* value)
{
    T targ_var = *(T*)value;
    size_t char_use = snprintf((char*)buf + used_bytes, 63 - used_bytes, "%d", targ_var);
    used_bytes += char_use;
}

static byte track_send_buf[64];
/**
 * @brief 发送监控信息
 * @note 先用着，写得草率点，后面再改
 */
void Monitor::LogTrack()
{
    if (track_count < 1)   return; // 没有跟踪变量，直接返回

    memset(track_send_buf, 0, 64);
    size_t used_bytes = 0;

    for(int i = 0; i < track_count; i++)
    {
        switch (track_type[i])
        {
            case track_uint8:
            {
                ConcatToBuf<uint8_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_int8:
            {
                ConcatToBuf<int8_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_uint16:
            {
                ConcatToBuf<uint16_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_int16:
            {
                ConcatToBuf<int16_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_uint32:
            {
                ConcatToBuf<uint32_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_int32:
            {
                ConcatToBuf<int32_t>(track_send_buf, used_bytes, track_list[i]);
                break;
            }
            case track_float:
            {
                float var = *(float*)track_list[i];
                uint8_t char_use = snprintf((char*)track_send_buf + used_bytes, 63 - used_bytes, "%.2f", var);
                used_bytes += char_use;
                break;
            }
        }
        // 添加逗号","
        if(i != track_count - 1)
        {
            track_send_buf[used_bytes] = ',';
            track_send_buf[used_bytes + 1] = '\0';
            used_bytes += 1;
        }
        else // 结束时添加换行符"\n"
        {
            size_t len = strlen((char*)track_send_buf);
            track_send_buf[len] = '\n';
            track_send_buf[len + 1] = '\0';
        }
    }

    // 发送编码后的数据
    host_coder.SendRawMsg(track_send_buf, strlen((char*)track_send_buf));   
}



static char err_log_buf[72] = {0};
/**
 * @brief 发送错误日志
 * @note 默认不向遥控器发送错误日志，只向上位机发送
 * @warning 总发送长度不超过72字节
 */
void Monitor::LogError(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    
    // 清空缓冲区
    memset(err_log_buf, 0, sizeof(err_log_buf));
    
    // 填充错误头和时间戳
    uint8_t used_bytes = snprintf(err_log_buf, 24, "[ERR][%.2f]", System.runtime_tick);
    vsnprintf(err_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 最后一位写 换行符（如果没越界）
    if (strlen(err_log_buf) < 72)
    {
        err_log_buf[strlen(err_log_buf)] = '\n';
    }

    // 发送日志到上位机
    host_coder.SendRawMsg((uint8_t*)err_log_buf, strlen(err_log_buf));
}

static char wrn_log_buf[72] = {0};
/**
 * @brief 发送警告日志
 * @note 同上
 */
void Monitor::LogWarning(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    
    // 清空缓冲区
    memset(wrn_log_buf, 0, sizeof(wrn_log_buf));
    
    // 填充警告头和时间戳
    uint8_t used_bytes = snprintf(wrn_log_buf, 24, "[WRN][%.2f]", System.runtime_tick);
    vsnprintf(wrn_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 最后一位写 换行符（如果没越界）
    if (strlen(wrn_log_buf) < 72)
    {
        wrn_log_buf[strlen(wrn_log_buf)] = '\n';
    }
    
    // 发送日志到上位机
    host_coder.SendRawMsg((uint8_t*)wrn_log_buf, strlen(wrn_log_buf));
}

static char nrm_log_buf[72] = {0};
/**
 * @brief 发送日志
 * @note 同上
 */
void Monitor::Log(const char* format, ...)
{
    // 解析可变参数列表
    va_list args;
    va_start(args, format);
    
    // 清空缓冲区
    memset(nrm_log_buf, 0, sizeof(nrm_log_buf));
    
    // 填充日志头和时间戳
    uint8_t used_bytes = snprintf(nrm_log_buf, 24, "[LOG][%.2f]", System.runtime_tick);
    vsnprintf(nrm_log_buf + used_bytes, 72 - used_bytes, format, args);
    va_end(args);

    // 最后一位写 换行符（如果没越界）
    if (strlen(nrm_log_buf) < 72)
    {
        nrm_log_buf[strlen(nrm_log_buf)] = '\n';
    }

    // 发送日志到上位机
    host_coder.SendRawMsg((uint8_t*)nrm_log_buf, strlen(nrm_log_buf));
}


void Monitor::Watch(WatchInfo info)
{
    // 将监视信息存入监视缓冲区
    if (watch_count < 24)
    {
        watch_buf[watch_count] = info;
        watch_count++;
    }
}


void Monitor::Init(UART_HandleTypeDef *huart_host, UART_HandleTypeDef *huart_farc, bool vofa_mode)
{
    host_coder.Init(huart_host);
    farcon_coder.Init(huart_farc);
}