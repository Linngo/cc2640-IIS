#ifndef SERIAL_UART_H
#define SERIAL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************
 * 串口任务初始化
*/  
void TaskUART_createTask(void);

void TaskUART_taskInit(void);

/*****************************************************
 * 串口写函数
*/
void TaskUARTdoWrite(uint8_t *buf, uint8_t len, const char* format, ...);
void TaskUARTWrite(uint8_t *buf, uint8_t len);
/*****************************************************
 * 串口接收数据回调（包括数据buf及len）
*/
typedef void (*GY_UartRxBufCallback)(uint8_t *buf, uint8_t len);

/*****************************************************
 * 注册串口接收回调任务（将串口接收的数据传给app任务去处理）
*/
void GY_UartTask_RegisterPacketReceivedCallback(GY_UartRxBufCallback callback);


#ifdef __cplusplus
{
#endif // extern "C"

#endif // end of SERIAL_UART_H definition