#ifndef SERIAL_UART_H
#define SERIAL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************
 * ���������ʼ��
*/  
void TaskUART_createTask(void);

void TaskUART_taskInit(void);

/*****************************************************
 * ����д����
*/
void TaskUARTdoWrite(uint8_t *buf, uint8_t len, const char* format, ...);
void TaskUARTWrite(uint8_t *buf, uint8_t len);
/*****************************************************
 * ���ڽ������ݻص�����������buf��len��
*/
typedef void (*GY_UartRxBufCallback)(uint8_t *buf, uint8_t len);

/*****************************************************
 * ע�ᴮ�ڽ��ջص����񣨽����ڽ��յ����ݴ���app����ȥ����
*/
void GY_UartTask_RegisterPacketReceivedCallback(GY_UartRxBufCallback callback);


#ifdef __cplusplus
{
#endif // extern "C"

#endif // end of SERIAL_UART_H definition