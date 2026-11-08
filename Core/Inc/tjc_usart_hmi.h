#ifndef TJC_USART_HMI_H
#define TJC_USART_HMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define TJC_RING_BUFFER_SIZE 500U
#define TJC_COMMAND_BUFFER_SIZE 256U

typedef enum
{
  TJC_LCD_EVENT_USER_COMMAND = 0,
  TJC_LCD_EVENT_PAGE_ID,
  TJC_LCD_EVENT_TRANSPARENT_READY,
  TJC_LCD_EVENT_TRANSPARENT_FINISHED
} TJC_LCD_EventType;

typedef struct
{
  TJC_LCD_EventType type;
  uint8_t value;
} TJC_LCD_Event;

/* Live USART diagnostics, intended for the debugger watch window. */
extern volatile uint32_t g_tjc_uart_rx_count;
extern volatile uint32_t g_tjc_uart_error_count;
extern volatile uint32_t g_tjc_page_event_count;
extern volatile uint32_t g_tjc_command_event_count;
extern volatile uint8_t g_tjc_uart_last_rx;

/* Bind the driver to the UART connected to the TJC display. */
void TJC_LCD_Init(UART_HandleTypeDef *huart);

/* Low-level command interfaces. Every command is terminated with FF FF FF. */
HAL_StatusTypeDef TJC_LCD_SendCommand(const char *command);
HAL_StatusTypeDef TJC_LCD_Printf(const char *format, ...);
HAL_StatusTypeDef TJC_LCD_SendData(const uint8_t *data, uint16_t length);

/* Common TJC/Nextion instruction helpers. */
HAL_StatusTypeDef TJC_LCD_Page(const char *page);
HAL_StatusTypeDef TJC_LCD_SetText(const char *component, const char *text);
HAL_StatusTypeDef TJC_LCD_SetValue(const char *component, int32_t value);
HAL_StatusTypeDef TJC_LCD_SetColor(const char *component, uint16_t rgb565);
HAL_StatusTypeDef TJC_LCD_SetVisible(const char *component, bool visible);
HAL_StatusTypeDef TJC_LCD_SetTouchEnabled(const char *component, bool enabled);
HAL_StatusTypeDef TJC_LCD_SetBacklight(uint8_t percent);
HAL_StatusTypeDef TJC_LCD_Click(const char *component, bool pressed);
HAL_StatusTypeDef TJC_LCD_WaveformAdd(uint8_t component_id,
                                      uint8_t channel,
                                      uint8_t value);
HAL_StatusTypeDef TJC_LCD_WaveformAddTransparent(
    uint8_t component_id,
    uint8_t channel,
    const uint8_t *points,
    uint16_t count,
    uint32_t timeout_ms);
HAL_StatusTypeDef TJC_LCD_GetValue(const char *component);
HAL_StatusTypeDef TJC_LCD_GetText(const char *component);

/* Optional interrupt-driven receive support. */
HAL_StatusTypeDef TJC_LCD_StartReceiveIT(void);
void TJC_LCD_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void TJC_LCD_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* Receive ring-buffer interfaces. */
void TJC_LCD_ClearRxBuffer(void);
void TJC_LCD_RxPush(uint8_t data);
uint16_t TJC_LCD_RxAvailable(void);
bool TJC_LCD_RxPeek(uint16_t position, uint8_t *data);
void TJC_LCD_RxDrop(uint16_t length);

/* Read a raw button byte, legacy A5 packet, page reply, or addt handshake. */
bool TJC_LCD_ReadEvent(TJC_LCD_Event *event);

/* Compatibility helper for the original A5 <command> 5A screen project. */
bool TJC_LCD_ReadUserCommand(uint8_t *command);

#ifdef __cplusplus
}
#endif

#endif /* TJC_USART_HMI_H */
