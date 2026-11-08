#include "tjc_usart_hmi.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t length;
  uint8_t data[TJC_RING_BUFFER_SIZE];
} TJC_RingBufferTypeDef;

static UART_HandleTypeDef *tjc_uart;
static TJC_RingBufferTypeDef tjc_rx_buffer;
static uint8_t tjc_rx_byte;
static uint8_t tjc_deferred_command;
static uint8_t tjc_deferred_page;
static uint8_t tjc_has_deferred_command;
static uint8_t tjc_has_deferred_page;

volatile uint32_t g_tjc_uart_rx_count;
volatile uint32_t g_tjc_uart_error_count;
volatile uint32_t g_tjc_page_event_count;
volatile uint32_t g_tjc_command_event_count;
volatile uint8_t g_tjc_uart_last_rx;

static bool TJC_LCD_ReadEventFromRx(TJC_LCD_Event *event);

static HAL_StatusTypeDef TJC_LCD_SendTerminator(void)
{
  static const uint8_t terminator[3] = {0xFFU, 0xFFU, 0xFFU};

  if (tjc_uart == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(tjc_uart, (uint8_t *)terminator,
                           sizeof(terminator), HAL_MAX_DELAY);
}

static bool TJC_LCD_IsValidName(const char *name)
{
  return (name != NULL) && (name[0] != '\0');
}

static bool TJC_LCD_HasTerminatedPacket(uint16_t length)
{
  uint8_t value;
  uint16_t index;

  if (TJC_LCD_RxAvailable() < length)
  {
    return false;
  }
  for (index = (uint16_t)(length - 3U); index < length; ++index)
  {
    (void)TJC_LCD_RxPeek(index, &value);
    if (value != 0xFFU)
    {
      return false;
    }
  }
  return true;
}

static HAL_StatusTypeDef TJC_LCD_WaitForEvent(TJC_LCD_EventType type,
                                               uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  TJC_LCD_Event event;

  do
  {
    while (TJC_LCD_ReadEventFromRx(&event))
    {
      if (event.type == type)
      {
        return HAL_OK;
      }
      if (event.type == TJC_LCD_EVENT_USER_COMMAND)
      {
        /* A press just before transparent mode must still take effect after
         * the 600-byte transfer. Keeping the latest press matches the UI. */
        tjc_deferred_command = event.value;
        tjc_has_deferred_command = 1U;
      }
      else if (event.type == TJC_LCD_EVENT_PAGE_ID)
      {
        tjc_deferred_page = event.value;
        tjc_has_deferred_page = 1U;
      }
    }
    HAL_Delay(1U);
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return HAL_TIMEOUT;
}

void TJC_LCD_Init(UART_HandleTypeDef *huart)
{
  tjc_uart = huart;
  g_tjc_uart_rx_count = 0U;
  g_tjc_uart_error_count = 0U;
  g_tjc_page_event_count = 0U;
  g_tjc_command_event_count = 0U;
  g_tjc_uart_last_rx = 0U;
  tjc_has_deferred_command = 0U;
  tjc_has_deferred_page = 0U;
  TJC_LCD_ClearRxBuffer();
}

HAL_StatusTypeDef TJC_LCD_SendData(const uint8_t *data, uint16_t length)
{
  if ((tjc_uart == NULL) || ((data == NULL) && (length != 0U)))
  {
    return HAL_ERROR;
  }

  if (length == 0U)
  {
    return HAL_OK;
  }

  return HAL_UART_Transmit(tjc_uart, (uint8_t *)data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef TJC_LCD_SendCommand(const char *command)
{
  HAL_StatusTypeDef status;
  size_t length;

  if ((tjc_uart == NULL) || (command == NULL))
  {
    return HAL_ERROR;
  }

  length = strlen(command);
  if (length > UINT16_MAX)
  {
    return HAL_ERROR;
  }

  status = TJC_LCD_SendData((const uint8_t *)command, (uint16_t)length);
  if (status != HAL_OK)
  {
    return status;
  }

  return TJC_LCD_SendTerminator();
}

HAL_StatusTypeDef TJC_LCD_Printf(const char *format, ...)
{
  char command[TJC_COMMAND_BUFFER_SIZE];
  va_list arguments;
  int length;

  if (format == NULL)
  {
    return HAL_ERROR;
  }

  va_start(arguments, format);
  length = vsnprintf(command, sizeof(command), format, arguments);
  va_end(arguments);

  if ((length < 0) || ((size_t)length >= sizeof(command)))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_SendCommand(command);
}

HAL_StatusTypeDef TJC_LCD_Page(const char *page)
{
  if (!TJC_LCD_IsValidName(page))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("page %s", page);
}

HAL_StatusTypeDef TJC_LCD_SetText(const char *component, const char *text)
{
  char command[TJC_COMMAND_BUFFER_SIZE];
  size_t used;
  const char *source;

  if (!TJC_LCD_IsValidName(component) || (text == NULL))
  {
    return HAL_ERROR;
  }

  used = (size_t)snprintf(command, sizeof(command), "%s.txt=\"", component);
  if (used >= sizeof(command))
  {
    return HAL_ERROR;
  }

  for (source = text; *source != '\0'; ++source)
  {
    if ((*source == '\\') || (*source == '"'))
    {
      if ((used + 2U) >= sizeof(command))
      {
        return HAL_ERROR;
      }
      command[used++] = '\\';
    }
    else if ((used + 1U) >= sizeof(command))
    {
      return HAL_ERROR;
    }

    command[used++] = *source;
  }

  if ((used + 2U) > sizeof(command))
  {
    return HAL_ERROR;
  }
  command[used++] = '"';
  command[used] = '\0';

  return TJC_LCD_SendCommand(command);
}

HAL_StatusTypeDef TJC_LCD_SetValue(const char *component, int32_t value)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("%s.val=%ld", component, (long)value);
}

HAL_StatusTypeDef TJC_LCD_SetColor(const char *component, uint16_t rgb565)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("%s.pco=%u", component, (unsigned int)rgb565);
}

HAL_StatusTypeDef TJC_LCD_SetVisible(const char *component, bool visible)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("vis %s,%u", component, visible ? 1U : 0U);
}

HAL_StatusTypeDef TJC_LCD_SetTouchEnabled(const char *component, bool enabled)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("tsw %s,%u", component, enabled ? 1U : 0U);
}

HAL_StatusTypeDef TJC_LCD_SetBacklight(uint8_t percent)
{
  if (percent > 100U)
  {
    percent = 100U;
  }

  return TJC_LCD_Printf("dim=%u", (unsigned int)percent);
}

HAL_StatusTypeDef TJC_LCD_Click(const char *component, bool pressed)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("click %s,%u", component, pressed ? 1U : 0U);
}

HAL_StatusTypeDef TJC_LCD_WaveformAdd(uint8_t component_id,
                                      uint8_t channel,
                                      uint8_t value)
{
  return TJC_LCD_Printf("add %u,%u,%u", (unsigned int)component_id,
                        (unsigned int)channel, (unsigned int)value);
}

HAL_StatusTypeDef TJC_LCD_WaveformAddTransparent(
    uint8_t component_id,
    uint8_t channel,
    const uint8_t *points,
    uint16_t count,
    uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;

  if ((tjc_uart == NULL) || (points == NULL) || (count == 0U) ||
      (timeout_ms == 0U))
  {
    return HAL_ERROR;
  }

  status = TJC_LCD_Printf("cle %u,255", (unsigned int)component_id);
  if (status != HAL_OK)
  {
    return status;
  }
  status = TJC_LCD_Printf("addt %u,%u,%u", (unsigned int)component_id,
                          (unsigned int)channel, (unsigned int)count);
  if (status != HAL_OK)
  {
    return status;
  }

  /* The display must explicitly enter transparent-transfer mode before any
   * raw byte is sent. Sending the data after a timeout would make the bytes
   * execute as screen commands and can corrupt the current page. */
  status = TJC_LCD_WaitForEvent(TJC_LCD_EVENT_TRANSPARENT_READY, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_UART_Transmit(tjc_uart, (uint8_t *)points, count, timeout_ms);
  if (status != HAL_OK)
  {
    return status;
  }
  return TJC_LCD_WaitForEvent(TJC_LCD_EVENT_TRANSPARENT_FINISHED, timeout_ms);
}

HAL_StatusTypeDef TJC_LCD_GetValue(const char *component)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("get %s.val", component);
}

HAL_StatusTypeDef TJC_LCD_GetText(const char *component)
{
  if (!TJC_LCD_IsValidName(component))
  {
    return HAL_ERROR;
  }

  return TJC_LCD_Printf("get %s.txt", component);
}

HAL_StatusTypeDef TJC_LCD_StartReceiveIT(void)
{
  if (tjc_uart == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_IT(tjc_uart, &tjc_rx_byte, 1U);
}

void TJC_LCD_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((tjc_uart == NULL) || (huart != tjc_uart))
  {
    return;
  }

  g_tjc_uart_last_rx = tjc_rx_byte;
  ++g_tjc_uart_rx_count;
  TJC_LCD_RxPush(tjc_rx_byte);
  (void)HAL_UART_Receive_IT(tjc_uart, &tjc_rx_byte, 1U);
}

void TJC_LCD_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((tjc_uart == NULL) || (huart != tjc_uart))
  {
    return;
  }

  ++g_tjc_uart_error_count;

  /* ORE is a blocking receive error in the STM32 HAL: it ends the active
   * interrupt transfer before invoking this callback. Clear the error flags
   * and arm the next byte again so screen communication recovers by itself. */
  __HAL_UART_CLEAR_OREFLAG(tjc_uart);
  tjc_uart->ErrorCode = HAL_UART_ERROR_NONE;
  if (tjc_uart->RxState == HAL_UART_STATE_READY)
  {
    (void)HAL_UART_Receive_IT(tjc_uart, &tjc_rx_byte, 1U);
  }
}

void TJC_LCD_ClearRxBuffer(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  tjc_rx_buffer.head = 0U;
  tjc_rx_buffer.tail = 0U;
  tjc_rx_buffer.length = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void TJC_LCD_RxPush(uint8_t data)
{
  if (tjc_rx_buffer.length >= TJC_RING_BUFFER_SIZE)
  {
    return;
  }

  tjc_rx_buffer.data[tjc_rx_buffer.tail] = data;
  tjc_rx_buffer.tail = (uint16_t)((tjc_rx_buffer.tail + 1U) %
                                  TJC_RING_BUFFER_SIZE);
  tjc_rx_buffer.length++;
}

uint16_t TJC_LCD_RxAvailable(void)
{
  return tjc_rx_buffer.length;
}

bool TJC_LCD_RxPeek(uint16_t position, uint8_t *data)
{
  uint16_t index;

  if ((data == NULL) || (position >= tjc_rx_buffer.length))
  {
    return false;
  }

  index = (uint16_t)((tjc_rx_buffer.head + position) % TJC_RING_BUFFER_SIZE);
  *data = tjc_rx_buffer.data[index];
  return true;
}

void TJC_LCD_RxDrop(uint16_t length)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (length >= tjc_rx_buffer.length)
  {
    tjc_rx_buffer.head = 0U;
    tjc_rx_buffer.tail = 0U;
    tjc_rx_buffer.length = 0U;
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  tjc_rx_buffer.head = (uint16_t)((tjc_rx_buffer.head + length) %
                                  TJC_RING_BUFFER_SIZE);
  tjc_rx_buffer.length = (uint16_t)(tjc_rx_buffer.length - length);
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static bool TJC_LCD_ReadEventFromRx(TJC_LCD_Event *event)
{
  uint8_t header;
  uint8_t value;
  uint8_t footer;

  if (event == NULL)
  {
    return false;
  }

  while (TJC_LCD_RxAvailable() != 0U)
  {
    (void)TJC_LCD_RxPeek(0U, &header);

    /* sendme: 66 <page id> FF FF FF. This check must precede raw button
     * parsing because page 1 itself contains the byte value 01. */
    if (header == 0x66U)
    {
      if (TJC_LCD_RxAvailable() < 5U)
      {
        return false;
      }
      if (TJC_LCD_HasTerminatedPacket(5U))
      {
        (void)TJC_LCD_RxPeek(1U, &value);
        TJC_LCD_RxDrop(5U);
        event->type = TJC_LCD_EVENT_PAGE_ID;
        event->value = value;
        ++g_tjc_page_event_count;
        return true;
      }
      TJC_LCD_RxDrop(1U);
      continue;
    }

    if ((header == 0xFEU) || (header == 0xFDU))
    {
      if (TJC_LCD_RxAvailable() < 4U)
      {
        return false;
      }
      if (TJC_LCD_HasTerminatedPacket(4U))
      {
        TJC_LCD_RxDrop(4U);
        event->type = (header == 0xFEU) ?
            TJC_LCD_EVENT_TRANSPARENT_READY :
            TJC_LCD_EVENT_TRANSPARENT_FINISHED;
        event->value = 0U;
        return true;
      }
      TJC_LCD_RxDrop(1U);
      continue;
    }

    if (header == 0xA5U)
    {
      if (TJC_LCD_RxAvailable() < 3U)
      {
        return false;
      }
      (void)TJC_LCD_RxPeek(1U, &value);
      (void)TJC_LCD_RxPeek(2U, &footer);
      if (footer == 0x5AU)
      {
        TJC_LCD_RxDrop(3U);
        event->type = TJC_LCD_EVENT_USER_COMMAND;
        event->value = value;
        return true;
      }
      TJC_LCD_RxDrop(1U);
      continue;
    }

    /* G.HMI sends 01/02 for time-waveform controls and 03/04 for spectrum
     * controls. Each is a complete one-byte user command. */
    if ((header == 0x01U) || (header == 0x02U) ||
        (header == 0x03U) || (header == 0x04U))
    {
      TJC_LCD_RxDrop(1U);
      event->type = TJC_LCD_EVENT_USER_COMMAND;
      event->value = header;
      ++g_tjc_command_event_count;
      return true;
    }

    /* Unknown return codes cannot permanently block the stream. */
    TJC_LCD_RxDrop(1U);
  }
  return false;
}

bool TJC_LCD_ReadEvent(TJC_LCD_Event *event)
{
  if (event == NULL)
  {
    return false;
  }
  if (tjc_has_deferred_page != 0U)
  {
    tjc_has_deferred_page = 0U;
    event->type = TJC_LCD_EVENT_PAGE_ID;
    event->value = tjc_deferred_page;
    return true;
  }
  if (tjc_has_deferred_command != 0U)
  {
    tjc_has_deferred_command = 0U;
    event->type = TJC_LCD_EVENT_USER_COMMAND;
    event->value = tjc_deferred_command;
    return true;
  }
  return TJC_LCD_ReadEventFromRx(event);
}

bool TJC_LCD_ReadUserCommand(uint8_t *command)
{
  TJC_LCD_Event event;

  if (command == NULL)
  {
    return false;
  }
  while (TJC_LCD_ReadEvent(&event))
  {
    if (event.type == TJC_LCD_EVENT_USER_COMMAND)
    {
      *command = event.value;
      return true;
    }
  }
  return false;
}
