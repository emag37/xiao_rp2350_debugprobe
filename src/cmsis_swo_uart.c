#include <pico/stdlib.h>
#include <string.h>

#include "Driver_USART.h"
#include "FreeRTOS.h"
#include "atomic.h"
#include "queue.h"
#include "probe_config.h"

static ARM_USART_SignalEvent_t eventHandler;

void rx_thread(void *ptr);

typedef struct  {
  uint8_t *data;
  uint32_t num;
} RxData_t;

typedef union {
  RxData_t rx;
  uint32_t baudrate;
} CmdData_t;


typedef enum CmdType {
  CMD_RX,
  CMD_SET_BAUDRATE,
  CMD_STOP,
} CmdType_t;

typedef struct {
  CmdType_t type;
  CmdData_t data;
} Cmd_t;

static TaskHandle_t rxThreadHandle;
static QueueHandle_t cmdQ;
static bool rxActive;
static uint32_t rxBytes;

static ARM_USART_CAPABILITIES GetCapabilities(void) {
  return (ARM_USART_CAPABILITIES) {
    .asynchronous = 1,
    .synchronous_master = 0,
    .synchronous_slave = 0,
    .single_wire = 0,
    .irda = 0,
    .smart_card = 0,
    .smart_card_clock = 0,
    .flow_control_rts = 0,
    .flow_control_cts = 0,
    .event_tx_complete = 0,
    .event_rx_timeout = 0,
    .rts = 0,
    .cts = 0,
    .dtr = 0,
    .dsr = 0,
    .dcd = 0,
    .ri = 0,
    .event_cts = 0,
    .event_dsr = 0,
    .event_dcd = 0,
    .event_ri = 0,
    .reserved = 0
  };
}

static ARM_DRIVER_VERSION GetVersion(void) {
  return (ARM_DRIVER_VERSION) {
    .api = ARM_USART_API_VERSION,
    .drv = 1
  };
}

static int32_t Initialize (ARM_USART_SignalEvent_t cb_event) {
    eventHandler = cb_event;
    cmdQ = xQueueCreate(1, sizeof(Cmd_t));
    gpio_set_function(SWO_UART_RX, GPIO_FUNC_UART);
    gpio_set_pulls(SWO_UART_RX, 1, 0);
    uart_init(SWO_UART_INTERFACE, SWO_UART_BAUDRATE);

    xTaskCreate(rx_thread, "SWO_RX", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &rxThreadHandle);
    return ARM_DRIVER_OK;
}

static int32_t Uninitialize (void) {
    if (rxThreadHandle != NULL) {
      vTaskDelete(rxThreadHandle);
      rxThreadHandle = NULL;
    }
    if (cmdQ != NULL) {
      vQueueDelete(cmdQ);
      cmdQ = NULL;
    }
    uart_deinit(SWO_UART_INTERFACE);
    taskENTER_CRITICAL();
    rxActive = false;
    rxBytes = 0;
    taskEXIT_CRITICAL();
    return ARM_DRIVER_OK;
}

static int32_t PowerControl(ARM_POWER_STATE state) {
    return ARM_DRIVER_OK;
}

static int32_t Receive(void *data, uint32_t num) {
  bool isRxActive;

  taskENTER_CRITICAL();
  isRxActive = rxActive;
  taskEXIT_CRITICAL();

  if (isRxActive) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  Cmd_t cmd = {
    .type = CMD_RX,
    .data.rx = {
      .data = data,
      .num = num
    }
  };
  if (xQueueSend(cmdQ, &cmd, pdMS_TO_TICKS(250)) == pdFALSE) {
    return ARM_DRIVER_ERROR_TIMEOUT;
  }
  return ARM_DRIVER_OK;
}

static uint32_t GetRxCount(void) {
  uint32_t count;
  taskENTER_CRITICAL();
  count = rxBytes;
  taskEXIT_CRITICAL();
  return count;
}

static int32_t Control(uint32_t control, uint32_t arg) {
    if (arg != 0) {
      Cmd_t cmd = {
        .type = CMD_SET_BAUDRATE,
        .data.baudrate = arg
      };
      if (xQueueSend(cmdQ, &cmd, pdMS_TO_TICKS(250)) == pdFALSE) {
        return ARM_DRIVER_ERROR_TIMEOUT;
      }
    }
    // Everything else matches the default, since we're synchronously reading we can't
    // really abort transfer
    return ARM_DRIVER_OK;
}

static ARM_USART_STATUS GetStatus(void) {
  ARM_USART_STATUS ret_usart_status;
  memset(&ret_usart_status, 0, sizeof(ret_usart_status));

  taskENTER_CRITICAL();
  ret_usart_status.rx_busy = rxActive;
  taskEXIT_CRITICAL();
  return ret_usart_status;
}
ARM_DRIVER_USART Driver_USART0 = {
  .GetCapabilities = GetCapabilities,
  .GetVersion = GetVersion,
  .Initialize = Initialize,
  .Uninitialize = Uninitialize,
  .PowerControl = PowerControl,
  .Receive = Receive,
  .Transfer = NULL, // Not used for SWO
  .GetTxCount = NULL, // Not used for SWO
  .GetRxCount = GetRxCount,
  .Control = Control,
  .GetStatus = GetStatus,
  .SetModemControl = NULL, // Not used for SWO
  .GetModemStatus = NULL, // Not used for SWO
};


static uint32_t calculate_interval(uint32_t baudrate) {
  uint32_t micros = (1000 * 1000 * 16 * 10) / MAX(baudrate, 1);
  return MAX(1, micros / ((1000 * 1000) / configTICK_RATE_HZ));
}

void rx_thread(void *ptr)
{
  uint32_t nextInterval = calculate_interval(SWO_UART_BAUDRATE);
  uint8_t* nextRxData = NULL;
  uint32_t nextRxLen = 0;

  while (1) {
    Cmd_t nextCmd;
    BaseType_t result = xQueueReceive(cmdQ, &nextCmd, nextInterval);

    if (result == pdTRUE) {
      switch (nextCmd.type) {
        case CMD_RX:
          nextRxData = nextCmd.data.rx.data;
          nextRxLen = nextCmd.data.rx.num;
          taskENTER_CRITICAL();
          rxActive = true;
          rxBytes = 0;
          taskEXIT_CRITICAL();
          break;
        case CMD_SET_BAUDRATE:
          uart_set_baudrate(SWO_UART_INTERFACE, nextCmd.data.baudrate);
          nextInterval = calculate_interval(nextCmd.data.baudrate);
          break;
        case CMD_STOP:
          taskENTER_CRITICAL();
          rxActive = false;
          taskEXIT_CRITICAL();
          
          if (nextRxData != NULL) {
            eventHandler(ARM_USART_EVENT_RECEIVE_COMPLETE);
            nextRxData = NULL;
          }
          
          break;
      }
    }
    
    if (nextRxData != NULL) {
      uint32_t thisReadBytes = 0;

      while (nextRxLen > 0 && uart_is_readable(SWO_UART_INTERFACE)) {
        uint8_t byte = uart_getc(SWO_UART_INTERFACE);
        *nextRxData++ = byte;
        nextRxLen--;
        thisReadBytes++;
      }

      taskENTER_CRITICAL();
      rxBytes += thisReadBytes;
      taskEXIT_CRITICAL();
      
      if(rxActive && nextRxLen == 0) {
        eventHandler(ARM_USART_EVENT_RECEIVE_COMPLETE);
        taskENTER_CRITICAL();
        rxActive = false;
        taskEXIT_CRITICAL();
        nextRxData = NULL;
      }
    }
  }
}