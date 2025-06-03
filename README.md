# Explanation of ADC DMA Integration in TM4C Microcontroller with FreeRTOS

This document explains the implementation of an ADC (Analog-to-Digital Converter) with DMA (Direct Memory Access) integration using FreeRTOS on a Tiva C microcontroller. The setup enables efficient data acquisition triggered by external GPIO interrupts, from ADC buffer to UART TX buffer, leveraging DMA to offload data transfers and reduce CPU overhead.

This feature is one of the computational components I built during my master's. The purpose of this module is to quickly read a thermopile that is being applied in an infrared interferometer environment.

## 1. Header File: ADC_DMA.h
Purpose:
This header file declares key macros, variables, and function prototypes for ADC and DMA-based burst mode sampling.

Key Components:
Includes: Contains both TivaWare peripheral headers (driverlib, hw_\*.h) and FreeRTOS headers for multitasking and event handling.

#### DMA Control Table:

`uint8_t pui8ControlTable[1024];`

The uDMA controller requires a 1024-byte aligned control table for managing DMA transfer descriptors. Compiler-specific pragmas ensure proper alignment.

#### GPIO for ADC Monitoring:

`` #define ADC_MONITOR_BASE GPIO_PORTD_BASE <br> 
  #define ADC_MONITOR_GPIO GPIO_PIN_3```

Configures a GPIO pin for monitoring ADC triggering/debugging.

#### Event Group:

EventGroupHandle_t BurstEventGroup;
#define BURST_FIFO_FULL (1 << 0)
Uses FreeRTOS Event Groups to signal when the ADC buffer is full (BURST_FIFO_FULL).

#### ADC Buffer:

volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];
Buffer to store ADC samples transferred by DMA.

Function Prototypes:

BurstModeConfig(): Initializes ADC, DMA, interrupts, and GPIO trigger.

BurstDMA_Check(): Verifies DMA and ADC status.

ADCTriggerDbgSet() and ADCTriggerDbgRst(): Debug functions to indicate ADC triggering.

# 2. Source File: ADC_DMA.c
Purpose:
Implements the initialization and interrupt-driven control flow for ADC sampling with DMA, handling transfer completions, errors, and external triggers via GPIO.

Key Components and Functionality:
### 2.1. Global Definitions:
ADC Sequencer:


#define ADC_SEQ 3
Uses ADC sequencer 3 for simplicity—single-step conversion.

#### DMA Channel:


#define DMA_CHANNEL UDMA_CH17_ADC0_3
The uDMA channel mapped to ADC0 sequencer 3.

#### Interrupts:


#define ADC_INT_SEQ INT_ADC0SS3
### 2.2. Interrupt Handlers
GPIOFIntHandler:
Handles external GPIO triggers.

On GPIO_PIN_4 rising edge:

Sets ADC trigger debug signal.

Triggers ADC sequencer via ADCProcessorTrigger().

Also handles a PWM-related interrupt for motion control and synchronization using FreeRTOS task notifications.

ADCIntHandler:
Clears ADC interrupt flag.

Reads 16-bit ADC result from ADC0_BASE + ADC_O_SSFIFO3.

Splits it into 2 bytes (ADC_rslt[0] and ADC_rslt[1]).

Sets up a DMA transfer from ADC_rslt to UART5 transmit data register (UART5_BASE + UART_O_DR).

Monitors sample count:

Every 200 samples: optional debug log.

At 4096 samples: sets BURST_FIFO_FULL event.

Resets ADC trigger debug signal.

uDMAErrIntHandler:
Handles DMA errors, logs the issue, and invokes BurstDMA_Check() to verify and potentially recover DMA state.

uDMAIntHandler:
Clears DMA interrupt flags.
Can be used to signal completion of buffer transfers or chained DMA operations.

### 2.3. Initialization Functions
InitGPIOTrigger:
Configures GPIO_PORTF_BASE pin 4 as input with pull-up.

Configures interrupt on rising edge.

Registers GPIOFIntHandler and enables it.

Also configures ADC_MONITOR_GPIO as output for debug purposes.

InitADC:
Enables ADC0 peripheral.

Configures GPIO_PORTD_BASE pin 2 as analog input.

Sets ADC sequencer 3 to software trigger with single step:

Sample channel 5.

Enable interrupt and end of sequence.

Enables ADC DMA for sequencer 3.

InitDMA:
Assigns DMA channel to ADC0 sequencer 3.

Configures DMA attributes:

16-bit size.

No source increment.

16-bit destination increment.

Arbitration size of 8.

Links ADC FIFO register to adcBuffer.

Enables high-priority and burst-only mode.

Enables DMA channel.

InitInterruptions:
Sets priority and enables ADC and DMA interrupts.

Registers ADC interrupt handler.

### 2.4. Core Configuration
BurstModeConfig():
Initializes FreeRTOS Event Group (BurstEventGroup).

Prepares known buffer patterns for verification.

Calls all initialization functions (InitDMA(), InitADC(), InitInterruptions(), InitGPIOTrigger()).

Returns true upon successful configuration.

BurstDMA_Check():
Diagnostic function that:

Checks if DMA and ADC sequencer are enabled.

Verifies DMA channel mode.

Checks and clears any DMA errors.

Reports status via UARTprintf().

### 2.5. Auxiliary Functions
ADCTriggerDbgSet() / ADCTriggerDbgRst(): Control ADC_MONITOR_GPIO pin to signal when an ADC trigger occurs.

## 3. Data Flow Summary:
Trigger: External rising edge on GPIO_PIN_4.

Action: ADC sequencer 3 starts conversion.

ISR: ADCIntHandler reads result, transfers via DMA to UART.

Event: On full buffer, sets BURST_FIFO_FULL event.

Monitoring: BurstDMA_Check() used for diagnostics.

## 4. FreeRTOS Integration
Event Groups: BurstEventGroup is used to signal ADC burst completion.

Task Notifications: Used in PWM interrupt for motion-related task coordination.

ISR Safety: All ISR interactions with FreeRTOS are ISR-safe API calls (xEventGroupSetBitsFromISR(), vTaskNotifyGiveFromISR()).

## 5. System Highlights
✅ Efficient ADC sampling using DMA with minimal CPU intervention.
✅ Modular ISR-driven architecture.
✅ Integration with FreeRTOS for event signaling and task coordination.
✅ Debug features via GPIO and UART logging.

## 6. Potential Extensions
Expand DMA to support multiple ADC sequences.

Add error recovery logic in uDMAErrIntHandler.

Implement buffer overflow protection.

Support multiple concurrent DMA streams for higher throughput.

## 7. Reference
Tiva C Series TM4C123GH6PM Microcontroller Data Sheet.

TI TivaWare Peripheral Driver Library.

FreeRTOS Documentation.


