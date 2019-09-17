/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_uart.h"
#include "kws_dnn.h"
// #include "kws_ds_cnn.h"

#include "fsl_sai.h"
#include "fsl_codec_common.h"
#include "fsl_wm8524.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_codec_adapter.h"
#include "fsl_rdc.h"
#include "fsl_gpc.h"
#include "sai_low_power_audio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RDC_DISABLE_A53_ACCESS 0xFC
#define RDC_DISABLE_M4_ACCESS 0xF3

#define ServiceFlagAddr SRC->GPR9
#define ServiceBusy (0x5555U)
#define ServiceIdle (0x0U)


#define DEMO_SAI (I2S3)
#define DEMO_SAI_CLK_FREQ                                                                  \
    (CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootSai3)) / \
     (CLOCK_GetRootPostDivider(kCLOCK_RootSai3)))
#define DEMO_CODEC_WM8524 (1)
#define DEMO_CODEC_BUS_PIN (NULL)
#define DEMO_CODEC_BUS_PIN_NUM (0)
#define DEMO_CODEC_MUTE_PIN (GPIO5)
#define DEMO_CODEC_MUTE_PIN_NUM (21)
/*set Bclk source to Mclk clock*/
#define DEMO_SAI_CLOCK_SOURCE (1U)
#define OVER_SAMPLE_RATE (384U)

#define DEMO_AUDIO_SAMPLE_RATE (kSAI_SampleRate16KHz)

#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
#define DEMO_AUDIO_MASTER_CLOCK OVER_SAMPLE_RATE *DEMO_AUDIO_SAMPLE_RATE
#else
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ
#endif
/* demo audio data channel */
#define DEMO_AUDIO_DATA_CHANNEL (2U)
/* demo audio bit width */
#define DEMO_AUDIO_BIT_WIDTH kSAI_WordWidth16bits

#define RECORDING_WIN (2)
#define AVERAGING_WINDOW_LEN (3)

#define BUFFER_SIZE (128)
#define BUFFER_NUMBER (4)
#define AUDIO_BUFFER_SIZE (RECORDING_WIN * FRAME_SHIFT)
#define AUDIO_BUFFER_COUNT (2 * (AUDIO_BUFFER_SIZE / BUFFER_SIZE))

#define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMX8MM_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE 0xB8000000
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-openamp-demo-channel"
#define RPMSG_LITE_MASTER_IS_LINUX

#define APP_DEBUG_UART_BAUDRATE (115200U) /* Debug console baud rate. */
#define APP_TASK_STACK_SIZE (256)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif
#define APP_RPMSG_READY_EVENT_DATA (1)

#define TASK_PRIO (configMAX_PRIORITIES - 1)
#define CONSUMER_LINE_SIZE 1

SemaphoreHandle_t xSemaphore_producer;
SemaphoreHandle_t xSemaphore_consumer;


typedef struct the_message
{
    uint32_t DATA;
} volatile THE_MESSAGE, *THE_MESSAGE_PTR;

volatile THE_MESSAGE msg = {0};
#ifdef RPMSG_LITE_MASTER_IS_LINUX
static char helloMsg[13];
#endif /* RPMSG_LITE_MASTER_IS_LINUX */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_WM8524_Mute_GPIO(uint32_t output);
int run_kws();

KWS_DNN *kws;
// KWS_DS_CNN *kws;
static char output_class[12][8] = {"Silence", "Unknown","yes","no","up","down","left","right","on","off","stop","go"};
int detection_threshold = 95;
static volatile uint32_t audio_buffer_counter = 0U;

/*******************************************************************************
 * Variables
 ******************************************************************************/
static wm8524_config_t wm8524Config = {
    .setMute     = BOARD_WM8524_Mute_GPIO,
    .setProtocol = NULL,
    .protocol    = kWM8524_ProtocolI2S,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8524, .codecDevConfig = &wm8524Config};

//sai_handle_t txHandle           = {0};
static volatile bool isFinished = false;
extern codec_config_t boardCodecConfig;
#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) || \
    (defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
sai_master_clock_t mclkConfig = {
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    .mclkOutputEnable = true,
#if !(defined(FSL_FEATURE_SAI_HAS_NO_MCR_MICS) && (FSL_FEATURE_SAI_HAS_NO_MCR_MICS))
    .mclkSource = kSAI_MclkSourceSysclk,
#endif
#endif
};
#endif

uint8_t codecHandleBuffer[CODEC_HANDLE_SIZE] = {0U};
codec_handle_t *codecHandle                  = (codec_handle_t *)codecHandleBuffer;

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Buffer[BUFFER_SIZE * BUFFER_NUMBER], 4);

sai_handle_t txHandle = {0}, rxHandle = {0};
static uint32_t tx_index = 0U, rx_index = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/
TaskHandle_t app_task_handle = NULL;

void Peripheral_RdcSetting(void)
{
    rdc_domain_assignment_t assignment = {0};
    rdc_periph_access_config_t periphConfig;

    assignment.domainId = BOARD_DOMAIN_ID;
    RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_PERIPH, &assignment);
    RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_BURST, &assignment);
    RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_SPBA2, &assignment);

    RDC_GetDefaultPeriphAccessConfig(&periphConfig);
    /* Do not allow the A53 domain(domain0) to access the following peripherals. */
    periphConfig.policy = RDC_DISABLE_A53_ACCESS;
    periphConfig.periph = kRDC_Periph_SAI3;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
    periphConfig.periph = kRDC_Periph_UART4;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
    periphConfig.periph = kRDC_Periph_I2C4;
    RDC_SetPeriphAccessConfig(RDC, &periphConfig);
}

void BOARD_WM8524_Mute_GPIO(uint32_t output)
{
    GPIO_PinWrite(DEMO_CODEC_MUTE_PIN, DEMO_CODEC_MUTE_PIN_NUM, output);
}


static void rx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    sai_transfer_t xfer = { 0 };
    if(kStatus_SAI_RxError == status)
    {
      SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
      SAI_RxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
      SAI_TransferAbortReceive(DEMO_SAI, &rxHandle);

      xfer.data = Buffer + rx_index * BUFFER_SIZE;
      xfer.dataSize = BUFFER_SIZE;
      SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer);
    }
    else
    {
      rx_index = (rx_index + 1) % BUFFER_NUMBER;        
      xfer.data = Buffer + rx_index * BUFFER_SIZE;
      xfer.dataSize = BUFFER_SIZE;
      SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer);  
    }
}

static void tx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    sai_transfer_t xfer = { 0 };

    if(kStatus_SAI_TxError == status)
    {
      SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
      SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);
      SAI_TransferAbortSend(DEMO_SAI, &txHandle);

      xfer.data = Buffer + tx_index * BUFFER_SIZE;
      xfer.dataSize = BUFFER_SIZE;
      SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);
    }
    else
    { 
        memmove(kws->audio_buffer + audio_buffer_counter * (BUFFER_SIZE / 2), Buffer + tx_index * BUFFER_SIZE, BUFFER_SIZE);

        audio_buffer_counter ++;
        if (audio_buffer_counter == AUDIO_BUFFER_COUNT)
        {
            int max_ind = run_kws();
            if(kws->averaged_output[max_ind] > detection_threshold*128/100 && max_ind == 11)
            {
              PRINTF("%d%% %s\r\n",((int)kws->averaged_output[max_ind]*100/128),output_class[max_ind]);
              isFinished = true;
            }
            audio_buffer_counter = 0;
        }

        tx_index = (tx_index + 1) % BUFFER_NUMBER;  
        xfer.data = Buffer + tx_index * BUFFER_SIZE;
        xfer.dataSize = BUFFER_SIZE;
        SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);
    }
}



void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data)
{
}

#ifdef MCMGR_USED
/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    MCMGR_EarlyInit();
}
#endif /* MCMGR_USED */

void app_task(void *param)
{
    sai_transfer_t xfer;
    volatile unsigned long remote_addr;
    struct rpmsg_lite_endpoint *volatile my_ept;
    volatile rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *volatile my_rpmsg;
    volatile rpmsg_ns_handle ns_handle;

    /* Print the initial banner */
    PRINTF("\r\nRPMSG Ping-Pong FreeRTOS RTOS API Demo...\r\n");

#ifdef MCMGR_USED
    uint32_t startupData;
    mcmgr_status_t status;

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);

    my_rpmsg = rpmsg_lite_remote_init((void *)startupData, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    /* Signal the other core we are ready by triggering the event and passing the APP_RPMSG_READY_EVENT_DATA */
    MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, APP_RPMSG_READY_EVENT_DATA);
#else
    PRINTF("RPMSG Share Base Addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
#endif /* MCMGR_USED */
    while (!rpmsg_lite_is_link_up(my_rpmsg))
        ;
    PRINTF("Link is up!\r\n");

    my_queue  = rpmsg_queue_create(my_rpmsg);
    my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, NULL);
    platform_time_delay(1000);
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);
    PRINTF("Nameservice announce sent.\r\n");

#ifdef RPMSG_LITE_MASTER_IS_LINUX
    /* Wait Hello handshake message from Remote Core. */
    rpmsg_queue_recv(my_rpmsg, my_queue, (unsigned long *)&remote_addr, helloMsg, sizeof(helloMsg), NULL, RL_BLOCK);
#endif /* RPMSG_LITE_MASTER_IS_LINUX */

    ServiceFlagAddr = ServiceBusy;

    xfer.data = Buffer + rx_index * BUFFER_SIZE;
    xfer.dataSize = BUFFER_SIZE;
    SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer);

    xfer.data = Buffer + tx_index * BUFFER_SIZE;
    xfer.dataSize = BUFFER_SIZE;
    SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &xfer);


    PRINTF("Looping forever...\r\n");

    /* End of the example */
    while (1)
    {
      if(isFinished)
      {
        PRINTF("Waiting for ping...\r\n");
        rpmsg_queue_recv(my_rpmsg, my_queue, (unsigned long *)&remote_addr, (char *)&msg, sizeof(THE_MESSAGE), NULL,
                         RL_BLOCK);
        msg.DATA++;
        PRINTF("Sending pong...\r\n");
        rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&msg, sizeof(THE_MESSAGE), RL_BLOCK);

        isFinished = false;
      }
    }

    rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
    my_ept = NULL;
    rpmsg_queue_destroy(my_rpmsg, my_queue);
    my_queue = NULL;
    rpmsg_ns_unbind(my_rpmsg, ns_handle);
    rpmsg_lite_deinit(my_rpmsg);
    msg.DATA = 0;

}

/*!
 * @brief Main function
 */
int main(void)
{
    sai_transceiver_t config;
    int i;
    /* Initialize standard SDK demo application pins */
    /* Board specific RDC settings */
    BOARD_RdcInit();
    Peripheral_RdcSetting();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    /*
     * In order to wakeup M4 from LPM, all PLLCTRLs need to be set to "NeededRun"
     */
    for (i = 0; i != 39; i++)
    {
        CCM->PLL_CTRL[i].PLL_CTRL = kCLOCK_ClockNeededRun;
    }


    CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1); /* Set SAI source to AUDIO PLL1 786432000HZ*/
    CLOCK_SetRootDivider(kCLOCK_RootSai3, 1U, 32U);                /* Set root clock to 786432000HZ / 32 = 24.576M */
    /* gpio initialization */
    gpio_pin_config_t gpioConfig = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(DEMO_CODEC_MUTE_PIN, DEMO_CODEC_MUTE_PIN_NUM, &gpioConfig);

    PRINTF("SAI example started!\n\r");

    /* SAI init */
    SAI_Init(DEMO_SAI);
    SAI_TransferTxCreateHandle(DEMO_SAI, &txHandle, tx_callback, NULL);
    SAI_TransferRxCreateHandle(DEMO_SAI, &rxHandle, rx_callback, NULL);
    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&config, DEMO_AUDIO_BIT_WIDTH, kSAI_MonoLeft, kSAI_Channel0Mask);
    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &config);
    config.masterSlave = kSAI_Slave;
    config.syncMode = kSAI_ModeAsync;
    SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &config);

    /* set bit clock divider */
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);

    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);

    /* master clock configurations */
#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) || \
    (defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
#if defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
    mclkConfig.mclkHz          = DEMO_AUDIO_MASTER_CLOCK;
    mclkConfig.mclkSourceClkHz = DEMO_SAI_CLK_FREQ;
#endif
    SAI_SetMasterClockConfig(DEMO_SAI, &mclkConfig);
#endif

    /* Use default setting to init codec */
    CODEC_Init(codecHandle, &boardCodecConfig);

    kws = new KWS_DNN(RECORDING_WIN, AVERAGING_WINDOW_LEN);
    // kws = new KWS_DS_CNN(recording_win, averaging_window_len);
    kws->audio_buffer = new int16_t[kws->audio_buffer_size];

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    MCMGR_Init();
#endif /* MCMGR_USED */

    if (xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &app_task_handle) != pdPASS)
    {
        PRINTF("\r\nFailed to create application task\r\n");
        while (1)
            ;
    }

    vTaskStartScheduler();

    PRINTF("Failed to start FreeRTOS on core0.\n");
    while (1)
        ;
}

int run_kws()
{
  kws->extract_features();    //extract mfcc features
  kws->classify();	      //classify using dnn
  kws->average_predictions();

  int max_ind = kws->get_top_class(kws->averaged_output);

  return max_ind;
}