/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include <stm32f7xx_hal.h>
#include "config.h"
#include "errorHandler.h"
extern "C" {
#include "osal.h"
}
#include "sensor_service.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"

#include "ioBuffer/IoBuffer.h"
#include "usb/Usb.h"
#include "usb/Debug.h"

#include <cstdio>

#define BDADDR_SIZE 6

extern volatile uint8_t set_connectable;
extern volatile int connected;
extern AxesRaw_t axes_data;
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */

void User_Process (AxesRaw_t *p_axes);
static void systemClockConfig ();

/*****************************************************************************/

static void CPU_CACHE_Enable (void);
static void MPU_Config (void);

/*****************************************************************************/

int main (void)
{
        const char *name = "BlueNRG";
        uint8_t SERVER_BDADDR[] = { 0x12, 0x34, 0x00, 0xE1, 0x80, 0x03 };
        uint8_t bdaddr[BDADDR_SIZE];
        uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

        uint8_t hwVersion;
        uint16_t fwVersion;

        int ret;
        /* Configure the MPU attributes as Write Through */
        MPU_Config ();

        /* Enable the CPU Cache */
        CPU_CACHE_Enable ();

        HAL_Init ();

#if NEW_SERVICES
        /* Configure LED2 */
        BSP_LED_Init (LED2);
#endif

        /* Configure the system clock */
        systemClockConfig ();

        IoBuffer usbBuffer (1024);
//        Usb usb (&usbBuffer);
        Debug debug (&usbBuffer);
//        usb.init ();
        debug.log (1, MICRO_STRING, "µC Initialized");
        HAL_Delay (100);

        /* Initialize the BlueNRG SPI driver */
        BNRG_SPI_Init ();

        /* Initialize the BlueNRG HCI */
        HCI_Init ();

        /* Reset BlueNRG hardware */
        BlueNRG_RST ();

        /* get the BlueNRG HW and FW versions */
        getBlueNRGVersion (&hwVersion, &fwVersion);

        /*
         * Reset BlueNRG again otherwise we won't
         * be able to change its MAC address.
         * aci_hal_write_config_data() must be the first
         * command after reset otherwise it will fail.
         */
        BlueNRG_RST ();

        debug.log (2, MICRO_UINT_8, &hwVersion);
        debug.log (3, MICRO_UINT_16, &fwVersion);

        if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
                bnrg_expansion_board = IDB05A1;
                /*
                 * Change the MAC address to avoid issues with Android cache:
                 * if different boards have the same MAC address, Android
                 * applications unless you restart Bluetooth on tablet/phone
                 */
                SERVER_BDADDR[5] = 0x02;
        }

        /* The Nucleo board must be configured as SERVER */
        Osal_MemCpy (bdaddr, SERVER_BDADDR, sizeof (SERVER_BDADDR));

        ret = aci_hal_write_config_data (CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);

        if (ret) {
                printf ("Setting BD_ADDR failed.\n");
        }

        ret = aci_gatt_init ();

        if (ret) {
                printf ("GATT_Init failed.\n");
        }

        if (bnrg_expansion_board == IDB05A1) {
                ret = aci_gap_init_IDB05A1 (GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
        }
        else {
                ret = aci_gap_init_IDB04A1 (GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
        }

        if (ret != BLE_STATUS_SUCCESS) {
                printf ("GAP_Init failed.\n");
        }

        ret = aci_gatt_update_char_value (service_handle, dev_name_char_handle, 0, strlen (name), (uint8_t *)name);

        if (ret) {
                printf ("aci_gatt_update_char_value failed.\n");
                while (1)
                        ;
        }

        ret = aci_gap_set_auth_requirement (MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING);
        if (ret == BLE_STATUS_SUCCESS) {
                printf ("BLE Stack Initialized.\n");
        }

        printf ("SERVER: BLE Stack Initialized\n");

        ret = Add_Acc_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                printf ("Acc service added successfully.\n");
        else
                printf ("Error while adding Acc service.\n");

        ret = Add_Environmental_Sensor_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                printf ("Environmental Sensor service added successfully.\n");
        else
                printf ("Error while adding Environmental Sensor service.\n");

#if NEW_SERVICES
        /* Instantiate Timer Service with two characteristics:
         * - seconds characteristic (Readable only)
         * - minutes characteristics (Readable and Notifiable )
         */
        ret = Add_Time_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                printf ("Time service added successfully.\n");
        else
                printf ("Error while adding Time service.\n");

        /* Instantiate LED Button Service with one characteristic:
         * - LED characteristic (Readable and Writable)
         */
        ret = Add_LED_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                printf ("LED service added successfully.\n");
        else
                printf ("Error while adding LED service.\n");
#endif

        /* Set output power level */
        ret = aci_hal_set_tx_power_level (1, 4);

        debug.log (1, MICRO_STRING, "BlueNRG ready");

        while (1) {
                HCI_Process ();
                User_Process (&axes_data);
#if NEW_SERVICES
                Update_Time_Characteristics ();
#endif
        }
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  AxesRaw_t* p_axes
 * @retval None
 */
void User_Process (AxesRaw_t *p_axes)
{
        if (set_connectable) {
                setConnectable ();
                set_connectable = FALSE;
        }

        static int i = 0;

        if (++i > 1000000) {
                i = 0;

                if (connected) {
                        /* Update acceleration data */
                        p_axes->AXIS_X += 100;
                        p_axes->AXIS_Y += 100;
                        p_axes->AXIS_Z += 100;
                        // printf("ACC: X=%6d Y=%6d Z=%6d\r\n", p_axes->AXIS_X, p_axes->AXIS_Y, p_axes->AXIS_Z);
                        Acc_Update (p_axes);
                }
        }
}

/*****************************************************************************/

static void systemClockConfig ()
{
        RCC_ClkInitTypeDef RCC_ClkInitStruct;
        RCC_OscInitTypeDef RCC_OscInitStruct;
        HAL_StatusTypeDef ret = HAL_OK;

        /* Enable HSE Oscillator and activate PLL with HSE as source */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 25;
        RCC_OscInitStruct.PLL.PLLN = 432;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 9;

        ret = HAL_RCC_OscConfig (&RCC_OscInitStruct);
        if (ret != HAL_OK) {
                while (1) {
                        ;
                }
        }

        /* Activate the OverDrive to reach the 216 MHz Frequency */
        ret = HAL_PWREx_EnableOverDrive ();
        if (ret != HAL_OK) {
                while (1) {
                        ;
                }
        }

        /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
        RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

        ret = HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_7);
        if (ret != HAL_OK) {
                while (1) {
                        ;
                }
        }
}
/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config (void)
{
        MPU_Region_InitTypeDef MPU_InitStruct;

        /* Disable the MPU */
        HAL_MPU_Disable ();

        /* Configure the MPU attributes as WT for SRAM */
        MPU_InitStruct.Enable = MPU_REGION_ENABLE;
        MPU_InitStruct.BaseAddress = 0x20010000;
        MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
        MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
        MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
        MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
        MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
        MPU_InitStruct.Number = MPU_REGION_NUMBER0;
        MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
        MPU_InitStruct.SubRegionDisable = 0x00;
        MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

        HAL_MPU_ConfigRegion (&MPU_InitStruct);

        /* Enable the MPU */
        HAL_MPU_Enable (MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable (void)
{
        /* Enable I-Cache */
        SCB_EnableICache ();

        /* Enable D-Cache */
        SCB_EnableDCache ();
}
