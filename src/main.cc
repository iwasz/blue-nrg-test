/****************************************************************************
 *                                                                          *
 *  Author : lukasz.iwaszkiewicz@gmail.com                                  *
 *  ~~~~~~~~                                                                *
 *  License : see COPYING file for details.                                 *
 *  ~~~~~~~~~                                                               *
 ****************************************************************************/

#include <stm32f4xx_hal.h>
#include "config.h"
#include "errorHandler.h"
extern "C" {
#include "osal.h"
}
#include "sensor_service.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"

#include <cstdio>

#define BDADDR_SIZE 6

extern volatile uint8_t set_connectable;
extern volatile int connected;
extern AxesRaw_t axes_data;
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */
                                        /**
                                         * @}
                                         */

/** @defgroup MAIN_Private_Function_Prototypes
 * @{
 */
/* Private function prototypes -----------------------------------------------*/
void User_Process (AxesRaw_t *p_axes);
static void systemClockConfig ();

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

        /* STM32Cube HAL library initialization:
         *  - Configure the Flash prefetch, Flash preread and Buffer caches
         *  - Systick timer is configured by default as source of time base, but user
         *    can eventually implement his proper time base source (a general purpose
         *    timer for example or other time source), keeping in mind that Time base
         *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         *    handled in milliseconds basis.
         *  - Low Level Initialization
         */
        HAL_Init ();

#if NEW_SERVICES
        /* Configure LED2 */
        BSP_LED_Init (LED2);
#endif

        /* Configure the system clock */
        systemClockConfig ();

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

        PRINTF ("HWver %d, FWver %d", hwVersion, fwVersion);

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
                PRINTF ("Setting BD_ADDR failed.\n");
        }

        ret = aci_gatt_init ();
        if (ret) {
                PRINTF ("GATT_Init failed.\n");
        }

        if (bnrg_expansion_board == IDB05A1) {
                ret = aci_gap_init_IDB05A1 (GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
        }
        else {
                ret = aci_gap_init_IDB04A1 (GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
        }

        if (ret != BLE_STATUS_SUCCESS) {
                PRINTF ("GAP_Init failed.\n");
        }

        ret = aci_gatt_update_char_value (service_handle, dev_name_char_handle, 0, strlen (name), (uint8_t *)name);

        if (ret) {
                PRINTF ("aci_gatt_update_char_value failed.\n");
                while (1)
                        ;
        }

        ret = aci_gap_set_auth_requirement (MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING);
        if (ret == BLE_STATUS_SUCCESS) {
                PRINTF ("BLE Stack Initialized.\n");
        }

        PRINTF ("SERVER: BLE Stack Initialized\n");

        ret = Add_Acc_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                PRINTF ("Acc service added successfully.\n");
        else
                PRINTF ("Error while adding Acc service.\n");

        ret = Add_Environmental_Sensor_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                PRINTF ("Environmental Sensor service added successfully.\n");
        else
                PRINTF ("Error while adding Environmental Sensor service.\n");

#if NEW_SERVICES
        /* Instantiate Timer Service with two characteristics:
         * - seconds characteristic (Readable only)
         * - minutes characteristics (Readable and Notifiable )
         */
        ret = Add_Time_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                PRINTF ("Time service added successfully.\n");
        else
                PRINTF ("Error while adding Time service.\n");

        /* Instantiate LED Button Service with one characteristic:
         * - LED characteristic (Readable and Writable)
         */
        ret = Add_LED_Service ();

        if (ret == BLE_STATUS_SUCCESS)
                PRINTF ("LED service added successfully.\n");
        else
                PRINTF ("Error while adding LED service.\n");
#endif

        /* Set output power level */
        ret = aci_hal_set_tx_power_level (1, 4);

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
                        // PRINTF("ACC: X=%6d Y=%6d Z=%6d\r\n", p_axes->AXIS_X, p_axes->AXIS_Y, p_axes->AXIS_Z);
                        Acc_Update (p_axes);
                }
        }
}

/*****************************************************************************/

static void systemClockConfig ()
{
        /*
         * Power interface clock enable. This is quivalent to RCC_AHB1PeriphClockCmd (xxx)
         * which seems to be now obsolete.
         */
        __HAL_RCC_PWR_CLK_ENABLE ();

        /*
         * The voltage scaling allows optimizing the power consumption when the device is
         * clocked below the maximum system frequency.
         *
         * To dotyczy zasilania rdzenia (1.2V). Default is 1 (full voltage), min volatge
         * to będzie opcja 3.
         *
         * Można to ustawienie zmienić *tylko* gdy PLL off, a źródłem jest bezpośrednio HSE
         * lub HSI. Nowe ustawienie zaczyna działać kiedy włączy się PLL (poraz pierwszy lub
         * spowrotem).
         */
        __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

        RCC_OscInitTypeDef rccOscInitStruct;
        rccOscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // HSE, HSI, LSE, LSI, NONE
        rccOscInitStruct.HSEState = RCC_HSE_ON;                   // ON, OFF, BYPASS
        rccOscInitStruct.HSIState = RCC_HSI_OFF;
        rccOscInitStruct.LSEState = RCC_LSE_OFF;
        rccOscInitStruct.LSIState = RCC_LSI_OFF;
        rccOscInitStruct.PLL.PLLState = RCC_PLL_ON;         // On / Off
        rccOscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // HSE or HSI
        rccOscInitStruct.PLL.PLLM = 16;                     // Between 0 and 63. 8 dla STM32F4-DISCO, 16 dla gpA
        rccOscInitStruct.PLL.PLLN = 336;                    // Betwen 192 and 432
        rccOscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;          // RCC_PLLP_DIV2, RCC_PLLP_DIV4, RCC_PLLP_DIV6, RCC_PLLP_DIV8
        rccOscInitStruct.PLL.PLLQ = 7;                      // Between 4 and 15.

        if (HAL_RCC_OscConfig (&rccOscInitStruct) != HAL_OK) {
                Error_Handler ();
        }

        RCC_ClkInitTypeDef rccClkInitStruct;

        // ClockType mówi które zegary konfigurujemy. W tym przypadku konfigurujemy wszytskie.
        rccClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
        rccClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // HSI, HSE lub PLL
        rccClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        rccClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        rccClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

        if (HAL_RCC_ClockConfig (&rccClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
                Error_Handler ();
        }
}
