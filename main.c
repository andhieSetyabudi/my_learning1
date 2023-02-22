/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include <ctype.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
/* Include emUSB-Device headers */
#include "USB.h"
#include "USB_CDC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/
static const USB_DEVICE_INFO device_info = {
	0x058B,                       /* VendorId    */
	0x027D,                       /* ProductId    */
    "Andhie",                     /* VendorName */
    "USB-CDC",  /* ProductName */
    "12345678"                    /* SerialNumber */
};

typedef struct {
    uint16_t key_code;
    char     key_char;
} code_to_desc_t;

static USB_CDC_HANDLE usb_cdcHandle;
static char           read_buffer[USB_FS_BULK_MAX_PACKET_SIZE];
static char           write_buffer[USB_FS_BULK_MAX_PACKET_SIZE];

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void usb_add_cdc(void);
void task_emUSB(void* param);
void task_user(void* param);
/*******************************************************************************
* Function Definitions
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();


    /* Initialize retarget-io to use the debug UART port */
	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	init_cycfg_peripherals();




	printf("\x1b[2J\x1b[;H");

	    printf("****************** "
	           "emUSB Device: CDC echo application "
	           "****************** \r\n\n");
	xTaskCreate(task_emUSB, "emUSB Task", 500U,NULL, configMAX_PRIORITIES - 6, NULL);
	xTaskCreate(task_user, "user Task", 500U,NULL, configMAX_PRIORITIES - 5, NULL);
	vTaskStartScheduler();
    for (;;)
    {
    	printf("\x1b[2J\x1b[;H");

    	    printf("****************** "
    	           "emUSB Device: CDC echo application "
    	           "****************** \r\n\n");

    	    cyhal_system_delay_ms(1000);
    }
}

/*********************************************************************
* Function Name: USBD_CDC_Echo_Init
**********************************************************************
* Summary:
*  Add communication device class to USB stack
*
* Parameters:
*  void
*
* Return:
*  void
**********************************************************************/

void usb_add_cdc(void) {

    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    EPBulkIn.Flags          = 0;                             /* Flags not used */
    EPBulkIn.InDir          = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPBulkIn.Interval       = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    EPBulkOut.Flags         = 0;                             /* Flags not used */
    EPBulkOut.InDir         = USB_DIR_OUT;                   /* OUT direction (Host to Device) */
    EPBulkOut.Interval      = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

    EPIntIn.Flags           = 0;                             /* Flags not used */
    EPIntIn.InDir           = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPIntIn.Interval        = 64;                            /* Interval of 8 ms (64 * 125us) */
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;   /* Maximum packet size (64 for Interrupt) */
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         /* Endpoint type - Interrupt */
    InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

    usb_cdcHandle = USBD_CDC_Add(&InitData);


}

void task_emUSB(void* param)
{
	/* Suppress warning for unused parameter */
    (void)param;

    /* Initializes the USB stack */
    	USBD_Init();

    	/* Endpoint Initialization for CDC class */
    	usb_add_cdc();

    	/* Set device info used in enumeration */
    	USBD_SetDeviceInfo(&device_info);

    	USBD_X_EnableInterrupt();
    	/* Start the USB stack */
    	USBD_Start();

    	const char* hl="hello user \r\n";
    	while ( (USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
		{
    		vTaskDelay(10);

		}
    /* Repeatedly running part of the task */
	for(;;)
	{
		cyhal_system_delay_ms(1000);
		/* Sending one packet to host */
		USBD_CDC_Write(usb_cdcHandle, hl, strlen(hl), 0);

		/* Waits for specified number of bytes to be written to host */
		USBD_CDC_WaitForTX(usb_cdcHandle, 0);
	}
}

bool callback_Stats= false;
void callbackFunction(void *callback_arg, cyhal_gpio_event_t event)
{
	callback_Stats = true;
}
void task_user(void* param)
{
	uint8_t phase_t = 0;
	callback_Stats = false;

	// setup pin
	// user button pin as input pullup
	cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
	// user led pin as output
//	cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	/* PWM using HAL*/
	//try to setup
		cyhal_pwm_t myPWM;
		//                   obj,  pin,complementary_pin, alignment,continuous,dead-time, pin inverting
		cyhal_pwm_init_adv(&myPWM, CYBSP_USER_LED, NC, CYHAL_PWM_LEFT_ALIGN, true, 0, false, NULL);
		// for set duty cycle
		cyhal_pwm_set_duty_cycle(&myPWM, 50.f, 1);
		// for set the period
  //    cyhal_pwm_set_period(&myPWM, period_us, pulse_width_us)

		// start that PWM
		cyhal_pwm_start(&myPWM);
	 /*end of PWM HAL*/


	/*Interrupt button using HAL*/
		cyhal_gpio_callback_data_t my_callback_data = {.callback = callbackFunction};
		cyhal_gpio_register_callback(CYBSP_USER_BTN, &my_callback_data);
		cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
	/* end of interrupt HAL*/
	for(;;)
	{
		if( callback_Stats )
		{
			callback_Stats = false;
			phase_t ++;
			if( phase_t > 5)
				phase_t = 0;
			switch(phase_t)
			{
				case 1:
						cyhal_pwm_set_duty_cycle(&myPWM, 10.f, 1);
						break;
				case 2:
						cyhal_pwm_set_duty_cycle(&myPWM, 20.f, 1);
						break;
				case 3:
						cyhal_pwm_set_duty_cycle(&myPWM, 30.f, 1);
						break;
				case 4:
						cyhal_pwm_set_duty_cycle(&myPWM, 40.f, 1);
						break;
				case 5:
						cyhal_pwm_set_duty_cycle(&myPWM, 50.f, 1);
						break;
				default:
					cyhal_pwm_set_duty_cycle(&myPWM, 70.f, 1);
					break;
			}
			printf("****************** "
				           "i'm here "
				           "****************** \r\n\n");
		}
		vTaskDelay(100);
	}
}
/* [] END OF FILE */
