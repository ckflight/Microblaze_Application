/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "stdbool.h"

#include "xgpio.h"
#include "xuartlite.h"
#include "xspi.h"
#include <unistd.h>

#define LED 			0xFFFF
#define LED_DELAY     	1000000
#define LED_CHANNEL 	1

XSpi SpiInstance;

XGpio Gpio; /* The Instance of the GPIO Driver */

XUartLite UartLite;		/* Instance of the UartLite Device */

uint8_t uart_rx_buffer[512];
uint8_t uart_tx_buffer[512];

uint32_t gpio_state = 0x00000000;  // Tracks current output state

void toggle_led(int led_num) {
    uint32_t bit_mask = 1u << (led_num - 1);

    // Toggle the bit
    gpio_state ^= bit_mask;

    // Write updated state
    XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, gpio_state);
}

uint8_t Read_ADXL362_DeviceID()
{
	uint8_t send_buf[3] = {0x0B, 0x02, 0xFF};  // Read command, DEVID_AD, dummy
	uint8_t recv_buf[3] = {0};

	// Select the slave (CS low)
	XSpi_SetSlaveSelect(&SpiInstance, 1);

	// Full duplex SPI transfer
	XSpi_Transfer(&SpiInstance, send_buf, recv_buf, 3);

	// Select the slave (CS low)
	XSpi_SetSlaveSelect(&SpiInstance, 0);

	// recv_buf[2] should now contain 0xAD
	return recv_buf[2];
}

int main(void)
{
	int Status;

	/* Initialize the GPIO driver */
	Status = XGpio_Initialize(&Gpio, XPAR_XGPIO_0_BASEADDR);
	if (Status != XST_SUCCESS) {
		xil_printf("Gpio Initialization Failed\r\n");
		return XST_FAILURE;
	}

	XGpio_SetDataDirection(&Gpio, LED_CHANNEL, 0x0000); // 0 is output set all output

	for(int j = 0; j < 10; j++){
        
		XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, LED);

		usleep(100000);
		
		XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, ~LED);
        
		usleep(100000);
	}

	/*
	 * Initialize the UartLite driver so that it is ready to use.
	 */
	Status = XUartLite_Initialize(&UartLite, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Status = XUartLite_SelfTest(&UartLite);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    Status = XSpi_Initialize(&SpiInstance, XPAR_AXI_QUAD_SPI_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI Initialization Failed\r\n");
        return XST_FAILURE;
    }

    // Set options: master mode and manual slave select
    Status = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI SetOptions Failed\r\n");
        return XST_FAILURE;
    }

    // Start the SPI driver
    XSpi_Start(&SpiInstance);

    // Disable global interrupt mode
    XSpi_IntrGlobalDisable(&SpiInstance);

    u8 dev_id = Read_ADXL362_DeviceID();

	u8 send_buf[3] = {0x0B, 0x02, 0xFF};  // Read command, DEVID_AD, dummy
	u8 recv_buf[4];

	// Select the slave (CS low)
	XSpi_SetSlaveSelect(&SpiInstance, 1);

	// Full duplex SPI transfer
	XSpi_Transfer(&SpiInstance, send_buf, recv_buf, 3);

	// Deselect all slaves manually when done
	XSpi_SetSlaveSelect(&SpiInstance, 0);

	dev_id = recv_buf[0];
	dev_id = recv_buf[1];
	dev_id = recv_buf[2];

	int counter = 0;

	while (1) {

		uart_tx_buffer[0] = counter++;
        uart_tx_buffer[0] = recv_buf[2];
        
		XUartLite_Send(&UartLite, uart_tx_buffer, 1);

		while(XUartLite_Recv(&UartLite, uart_rx_buffer, 1) == 0);

		if(counter > 255){
			counter = 0;
		}

		// Select the slave (CS low)
		XSpi_SetSlaveSelect(&SpiInstance, 1);

		// Full duplex SPI transfer
		XSpi_Transfer(&SpiInstance, send_buf, recv_buf, 4);

		XSpi_SetSlaveSelect(&SpiInstance, 0);


        toggle_led(1);
		toggle_led(2);
		toggle_led(3);
		toggle_led(5);	
		toggle_led(10);
		toggle_led(16);

		//usleep(50000);
	}

}