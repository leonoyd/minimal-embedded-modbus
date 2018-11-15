/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef __linux__
#define __linux__ 0
#endif

#if __linux__
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#else
#include "ch.h"
#include "hal.h"
#include "hal_uart_lld.h"
#include "chtime.h"

#include "usbcfg.h"
#include "chprintf.h"
#include "l3gd20.h"
#endif

#include <stdint.h>
#include <stdbool.h>

#if !__linux__
#define cls(chp)  chprintf(chp, "\033[2J\033[1;1H")
extern UARTDriver UARTD6;

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  0,
  19200,
  0,
  USART_CR2_LINEN,
  0
};

/*===========================================================================*/
/* L3GD20 related.                                                           */
/*===========================================================================*/

void modbus_driver_thread();
/* L3GD20 Driver: This object represent an L3GD20 instance.*/
static L3GD20Driver L3GD20D1;

static int32_t gyroraw[L3GD20_GYRO_NUMBER_OF_AXES];
static float gyrocooked[L3GD20_GYRO_NUMBER_OF_AXES];

static char axisID[L3GD20_GYRO_NUMBER_OF_AXES] = {'X', 'Y', 'Z'};
static uint32_t i;

static const SPIConfig spicfg = {
  FALSE,
  NULL,
  GPIOE,
  GPIOE_L3GD20_CS,
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  0
};

static L3GD20Config l3gd20cfg = {
  &SPID1,
  &spicfg,
  NULL,
  NULL,
  L3GD20_FS_250DPS,
  L3GD20_ODR_760HZ,
#if L3GD20_GYRO_USE_ADVANCED
  L3GD20_BDU_CONTINUOUS,
  L3GD20_END_LITTLE,
  L3GD20_BW3,
  L3GD20_HPM_REFERENCE,
  L3GD20_HPCF_8,
  L3GD20_LP2M_ON,
#endif
};

/*===========================================================================*/
/* Generic code.                                                                */
/*===========================================================================*/

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
modbus_driver_thread();
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time;

    time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
    palClearLine(LINE_LED5);
    chThdSleepMilliseconds(time);
    palSetLine(LINE_LED5);
    chThdSleepMilliseconds(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {

//modbus_driver_thread();
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
#if 1
  halInit();
  chSysInit();

  /* Initializes a serial-over-USB CDC driver.*/
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* L3GD20 Object Initialization.*/
  l3gd20ObjectInit(&L3GD20D1);

  /* Activates the L3GD20 driver.*/
  l3gd20Start(&L3GD20D1, &l3gd20cfg);

  /* Normal main() thread activity, printing MEMS data on the SDU1.*/
  while (true) {
    l3gd20GyroscopeReadRaw(&L3GD20D1, gyroraw);
    //chprintf(chp, "L3GD20 Gyroscope raw data...\r\n");
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      //chprintf(chp, "%c-axis: %d\r\n", axisID[i], gyroraw[i]);
    }

    l3gd20GyroscopeReadCooked(&L3GD20D1, gyrocooked);
    //chprintf(chp, "L3GD20 Gyroscope cooked data...\r\n");
    for(i = 0; i < L3GD20_GYRO_NUMBER_OF_AXES; i++) {
      //chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], gyrocooked[i]);
    }

    chThdSleepMilliseconds(100);
    cls(chp);
  }
  l3gd20Stop(&L3GD20D1);
#endif
}
#endif
#define MAX_NUM_MODBUS_REGISTERS 12
uint16_t modbus_registers[MAX_NUM_MODBUS_REGISTERS];
typedef enum {
	MB_FRAME_SLAVE_ADDR,
	MB_FRAME_FUNCTION,
	MB_FRAME_STARTING_ADDR_HI,
	MB_FRAME_STARTING_ADDR_LOW,
	MB_FRAME_NO_REG_HI,
	MB_FRAME_NO_REG_LOW,
	MB_FRAME_CRC_HI,
	MB_FRAME_CRC_LOW
}mb_struct_t;

typedef enum {
    MB_TX_FRAME_ADDR,
    MB_TX_FRAME_FUNCTION,
    MB_TX_FRAME_COUNT,
    MB_TX_FRAME_DATA
} mb_tx_header_t;

#if __linux__
long start_time = 0;
long timeout = 2000;
unsigned long get_time()
{
struct timeval tv;
gettimeofday(&tv, NULL);
unsigned long ret = tv.tv_usec;
ret /= 1000;
ret += (tv.tv_sec * 1000);
return ret;
}
#else
systime_t start_time;
systime_t timeout;
#endif

bool timer_expired()
{
	return false;//((get_time()-start_time) >= timeout);
}

void stop_timer()
{
	start_time = 0;
}

void restart_timer()
{
#if __linux__
	start_time = get_time();
#else
	start_time = chVTGetSystemTime();
#endif
}
uint16_t construct_response(uint8_t* modbus_tx_frame, uint8_t* modbus_rx_frame,
                        uint16_t tx_data_len)
{
    //reconstruct packet
    //
    modbus_tx_frame[MB_TX_FRAME_ADDR] = modbus_rx_frame[MB_FRAME_SLAVE_ADDR];
    modbus_tx_frame[MB_TX_FRAME_FUNCTION] = modbus_rx_frame[MB_FRAME_FUNCTION];
    modbus_tx_frame[MB_TX_FRAME_COUNT] = tx_data_len;

    uint16_t crc_index = tx_data_len + 2;

    uint16_t calculated_crc = usMBCRC16(modbus_tx_frame, crc_index);

    modbus_tx_frame[crc_index] = (uint8_t)calculated_crc;
    modbus_tx_frame[crc_index+1] = (uint8_t)(calculated_crc >> 8);

    return crc_index + 2;
}
#define MODBUS_TX_FRAME_SIZE 20
#define MODBUS_RX_FRAME_SIZE 8

#if __linux__
int fd = 0;
#endif

void send_response(uint8_t* modbus_frame, uint16_t len)
{
    //send packets to uart here
#if __linux__
	int bytes_written = write(fd, modbus_frame, len);
#else
	uartStartSend(chp, len, modbus_frame);
#endif
}

bool is_frame_valid(uint8_t* mb_frame, uint16_t len)
{
	bool ret_val = 0;
	uint16_t calculated_crc = usMBCRC16(mb_frame, len);
	uint16_t aquired_crc = (mb_frame[MB_FRAME_CRC_HI] & 0x00FF)<<8
				| mb_frame[MB_FRAME_CRC_LOW];

	if (calculated_crc == aquired_crc) {
		ret_val = true;
	}
	return ret_val;
}

bool process_request(uint8_t* frame, uint16_t len, uint16_t num_reg_requested,
			uint16_t start_addr)
{
	if ((num_reg_requested + start_addr) > MAX_NUM_MODBUS_REGISTERS) {
		//construct and return error
        return false;
	}

    memcpy(&frame[MB_TX_FRAME_DATA], &modbus_registers[start_addr], num_reg_requested);

    return true;
}

int8_t poll_uart_for_character()
{
    int8_t ch = -1;
#if __linux__
    int bytes_read = read(fd, &ch, 4);
    if (bytes_read > 0) {
        return ch;
    } else if (bytes_read == -1) {
        return -1;//perror("Error is");
    } else {
        return -1;
    }
#else
	uartStartReceive(chp, 1, (void*)&ch);
	return ch;
#endif
}

void wait_for_start_signal()
{
	restart_timer();
     while (!timer_expired()) {
        int8_t ch = poll_uart_for_character();
        if (ch > -1) {
            restart_timer();
        }
    }
}


void handle_rx_packet(uint8_t* mb_frame, uint16_t mb_frame_size)
{
	int16_t ch = -1;
	uint8_t frame_ptr = 0;
    bool packet_received = false;
    //wait_for_start_signal();
	while (!packet_received) {
		while (ch < 0) {
			ch = poll_uart_for_character();
			if (frame_ptr == mb_frame_size) {
				if (timer_expired()) {
					break;
				}
			}
		}

		chprintf(chp, "got a char: %d\r\n", ch);
		if (timer_expired()) {
			if (frame_ptr == mb_frame_size) {
                packet_received = true;
			} else {

                frame_ptr = 0;
				stop_timer();
			}
		} else {

                //printf("timer not expFrame ptr is: %d\n", frame_ptr);
			mb_frame[frame_ptr] = ch;
			frame_ptr++;
		}
        ch = -1;
        restart_timer();
	}
}

#if __linux__
int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf("error %d from tcgetattr", errno);
            return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            printf ("error %d from tcsetattr", errno);
            return -1;
    }
    return 0;
}

void
set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tggetattr", errno);
            return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes", errno);
}
char *portname = "/dev/pts/7";
#endif

#define MY_MB_ADDRESS 1

void modbus_driver_thread()
{
	uint8_t modbus_tx_frame[MODBUS_TX_FRAME_SIZE];
	uint8_t modbus_rx_frame[MODBUS_RX_FRAME_SIZE];

	timeout = TIME_US2I(2000);
#if __linux__
	fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd <-1)
        perror("failed to open port");
	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 1);
#else
	/*
   * Activates the serial driver 1, PA9 and PA10 are routed to USART2.
   */
  uartStart(&UARTD6, &uart_cfg_1);

#endif

	while (1) {
		handle_rx_packet(modbus_rx_frame, MODBUS_RX_FRAME_SIZE);

        if((modbus_rx_frame[MB_FRAME_SLAVE_ADDR] == MY_MB_ADDRESS)
            && ((modbus_rx_frame[MB_FRAME_FUNCTION] == 0x03)
           || (modbus_rx_frame[MB_FRAME_FUNCTION] == 0x00))) {

            if (is_frame_valid(modbus_rx_frame, MODBUS_RX_FRAME_SIZE)) {
                uint16_t num_reg_requested = (modbus_rx_frame[MB_FRAME_NO_REG_HI] & 0x00FF)<<8
                            | modbus_rx_frame[MB_FRAME_NO_REG_LOW];


                uint16_t start_addr = (modbus_rx_frame[MB_FRAME_STARTING_ADDR_HI] & 0x00FF)<<8
                            | modbus_rx_frame[MB_FRAME_STARTING_ADDR_LOW];

                if (process_request(modbus_tx_frame, MODBUS_TX_FRAME_SIZE,
                                    num_reg_requested, start_addr)) {
                    uint16_t tx_packet_len = construct_response(modbus_tx_frame, modbus_rx_frame, num_reg_requested * 2);
                    send_response(modbus_tx_frame, tx_packet_len);
                }
            }
	    }
    }
}
#if __linux__
void main()
{
    modbus_driver_thread();
}
#endif

