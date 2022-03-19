/* $Id: main.c,v 1.36 2019/03/12 19:24:19 bouyer Exp $ */
/*
 * Copyright (c) 2021 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <nmea2000.h>
#include <nmea2000_pgn.h>
#include <raddeg.h>
#include "serial.h"
#include "i2c.h"
#include "bme280.h"

unsigned int devid, revid;
unsigned long nmea2000_user_id;

static unsigned char sid;

static struct nmea2000_msg msg;
static unsigned char nmea2000_data[NMEA2000_DATA_LENGTH];

unsigned int timer0_read(void);

#define TIMER0_5MS 48
#define TIMER0_1MS 10

#define LED LATBbits.LATB0
#define PWR_ANEMO LATAbits.LATA0

static unsigned char led_pattern;

static char counter_10hz;
static char counter_1hz;
static unsigned char seconds;
static volatile union softintrs {
	struct softintrs_bits {
		char int_10hz : 1;	/* 0.1s timer */
		char anemo_rx : 1;	/* NMEA string ready */
	} bits;
	char byte;
} softintrs;

static enum i2c_status {
	IDLE,
	READ_TEMP,
	READ_HUM,
	SEND_TEMP,
	SEND_HUM,
	FAIL
} i2c_status;

struct bme280_dev bme280_dev;

int32_t temp; /* degK * 100 */
uint32_t hum; /* %RH * 1000 */
uint32_t press; /* Pa */
uint8_t bme280_id;
static char counter_sth20;
static uint16_t i2cval;

#define ANEMO_BUFSIZE 128
static char anemo_rxbuf1[ANEMO_BUFSIZE];
static char anemo_rxbuf2[ANEMO_BUFSIZE];
static unsigned char anemo_rxbufi;
static unsigned char anemo_rxbufs;
static unsigned char anemo_rxbufs_ready;

#define ANEMO_DIR_OFFSET 0 /* deg * 10 */

static uint16_t wind_speed; /* kn * 10 */
static uint16_t wind_angle; /* deg * 10 */
static unsigned char wind_ok;

static void
send_env_param(void)
{
	struct nmea2000_env_param *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_ENV_PARAM, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_env_param);
	msg.data = &nmea2000_data[0];
	data->sid  = sid;
	data->tsource = ENV_TSOURCE_OUTSIDE;
	data->hsource = ENV_HSOURCE_OUTSIDE;
	data->temp = (uint16_t)temp;
	data->hum = (uint16_t)(hum / 4);
	data->press = 0xffff;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_ENV_PARAM failed\n");
}

static void
send_wind_data(void)
{
	struct nmea2000_wind_data *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_WIND_DATA, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_wind_data);
	msg.data = &nmea2000_data[0];
	data->sid  = sid;
	if (wind_ok) {
		data->speed = (uint16_t)(
			(unsigned long)wind_speed * 1852UL / 360UL);
			/* (wind_speed / 10) * (1852 / 3600) * 100 */
		data->dir = deg2urad(wind_angle / 10);
	} else {
		data->speed = 0xffff;
		data->dir = 0xffff;
	}
	data->ref = WIND_REF_APPARENT;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_WIND_DATA failed\n");
}

void
user_handle_iso_request(unsigned long pgn)
{
	printf("ISO_REQUEST for %ld from %d\n", pgn, rid.saddr);
	switch(pgn) {
	case NMEA2000_ENV_PARAM:
		send_env_param();
		break;
	case NMEA2000_WIND_DATA:
		send_wind_data();
		break;
	}
}

void
user_receive()
{
#if 0
	unsigned long pgn;

	pgn = ((unsigned long)rid.page << 16) | ((unsigned long)rid.iso_pg << 8);
	if (rid.iso_pg > 239)
		pgn |= rid.daddr;

	switch(pgn) {
	case PRIVATE_COMMAND_STATUS:
	{
		struct private_command_status *command_status = (void *)rdata;
		last_command_data = timer0_read();
		nmea2000_command_address = rid.saddr;
		command_received_heading = command_status->heading;
		received_auto_mode = command_status->auto_mode;
		received_param_slot = command_status->params_slot;
		break;
	}
	}
#endif
}

void
putch(char c)
{
        if (PORTBbits.RB7) {
		usart_putchar(c);
	}
}

static void
parse_anemo_rx(void)
{
	static char *rxbuf;
	static unsigned char i;
	unsigned char dot;

	i = 0;
	if (anemo_rxbufs_ready == 0) {
		rxbuf = anemo_rxbuf1;
		printf("anemo1 %s\n", rxbuf);
	} else {
		rxbuf = anemo_rxbuf2;
		printf("anemo2 %s\n", rxbuf);
	}
	if (*rxbuf != '$')
		return;
	rxbuf++; i++;

	for (; i < 10 ; i++, rxbuf++) {
		/* we should have the NMEA ID 'MWV' in the first few chars */
		if (*rxbuf == 0)
			return;
		if (rxbuf[0] == 'M' && rxbuf[1] == 'W' && rxbuf[2] == 'V') {
			rxbuf += 3;
			i += 3;
			break;
		}
		rxbuf++; i++;
	}
	printf("found MWV\n");
	if (*rxbuf != ',')
		return;
	rxbuf++; i++;
	dot = 0;
	wind_angle = 0;
	wind_ok = 0;
	while(*rxbuf != ',') {
		if (*rxbuf == 0 || i >= ANEMO_BUFSIZE)
			return;

		if (*rxbuf == '.') {
			rxbuf++; i++;
			dot++;
			continue;
		}
		if (*rxbuf < '0' || *rxbuf > '9')
			return;
		wind_angle = wind_angle * 10;
		wind_angle += *rxbuf - '0';
		rxbuf++; i++;
	}
	if (dot == 0)
		wind_angle = wind_angle * 10;
	wind_angle = wind_angle + ANEMO_DIR_OFFSET;
	if (wind_angle > 3600)
		wind_angle = wind_angle - 3600;
	printf("wangle %d\n", wind_angle);

	rxbuf++; i++; /* skip , */
	rxbuf++; i++; /* skip R/T */
	rxbuf++; i++; /* skip , */

	wind_speed = 0;
	dot = 0;
	while(*rxbuf != ',') {
		if (*rxbuf == 0 || i >= ANEMO_BUFSIZE)
			return;

		if (*rxbuf == '.') {
			rxbuf++; i++;
			dot++;
			continue;
		}
		if (*rxbuf < '0' || *rxbuf > '9')
			return;
		wind_speed = wind_speed * 10;
		wind_speed += *rxbuf - '0';
		rxbuf++; i++;
	}
	if (dot == 0)
		wind_speed = wind_speed * 10;
	printf("wspeed %d\n", wind_speed);

	rxbuf++; i++; /* skip , */
	switch(*rxbuf) {
		case 'N': /* knots */
			break;
		case 'M': /* m/s */
			wind_speed = (uint16_t)(
			    (unsigned long)wind_speed * 3600UL / 1852UL);
			break;
		case 'K': /* km/h */
			wind_speed = (uint16_t)(
			    (unsigned long)wind_speed * 1000UL / 1852UL);
			break;
		default:
			return;
	}
	rxbuf++; i++; /* skip N/M/K */
	rxbuf++; i++; /* skip , */
	if (*rxbuf == 'A')
		wind_ok = 1;
	printf("wok %d\n", wind_ok);
}

int
main(void)
{
	char c;
	static unsigned int poll_count;

	sid = 0;

	devid = 0;
	revid = 0;
	nmea2000_user_id = 0;

	softintrs.byte = 0;
	counter_10hz = 25;
	counter_1hz = 10;
	seconds = 0;
	i2c_status = IDLE;

	anemo_rxbufi = 0;
	anemo_rxbufs = 0;
	wind_ok = 0;

	/* read devid and user IDs */
	TBLPTR = 0x200000;
	asm("tblrd*+;");
	asm("movff TABLAT, _nmea2000_user_id;");
	asm("tblrd*+;");
	asm("movff TABLAT, _nmea2000_user_id + 1;");
	TBLPTR = 0x3ffffc;
        asm("tblrd*+;");
	asm("movff TABLAT, _revid;");
	asm("tblrd*+;");
	asm("movff TABLAT, _revid + 1;");
	asm("tblrd*+;");
	asm("movff TABLAT, _devid;");
	asm("tblrd*+;");
	asm("movff TABLAT, _devid + 1;");

	/* disable unused modules */
	PMD0 = 0x7a; /* keep clock and IOC */
	PMD1 = 0xfe; /* keep timer0 */
	PMD2 = 0x02; /* keep can module, TU16A */
	PMD3 = 0xff;
	PMD4 = 0xff;
	PMD5 = 0xff;
	PMD6 = 0xe6; /* keep UART2, UART1 and I2C */
	PMD7 = 0xff;
	PMD8 = 0xff;

	ANSELC = ANSELB = ANSELA = 0;
	LED = 0;
	PWR_ANEMO = 0;
	TRISBbits.TRISB0 = TRISAbits.TRISA0 = 0;
	LED = 1;

	/* CANRX on RB3 */
	CANRXPPS = 0x0B;
	/* CANTX on RB2 */
	LATBbits.LATB2 = 1; /* output value when idle */
	TRISBbits.TRISB2 = 0;
	RB2PPS = 0x46;

        /* configure watchdog timer for 2s */
	WDTCON0 = 0x16;
	WDTCON1 = 0x07;

	/* configure sleep mode: PRI_IDLE */
	CPUDOZE = 0x80;

	/* everything is low priority by default */
	IPR1 = 0;
	IPR2 = 0;
	IPR3 = 0;
	IPR4 = 0;
	IPR5 = 0;
	IPR6 = 0;
	IPR7 = 0;
	IPR8 = 0;
	IPR9 = 0;
	IPR10 = 0;
	IPR11 = 0;
	IPR12 = 0;
	IPR13 = 0;
	IPR14 = 0;
	IPR15 = 0;
	INTCON0 = 0;
	INTCON1 = 0;
	INTCON0bits.IPEN=1; /* enable interrupt priority */

	USART_INIT(0)

	/* configure timer0 as free-running counter at 9.765625Khz * 4 */
	T0CON0 = 0x0;
	T0CON0bits.MD16 = 1; /* 16 bits */
	T0CON1 = 0x48; /* 01001000 Fosc/4, sync, 1/256 prescale */
	PIR3bits.TMR0IF = 0;
	PIE3bits.TMR0IE = 0; /* no interrupt */
	T0CON0bits.T0EN = 1;

	/* configure UTMR for 10Hz interrupt */
	TU16ACON0 = 0x04; /* period match IE */
	TU16ACON1 = 0x00;
	TU16AHLT = 0x00; /* can't use hardware reset because of HW bug */
	TU16APS = 249; /* prescaler = 250 -> 40000 Hz */
	TU16APRH = (4000 >> 8) & 0xff;
	TU16APRL = 4000 & 0xff;
	TU16ACLK = 0x2; /* Fosc */
	TUCHAIN = 0;
	TU16ACON0bits.ON = 1;
	TU16ACON1bits.CLR = 1;
	IPR0bits.TU16AIP = 1; /* high priority interrupt */
	PIE0bits.TU16AIE = 1;

	I2C_INIT;

	/* configure UART2 (anemo) for 4800Bps at 10Mhz */
	U2RXPPS = 0x17; /* RC7 */
	U2BRGL = 129;
	U2BRGH = 0;
	U2CON0 = 0x10; /* RXEN */
	U2CON2 = 0x80; /* run during overflow */
	U2CON1 = 0; /* off for now */
	IPR8bits.U2RXIP = 0;

	INTCON0bits.GIEH=1;  /* enable high-priority interrupts */
	INTCON0bits.GIEL=1; /* enable low-priority interrrupts */

	printf("hello user_id 0x%lx devid 0x%x revid 0x%x\n", nmea2000_user_id,
	    devid, revid);

	nmea2000_init();
	poll_count = timer0_read();

	/* enable watch dog timer */
	WDTCON0bits.SEN = 1;

	if (bme280_init(&bme280_dev) != BME280_OK) {
		printf("bme280 fail\n");
	} else {
		printf("bme280 id 0x%x\n", bme280_dev.chip_id);
	}

	printf("ready");
	while (nmea2000_status != NMEA2000_S_OK) {
		nmea2000_poll(5);
		while ((timer0_read() - poll_count) < TIMER0_5MS) {
			nmea2000_receive();
		}
		poll_count = timer0_read();
		CLRWDT();
	}

	printf(", addr %d, id %ld\n", nmea2000_addr, nmea2000_user_id);

	/* enable anemo RX */
	U2CON1 = 0x80; /* on */
	PIE8bits.U2RXIE = 1;

	led_pattern = 0x55; /* 4 blinks at startup */

again:
	PWR_ANEMO = 1;
	printf("hello user_id 0x%lx devid 0x%x\n", nmea2000_user_id, devid);
	while (1) {
		CLRWDT();
		if (C1INTLbits.RXIF) {
			nmea2000_receive();
		}

		if (nmea2000_status == NMEA2000_S_CLAIMING) {
			if ((timer0_read() - poll_count) > TIMER0_5MS) {
				nmea2000_poll(5);
				poll_count = timer0_read();
			}
			if (nmea2000_status == NMEA2000_S_OK) {
				printf("new addr %d\n", nmea2000_addr);
			}
		};

		if (softintrs.bits.int_10hz) {
			softintrs.bits.int_10hz = 0;
			if (canbus_mute)
				PWR_ANEMO = 0;
			else
				PWR_ANEMO = 1;
			counter_1hz--;
			if (counter_1hz == 0) {
				counter_1hz = 10;
				seconds++;
				if (nmea2000_status != NMEA2000_S_OK &&
				    led_pattern == 0) {
					led_pattern = 0xf; /* one long blink */
				}
				if (canbus_mute) {
					led_pattern = 0x1; /* 1 short blink */
				} else if (wind_ok == 0) {
					led_pattern = 0x5; /* 2 short blink */
				}
				wind_ok = 0;
				if (seconds == 10) {
					seconds = 0;
					/* in normal state, one short
					 * blink every 10s
					 */
					if (led_pattern == 0)
						led_pattern = 1;
				} 
			}
			LED = (led_pattern & 0x1);
			led_pattern = led_pattern >> 1;
		}
		if (softintrs.bits.anemo_rx) {
			parse_anemo_rx();
			softintrs.bits.anemo_rx = 0;
			if (wind_ok) {
				send_wind_data();
				sid++;
				if (sid == 0xfe)
					sid = 0;
			}
		}

		if (PIR4bits.U1RXIF && (U1RXB == 'r'))
			break;

		if (nmea2000_status == NMEA2000_S_OK) {
			SLEEP();
		}
	}
	while ((c = getchar()) != 'r') {
		printf("resumed\n");
		goto again;
	}
end:
	WDTCON0bits.SEN = 0;
	printf("returning\n");
	while (!PIR4bits.U1TXIF)
		; /* wait for transmit to complete */
	LED = 0;
	PWR_ANEMO = 0;
	INTCON0bits.GIEH=0;
	INTCON0bits.GIEL=0;
	RESET();
	return 0;
}

unsigned int
timer0_read()
{
	unsigned int value;

	/* timer0_value = TMR0L | (TMR0H << 8), reading TMR0L first */
	di();
	asm("movff TMR0L, timer0_read@value");
	asm("movff TMR0H, timer0_read@value+1");
	ei();
	return value;
}

void
delay_ms(unsigned char delay)
{
	unsigned int date = timer0_read();
	unsigned int diff;
	do {
		diff = timer0_read() - date;
	} while (diff < delay * TIMER0_1MS);
}

void __interrupt(__irq(TU16A), __high_priority, base(IVECT_BASE))
irqh_tu16a(void)
{
	TU16ACON1bits.CLR = 1;
	TU16ACON1bits.PRIF = 0;
	softintrs.bits.int_10hz = 1;
}

void __interrupt(__irq(U2RX), __low_priority, base(IVECT_BASE))
irql_uart2(void)
{
	char c;
	while (PIE8bits.U2RXIE && PIR8bits.U2RXIF) {
		c = U2RXB;
		if (c == 0x0a) {
			; /* ignore */
		} else if (c == 0x0d) {
			anemo_rxbufs_ready = anemo_rxbufs;
			if (anemo_rxbufs == 0) {
				anemo_rxbuf1[anemo_rxbufi] = 0;
				anemo_rxbufs = 1;
			} else {
				anemo_rxbuf2[anemo_rxbufi] = 0;
				anemo_rxbufs = 0;
			}
			softintrs.bits.anemo_rx = 1;
			anemo_rxbufi = 0;
		} else {
			if (anemo_rxbufi < (ANEMO_BUFSIZE - 1)) {
				if (anemo_rxbufs == 0) {
					anemo_rxbuf1[anemo_rxbufi] = c;
				} else {
					anemo_rxbuf2[anemo_rxbufi] = c;
				}
				anemo_rxbufi++;
			}
		}
		if (U2ERRIRbits.RXFOIF) {
			U2ERRIRbits.RXFOIF = 0;
		}
	}
}
