/*
 * YIC71513PGMGG.h
 *
 *  Created on: 28 Jan 2023
 *      Author: michaelliesenberg
 */

#ifndef APPLICATION_YIC71513PGMGG_H_
#define APPLICATION_YIC71513PGMGG_H_


void YIC71513PGMGG_uart_open();
void YIC71513PGMGG_uart_close();
void YIC71513PGMGG_uart_read();
void YIC71513PGMGG_read(int length);
void YIC71513PGMGG_uart_parse();


#endif /* APPLICATION_YIC71513PGMGG_H_ */
