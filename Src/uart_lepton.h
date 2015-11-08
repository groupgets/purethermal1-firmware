#ifndef LEPTON_H_
#define UART_LEPTON_H_

void send_lepton_via_usart(char * buffer);
PT_THREAD( uart_lepton_send(struct pt *pt,char * buffer));

#endif

