#ifndef CONNECT_H_
#define CONNECT_H_

#define LWIP_INIT BIT0
#define DHCP_START BIT1
#define IPV4_ADQUIRED BIT2

void WiFi_lwIP_Connection(void);

#endif