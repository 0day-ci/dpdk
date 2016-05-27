#ifndef __INCLUDE_KNI_H__
#define __INCLUDE_KNI_H__

#include <rte_common.h>

/* Total octets in ethernet header */
#define KNI_ENET_HEADER_SIZE    14

/* Total octets in the FCS */
#define KNI_ENET_FCS_SIZE       4

int kni_config_network_interface(uint8_t port_id, uint8_t if_up);
int kni_change_mtu(uint8_t port_id, unsigned new_mtu);

#endif