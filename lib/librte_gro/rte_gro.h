#ifndef _RTE_GRO_H_
#define _RTE_GRO_H_

/**
 * Initialize GRO environment for all ports. It should be called after
 * configuring all ethernet devices, and should be called just once.
 */
void
rte_gro_init(void);

/**
 * Enable GRO for a given port.
 * @param port_id
 *  The id of the port that is to enable GRO.
 * @param socket_id
 *  The NUMA socket id to which the ethernet device is connected.
 *  By default, it's value is SOCKET_ID_ANY.
 */
void
rte_gro_enable(uint8_t port_id, uint16_t socket_id);

/**
 * Disable GRO for a given port.
 * @param port_id
 *  The idd of the port that disables GRO.
 */
void
rte_gro_disable(uint8_t port_id);
#endif
