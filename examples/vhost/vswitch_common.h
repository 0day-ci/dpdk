/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Freescale Semiconductor. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Freescale Semiconductor nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __VHOST_SWITCH_COMMON_H__
#define __VHOST_SWITCH_COMMON_H__

#include <sys/queue.h>
#include "main.h"

#define VSWITCH_NAME_SIZE	32

enum vswitch_port_type {
	VSWITCH_PTYPE_PHYS = 1,
	VSWITCH_PTYPE_VIRTIO,
	VSWITCH_PTYPE_END,
};
enum vswitch_port_state {
	VSWITCH_PSTATE_ADDED = 1,
	VSWITCH_PSTATE_LEARNING,
	VSWITCH_PSTATE_FORWARDING,
	VSWITCH_PSTATE_NOT_INUSE,
	VSWITCH_PSTATE_END,
};

struct vswitch_port {
	enum vswitch_port_type type;
	enum vswitch_port_state state;
	unsigned int port_id;
	struct rte_eth_conf port_conf;
	struct vswitch_dev *vs_dev;	/*switch to which this port belongs */
	void *priv;			/*Private for port specefic data*/
	struct ether_addr mac_addr;
	int phys_port_rxq;
	uint16_t (*do_tx) (struct vswitch_port *port, uint16_t tx_q,
			   __attribute__((unused))struct rte_mempool *mbuf_pool,
			   struct rte_mbuf **tx_pkts,	uint16_t pkt_count);
	uint16_t (*do_rx) (struct vswitch_port *port, uint16_t rx_q,
			   __attribute__((unused))struct rte_mempool *mbuf_pool,
			   struct rte_mbuf **rx_pkts,	uint16_t pkt_count);
};

struct vswitch_dev {
	char name[VSWITCH_NAME_SIZE];
	void *priv;				/* Private for switch specefic
						 * data */
	uint64_t conf_flags;
	LIST_ENTRY (vswitch_dev) list;
	struct vswitch_port *ports;
	int port_count;
	struct vswitch_ops *ops;
};

/* Function typedefs */
typedef int (*vs_switch_init_t) (struct vswitch_dev *, uint64_t conf_flags);
typedef int (*vs_add_port_t) (struct vswitch_port *vs_port);
typedef int (*vs_del_port_t) (struct vswitch_port *port);
typedef int (*vs_learn_port_t) (struct vswitch_port *port, struct rte_mbuf
				**pkts, uint16_t count);
typedef int (*vs_unlearn_port_t) (struct vswitch_port *port);
typedef int (*vs_lookup_n_fwd_t)(struct vswitch_port *in_port, struct rte_mbuf
			      **pkts, uint16_t pkt_count, uint16_t rxq);
#if 0
typedef uint16_t (*vs_do_tx_t) (struct vswitch_port *port, uint16_t tx_q, struct
			   rte_mbuf **tx_pkts,	uint16_t pkt_count);

typedef uint16_t (*vs_do_rx_t) (struct vswitch_port *port, uint16_t rx_q, struct
			   rte_mbuf **rx_pkts,	uint16_t pkt_count);
#endif
typedef struct vswitch_port* (*vs_sched_tx_port_t)(struct vswitch_dev *vs_dev,
				enum vswitch_port_type ptype, uint16_t core_id);

typedef struct vswitch_port* (*vs_sched_rx_port_t)(struct vswitch_dev *vs_dev,
				enum vswitch_port_type ptype, uint16_t core_id);

typedef int (*vs_port_start_t) (struct vswitch_port *port);

struct vswitch_ops {
	vs_add_port_t add_port;
	vs_del_port_t del_port;
	vs_lookup_n_fwd_t lookup_n_fwd;
	vs_learn_port_t learn_port;
	vs_unlearn_port_t unlearn_port;
	vs_port_start_t port_start;
	vs_port_start_t port_stop;
	vs_switch_init_t switch_init;
	vs_sched_tx_port_t sched_tx_port;
	vs_sched_rx_port_t sched_rx_port;

};

/*VSWITCH conf flags */
#define VS_CNF_FLG_VM2VM_HARDWARE	(1 << 0)
#define VS_CNF_FLG_VM2VM_SOFTWARE	(1 << 1)
#define VS_CNF_FLG_PROMISCOUS_EN	(1 << 2)
#define VS_CNF_FLG_JUMBO_EN		(1 << 3)
#define VS_CNF_FLG_VLAN_STRIP_EN	(1 << 4)
#define VS_CNF_FLG_STATS_EN		(1 << 5)

/*API for vswitch implementations*/
struct vswitch_dev *vs_register_switch(const char *name, int priv_size,
				int max_ports, struct vswitch_ops *ops);
int  vs_unregister_switch(struct vswitch_dev *dev);

int vs_vswitch_init(void);


/*API for user of vswitch like vhost-switch application */

struct vswitch_dev *vs_get_vswitch_dev(const char *name);
struct vswitch_port *vs_add_port(struct vswitch_dev *vs_dev, int port_id,
		enum vswitch_port_type type, void *priv);

int vs_del_port(struct vswitch_port *port);

int vs_switch_dev_init(struct vswitch_dev *vs_dev, uint16_t conf_flags);

int vs_port_start(struct vswitch_port *vs_port);
int vs_port_stop(struct vswitch_port *vs_port);
int vs_learn_port(struct vswitch_port *port, struct rte_mbuf
				**pkts, uint16_t count);
int vs_unlearn_port(struct vswitch_port *port);
int vs_lookup_n_fwd(struct vswitch_port *in_port, struct rte_mbuf
			      **pkts, uint16_t count, uint16_t in_rxq);

int vs_do_broadcast_fwd(struct vswitch_dev *vs_dev,
			struct vswitch_port *in_port, struct rte_mbuf *mbuf);

struct vswitch_port *vs_sched_tx_port(struct vswitch_dev *vs_dev, enum
				vswitch_port_type ptype, uint16_t core_id);

struct vswitch_port *vs_sched_rx_port(struct vswitch_dev *vs_dev, enum
				vswitch_port_type ptype, uint16_t core_id);
/*Extern APIs from vhost/main.c */

extern struct mbuf_table *vhost_switch_get_txq(uint16_t core_id);
extern int virtio_tx_local(struct vhost_dev *vdev, struct rte_mbuf *m);
extern struct vhost_dev *find_vhost_dev(struct ether_addr *mac);
void do_drain_mbuf_table(struct mbuf_table *tx_q);

/* TBD:XXX: This needs to be removed here, when constructor mechanism
 * for registering swittches is in place
 */
extern void vmdq_switch_impl_init(void);
#endif

