#include <string.h>

#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_table_array.h>
#include <rte_kni.h>
#include <rte_ethdev.h>

#include "rte_port_kni.h"
#include "kni.h"

int
kni_config_network_interface(uint8_t port_id, uint8_t if_up)
{
    int ret = 0;

    if (port_id >= rte_eth_dev_count() || port_id >= RTE_MAX_ETHPORTS) {
        RTE_LOG(ERR, PORT, "%s: Invalid port id %d\n", __func__, port_id);
        return -EINVAL;
    }

    RTE_LOG(INFO, PORT, "%s: Configure network interface of %d %s\n",
            __func__, port_id, if_up ? "up" : "down");

    if (if_up != 0) { /* Configure network interface up */
        rte_eth_dev_stop(port_id);
        ret = rte_eth_dev_start(port_id);
    } else /* Configure network interface down */
        rte_eth_dev_stop(port_id);

    if (ret < 0)
        RTE_LOG(ERR, PORT, "%s: Failed to start port %d\n", __func__, port_id);

    return ret;
}

int
kni_change_mtu(uint8_t port_id, unsigned new_mtu)
{
    int ret;

    if (port_id >= rte_eth_dev_count()) {
        RTE_LOG(ERR, PORT, "%s: Invalid port id %d\n", __func__, port_id);
        return -EINVAL;
    }

    if (new_mtu > ETHER_MAX_LEN) {
        RTE_LOG(ERR, PORT, "%s: Fail to reconfigure port %d, the new MTU is too big\n", __func__, port_id);
        return -EINVAL;
    }

    RTE_LOG(INFO, PORT, "%s: Change MTU of port %d to %u\n", __func__, port_id, new_mtu);

    /* Stop specific port */
    rte_eth_dev_stop(port_id);

    /* Set new MTU */
    ret = rte_eth_dev_set_mtu(port_id, new_mtu);
    if (ret < 0) {
        RTE_LOG(ERR, PORT, "%s: Fail to reconfigure port %d\n", __func__, port_id);
        return ret;
    }

    /* Restart specific port */
    ret = rte_eth_dev_start(port_id);
    if (ret < 0) {
        RTE_LOG(ERR, PORT, "%s: Fail to restart port %d\n", __func__, port_id);
        return ret;
    }

    return 0;
}

