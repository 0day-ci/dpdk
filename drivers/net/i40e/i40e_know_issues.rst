Cannot set link speed on Intel® 40G Ethernet controller
-------------------------------------------------------

**Description**:
   On Intel® 40G Ethernet Controller you cannot set the link to specific speed.

**Implication**:
   The link speed cannot be changed forcibly, though it can be configured by application.

**Resolution/Workaround**:
   None

**Affected Environment/Platform**:
   All.


I40e VF may not receive packets in the promiscuous mode
-------------------------------------------------------

**Description**:
   Promiscuous mode is not supported by the DPDK i40e VF driver when using the
   i40e Linux kernel driver as host driver.

**Implication**:
   The i40e VF does not receive packets when the destination MAC address is unknown.

**Resolution/Workaround**:
   Use a explicit destination MAC address that matches the VF.

**Affected Environment/Platform**:
   All.


uio pci generic module bind failed in X710/XL710/XXV710
-------------------------------------------------------

**Description**:
   The ``uio_pci_generic`` module is not supported by XL710, since the errata of XL710
   states that the Interrupt Status bit is not implemented. The errata is the item #71
   from the `xl710 controller spec
   <http://www.intel.com/content/www/us/en/embedded/products/networking/xl710-10-40-controller-spec-update.html>`_.
   The hw limitation is the same as other X710/XXV710 NICs.

**Implication**:
   When use ``--bind=uio_pci_generic``, the ``uio_pci_generic`` module probes device and check the Interrupt
   Status bit. Since it is not supported by X710/XL710/XXV710, it return a *failed* value. The statement
   that these products don’t support INTx masking, is indicated in the related `linux kernel commit
   <https://git.kernel.org/cgit/linux/kernel/git/stable/linux-stable.git/commit/drivers/pci/quirks.c?id=8bcf4525c5d43306c5fd07e132bc8650e3491aec>`_.

**Resolution/Workaround**:
   Do not bind the ``uio_pci_generic`` module in X710/XL710/XXV710 NICs.

**Affected Environment/Platform**:
   All.
