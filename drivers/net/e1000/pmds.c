#include <rte_dev.h>
#include <rte_config.h>

#if RTE_LIBRTE_IGB_PMD
extern int
rte_igb_pmd_init(const char *name __rte_unused, const char *params __rte_unused);
extern int
rte_igbvf_pmd_init(const char *name __rte_unused, const char *params __rte_unused);

extern const struct rte_pci_id pci_id_igb_map[];
extern const struct rte_pci_id pci_id_igbvf_map[];

struct rte_driver __attribute__((used)) pmd_igb_drv = {
	.type = PMD_PDEV,
	.name = "igb_pmd",
	.init = rte_igb_pmd_init,
	.pci_table = pci_id_igb_map,
};

struct rte_driver __attribute__((used)) pmd_igbvf_drv = {
	.type = PMD_PDEV,
	.name = "igbvf_pmd",
	.init = rte_igbvf_pmd_init,
	.pci_table = pci_id_igbvf_map,
};

PMD_REGISTER_DRIVER(pmd_igb_drv);
PMD_REGISTER_DRIVER(pmd_igbvf_drv);
#endif

#if RTE_LIBRTE_EM_PMD
extern int
rte_em_pmd_init(const char *name __rte_unused, const char *params __rte_unused);

/*
 *  * The set of PCI devices this driver supports
 *   */
extern const struct rte_pci_id pci_id_em_map[];

static struct rte_driver __attribute__((used)) em_pmd_drv = {
	.type = PMD_PDEV,
	.name = "em_pmd",
	.init = rte_em_pmd_init,
	.pci_table = pci_id_em_map,
};

PMD_REGISTER_DRIVER(em_pmd_drv);
#endif
