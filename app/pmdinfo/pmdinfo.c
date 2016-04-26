#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h>
#include <string.h>
#include <rte_dev.h>


static void dump_pci_table(struct rte_driver *driver)
{
	int i;

	if (!driver->pci_table) {
		printf(" No PCI Table defined for this driver\n");
		return;
	}

	printf("|====================PCI Table========================|\n");
	printf("| VENDOR ID | DEVICE ID | SUBVENDOR ID | SUBDEVICE ID |\n");
	printf("|-----------------------------------------------------|\n");
	for (i=0; driver->pci_table[i].vendor_id != 0; i++) {
		printf("|%11x|%11x|%14x|%14x|\n",
		driver->pci_table[i].vendor_id, driver->pci_table[i].device_id,
		driver->pci_table[i].subsystem_vendor_id,
		driver->pci_table[i].subsystem_device_id);
	}
	printf("|-----------------------------------------------------|\n");
}

static void dump_driver_info(int idx, struct rte_driver *driver)
{
	printf("PMD %d Information:\n", idx);
	printf("Driver Name: %s\n", driver->name);

	switch (driver->type) {
	case PMD_VDEV:
		printf("Driver Type: Virtual\n");
		break;
	case PMD_PDEV:
		printf("Driver Type: PCI\n");
		dump_pci_table(driver);
		break;
	default:
		printf("Driver Type: UNKNOWN (%d)\n", driver->type);
		break;
	}
}

int main(int argc, char **argv)
{
	void *pmd;
	struct rte_driver *driver;
	int i = 0;
	int rc = 0;
	const char *basename = "this_pmd_driver";
	char symname[512];

	if (argc <= 1) {
		printf("You must specify a pmd to load\n");
		rc = 127;
		goto out;
	}

	pmd = dlopen(argv[1], RTLD_LAZY);


	if (!pmd) {
		printf("Unalbe to open dlopen library: %s\n", dlerror());
		rc = 128;
		goto out;
	}

	do {
		memset(symname, 0, 512);
		snprintf(symname, 512, "%s%d", basename, i);
		driver = dlsym(pmd, symname);
		
		if (!driver) {
			char *err = dlerror();
			if (err && !i) {
				printf("%s\n", err);
				rc = 129;
			}
			goto out_close;
		}
		dump_driver_info(i, driver);
		i++;
	} while(driver);

out_close:
	dlclose(pmd);
out:
	exit(rc);
}



