#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <elf.h>


/* On BSD-alike OSes elf.h defines these according to host's word size */
#undef ELF_ST_BIND
#undef ELF_ST_TYPE
#undef ELF_R_SYM
#undef ELF_R_TYPE

#define Elf_Ehdr    Elf64_Ehdr
#define Elf_Shdr    Elf64_Shdr
#define Elf_Sym     Elf64_Sym
#define Elf_Addr    Elf64_Addr
#define Elf_Sword   Elf64_Sxword
#define Elf_Section Elf64_Half
#define ELF_ST_BIND ELF64_ST_BIND
#define ELF_ST_TYPE ELF64_ST_TYPE

#define Elf_Rel     Elf64_Rel
#define Elf_Rela    Elf64_Rela
#define ELF_R_SYM   ELF64_R_SYM
#define ELF_R_TYPE  ELF64_R_TYPE

#define TO_NATIVE(x) (x)


struct rte_pci_id {
	uint16_t vendor_id;           /**< Vendor ID or PCI_ANY_ID. */
	uint16_t device_id;           /**< Device ID or PCI_ANY_ID. */
	uint16_t subsystem_vendor_id; /**< Subsystem vendor ID or PCI_ANY_ID. */
	uint16_t subsystem_device_id; /**< Subsystem device ID or PCI_ANY_ID. */
};

enum opt_params {
	PMD_PARAM_STRING = 0,
	PMD_OPT_MAX
};

struct pmd_driver {
	Elf_Sym *name_sym;
	const char *name;
	struct rte_pci_id *pci_tbl;
	struct pmd_driver *next;

	const char* opt_vals[PMD_OPT_MAX];
};

struct elf_info {
	unsigned long size;
	Elf_Ehdr     *hdr;
	Elf_Shdr     *sechdrs;
	Elf_Sym      *symtab_start;
	Elf_Sym      *symtab_stop;
	Elf_Section  export_sec;
	Elf_Section  export_unused_sec;
	Elf_Section  export_gpl_sec;
	Elf_Section  export_unused_gpl_sec;
	Elf_Section  export_gpl_future_sec;
	char         *strtab;
	char	     *modinfo;
	unsigned int modinfo_len;

	/* support for 32bit section numbers */

	unsigned int num_sections; /* max_secindex + 1 */
	unsigned int secindex_strings;
	/* if Nth symbol table entry has .st_shndx = SHN_XINDEX,
	 * take shndx from symtab_shndx_start[N] instead */
	Elf32_Word   *symtab_shndx_start;
	Elf32_Word   *symtab_shndx_stop;

	struct pmd_driver *drivers;
};

