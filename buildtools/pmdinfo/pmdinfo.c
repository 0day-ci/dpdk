/* Postprocess pmd object files to export hw support 
 *
 * Copyright 2016 Neil Horman <nhorman@tuxdriver.com>
 * Based in part on modpost.c from the linux kernel
 *
 * This software may be used and distributed according to the terms
 * of the GNU General Public License V2, incorporated herein by reference.
 *
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include <errno.h>
#include "pmdinfo.h"


static const char *sym_name(struct elf_info *elf, Elf_Sym *sym)
{
	if (sym)
		return elf->strtab + sym->st_name;
	else
		return "(unknown)";
}

void *grab_file(const char *filename, unsigned long *size)
{
	struct stat st;
	void *map = MAP_FAILED;
	int fd;

	fd = open(filename, O_RDONLY);
	if (fd < 0)
		return NULL;
	if (fstat(fd, &st))
		goto failed;

	*size = st.st_size;
	map = mmap(NULL, *size, PROT_READ|PROT_WRITE, MAP_PRIVATE, fd, 0);

failed:
	close(fd);
	if (map == MAP_FAILED)
		return NULL;
	return map;
}

/**
  * Return a copy of the next line in a mmap'ed file.
  * spaces in the beginning of the line is trimmed away.
  * Return a pointer to a static buffer.
  **/
char *get_next_line(unsigned long *pos, void *file, unsigned long size)
{
	static char line[4096];
	int skip = 1;
	size_t len = 0;
	signed char *p = (signed char *)file + *pos;
	char *s = line;

	for (; *pos < size ; (*pos)++) {
		if (skip && isspace(*p)) {
			p++;
			continue;
		}
		skip = 0;
		if (*p != '\n' && (*pos < size)) {
			len++;
			*s++ = *p++;
			if (len > 4095)
				break; /* Too long, stop */
		} else {
			/* End of string */
			*s = '\0';
			return line;
		}
	}
	/* End of buffer */
	return NULL;
}

void release_file(void *file, unsigned long size)
{
	munmap(file, size);
}


static void *get_sym_value(struct elf_info *info, const Elf_Sym *sym)
{
	void *ptr = (void *)info->hdr + info->sechdrs[sym->st_shndx].sh_offset;

	return (void *)(ptr + sym->st_value);
}

static Elf_Sym *find_sym_in_symtab(struct elf_info *info, 
				   const char *name, Elf_Sym *last)
{
	Elf_Sym *idx;
	if (last)
		idx = last+1;
	else
		idx = info->symtab_start;

	for(; idx < info->symtab_stop; idx++) {
		const char *n = sym_name(info, idx);
		if (!strncmp(n, name, strlen(name)))
			return idx;
	}
	return NULL;
}

static int parse_elf(struct elf_info *info, const char *filename)
{
	unsigned int i;
	Elf_Ehdr *hdr;
	Elf_Shdr *sechdrs;
	Elf_Sym  *sym;
	const char *secstrings;
	unsigned int symtab_idx = ~0U, symtab_shndx_idx = ~0U;

	hdr = grab_file(filename, &info->size);
	if (!hdr) {
		perror(filename);
		exit(1);
	}
	info->hdr = hdr;
	if (info->size < sizeof(*hdr)) {
		/* file too small, assume this is an empty .o file */
		return 0;
	}
	/* Is this a valid ELF file? */
	if ((hdr->e_ident[EI_MAG0] != ELFMAG0) ||
	    (hdr->e_ident[EI_MAG1] != ELFMAG1) ||
	    (hdr->e_ident[EI_MAG2] != ELFMAG2) ||
	    (hdr->e_ident[EI_MAG3] != ELFMAG3)) {
		/* Not an ELF file - silently ignore it */
		return 0;
	}
	/* Fix endianness in ELF header */
	hdr->e_type      = TO_NATIVE(hdr->e_type);
	hdr->e_machine   = TO_NATIVE(hdr->e_machine);
	hdr->e_version   = TO_NATIVE(hdr->e_version);
	hdr->e_entry     = TO_NATIVE(hdr->e_entry);
	hdr->e_phoff     = TO_NATIVE(hdr->e_phoff);
	hdr->e_shoff     = TO_NATIVE(hdr->e_shoff);
	hdr->e_flags     = TO_NATIVE(hdr->e_flags);
	hdr->e_ehsize    = TO_NATIVE(hdr->e_ehsize);
	hdr->e_phentsize = TO_NATIVE(hdr->e_phentsize);
	hdr->e_phnum     = TO_NATIVE(hdr->e_phnum);
	hdr->e_shentsize = TO_NATIVE(hdr->e_shentsize);
	hdr->e_shnum     = TO_NATIVE(hdr->e_shnum);
	hdr->e_shstrndx  = TO_NATIVE(hdr->e_shstrndx);
	sechdrs = (void *)hdr + hdr->e_shoff;
	info->sechdrs = sechdrs;

	/* Check if file offset is correct */
	if (hdr->e_shoff > info->size) {
		fprintf(stderr, "section header offset=%lu in file '%s' is bigger than "
		      "filesize=%lu\n", (unsigned long)hdr->e_shoff,
		      filename, info->size);
		return 0;
	}

	if (hdr->e_shnum == SHN_UNDEF) {
		/*
		 * There are more than 64k sections,
		 * read count from .sh_size.
		 */
		info->num_sections = TO_NATIVE(sechdrs[0].sh_size);
	}
	else {
		info->num_sections = hdr->e_shnum;
	}
	if (hdr->e_shstrndx == SHN_XINDEX) {
		info->secindex_strings = TO_NATIVE(sechdrs[0].sh_link);
	}
	else {
		info->secindex_strings = hdr->e_shstrndx;
	}

	/* Fix endianness in section headers */
	for (i = 0; i < info->num_sections; i++) {
		sechdrs[i].sh_name      = TO_NATIVE(sechdrs[i].sh_name);
		sechdrs[i].sh_type      = TO_NATIVE(sechdrs[i].sh_type);
		sechdrs[i].sh_flags     = TO_NATIVE(sechdrs[i].sh_flags);
		sechdrs[i].sh_addr      = TO_NATIVE(sechdrs[i].sh_addr);
		sechdrs[i].sh_offset    = TO_NATIVE(sechdrs[i].sh_offset);
		sechdrs[i].sh_size      = TO_NATIVE(sechdrs[i].sh_size);
		sechdrs[i].sh_link      = TO_NATIVE(sechdrs[i].sh_link);
		sechdrs[i].sh_info      = TO_NATIVE(sechdrs[i].sh_info);
		sechdrs[i].sh_addralign = TO_NATIVE(sechdrs[i].sh_addralign);
		sechdrs[i].sh_entsize   = TO_NATIVE(sechdrs[i].sh_entsize);
	}
	/* Find symbol table. */
	secstrings = (void *)hdr + sechdrs[info->secindex_strings].sh_offset;
	for (i = 1; i < info->num_sections; i++) {
		const char *secname;
		int nobits = sechdrs[i].sh_type == SHT_NOBITS;

		if (!nobits && sechdrs[i].sh_offset > info->size) {
			fprintf(stderr, "%s is truncated. sechdrs[i].sh_offset=%lu > "
			      "sizeof(*hrd)=%zu\n", filename,
			      (unsigned long)sechdrs[i].sh_offset,
			      sizeof(*hdr));
			return 0;
		}
		secname = secstrings + sechdrs[i].sh_name;
		if (strcmp(secname, ".modinfo") == 0) {
			if (nobits)
				fprintf(stderr, "%s has NOBITS .modinfo\n", filename);
			info->modinfo = (void *)hdr + sechdrs[i].sh_offset;
			info->modinfo_len = sechdrs[i].sh_size;
		} else if (strcmp(secname, "__ksymtab") == 0)
			info->export_sec = i;
		else if (strcmp(secname, "__ksymtab_unused") == 0)
			info->export_unused_sec = i;
		else if (strcmp(secname, "__ksymtab_gpl") == 0)
			info->export_gpl_sec = i;
		else if (strcmp(secname, "__ksymtab_unused_gpl") == 0)
			info->export_unused_gpl_sec = i;
		else if (strcmp(secname, "__ksymtab_gpl_future") == 0)
			info->export_gpl_future_sec = i;

		if (sechdrs[i].sh_type == SHT_SYMTAB) {
			unsigned int sh_link_idx;
			symtab_idx = i;
			info->symtab_start = (void *)hdr +
			    sechdrs[i].sh_offset;
			info->symtab_stop  = (void *)hdr +
			    sechdrs[i].sh_offset + sechdrs[i].sh_size;
			sh_link_idx = sechdrs[i].sh_link;
			info->strtab       = (void *)hdr +
			    sechdrs[sh_link_idx].sh_offset;
		}

		/* 32bit section no. table? ("more than 64k sections") */
		if (sechdrs[i].sh_type == SHT_SYMTAB_SHNDX) {
			symtab_shndx_idx = i;
			info->symtab_shndx_start = (void *)hdr +
			    sechdrs[i].sh_offset;
			info->symtab_shndx_stop  = (void *)hdr +
			    sechdrs[i].sh_offset + sechdrs[i].sh_size;
		}
	}
	if (!info->symtab_start)
		fprintf(stderr, "%s has no symtab?\n", filename);

	/* Fix endianness in symbols */
	for (sym = info->symtab_start; sym < info->symtab_stop; sym++) {
		sym->st_shndx = TO_NATIVE(sym->st_shndx);
		sym->st_name  = TO_NATIVE(sym->st_name);
		sym->st_value = TO_NATIVE(sym->st_value);
		sym->st_size  = TO_NATIVE(sym->st_size);
	}

	if (symtab_shndx_idx != ~0U) {
		Elf32_Word *p;
		if (symtab_idx != sechdrs[symtab_shndx_idx].sh_link)
			fprintf(stderr, "%s: SYMTAB_SHNDX has bad sh_link: %u!=%u\n",
			      filename, sechdrs[symtab_shndx_idx].sh_link,
			      symtab_idx);
		/* Fix endianness */
		for (p = info->symtab_shndx_start; p < info->symtab_shndx_stop;
		     p++)
			*p = TO_NATIVE(*p);
	}

	return 1;
}

static void parse_elf_finish(struct elf_info *info)
{
	struct pmd_driver *tmp, *idx = info->drivers;
	release_file(info->hdr, info->size);
	while (idx) {
		tmp = idx->next;
		free(idx);
		idx = tmp;
	}
}

static const char *sec_name(struct elf_info *elf, int secindex)
{
	Elf_Shdr *sechdrs = elf->sechdrs;
	return (void *)elf->hdr +
		elf->sechdrs[elf->secindex_strings].sh_offset +
		sechdrs[secindex].sh_name;
}

static int get_symbol_index(struct elf_info *info, Elf64_Sym *sym)
{
	const char *name =  sym_name(info, sym);
	const char *idx;

	idx = name;
	while (idx) {
		if (isdigit(*idx))
			return atoi(idx);
		idx++;
	}
	return -1;
}

static int complete_pmd_entry(struct elf_info *info, struct pmd_driver *drv)
{
	const char *tname;
	int i = get_symbol_index(info, drv->name_sym);
	char drvsym[128];

	if (i == -1)
		return -ENOENT;

	drv->name = get_sym_value(info, drv->name_sym);

	sprintf(drvsym, "this_pmd_driver%d", i);

	drv->driver = find_sym_in_symtab(info, drvsym, NULL);

	/*
 	 * If this returns NULL, then this is a PMD_VDEV, because
 	 * it has no pci table reference
 	 */
	if (!drv->driver) {
		drv->pci_tbl = NULL;
		return 0;
	}

	tname = get_sym_value(info, drv->driver);
	drv->pci_tbl_sym = find_sym_in_symtab(info, tname, NULL);

	if (!drv->pci_tbl_sym)
		return -ENOENT;

	drv->pci_tbl = (struct rte_pci_id *)get_sym_value(info, drv->pci_tbl_sym);
	if (!drv->pci_tbl)
		return -ENOENT;


	return 0;
	
}

static int locate_pmd_entries(struct elf_info *info)
{
	Elf_Sym *last = NULL;
	struct pmd_driver *new;

	info->drivers = NULL;

	do {
		new = malloc(sizeof(struct pmd_driver));
		new->name_sym = find_sym_in_symtab(info, "this_pmd_name", last);
		last = new->name_sym;
		if (!new->name_sym)
			free(new);
		else {
			if (complete_pmd_entry(info, new)) {
				fprintf(stderr, "Failed to complete pmd entry\n");
				free(new);
			} else {
				new->next = info->drivers;
				info->drivers = new;
			}
		}
	} while (last);
}

static void output_pmd_info_string(struct elf_info *info, char *outfile)
{
	FILE *ofd;
	struct pmd_driver *drv;
	struct rte_pci_id *pci_ids;
	int idx = 0;

	ofd = fopen(outfile, "w+");
	if (!ofd) {
		fprintf(stderr, "Unable to open output file\n");
		return;
	}

	drv = info->drivers;

	while (drv) {
		fprintf(ofd, "const char %s_pmd_info[] __attribute__((used)) = \"PMD_INFO_STRING= {",
			drv->name);
		fprintf(ofd,"\\\"name\\\" : \\\"%s\\\", ", drv->name);
		fprintf(ofd,"\\\"type\\\" : \\\"%s\\\", ", drv->pci_tbl ? "PMD_PDEV" : "PMD_VDEV");
 
		pci_ids = drv->pci_tbl;
		fprintf(ofd, "\\\"pci_ids\\\" : [");

		while (pci_ids && pci_ids->device_id) {
			fprintf(ofd, "[%d, %d, %d, %d]",
				pci_ids->vendor_id, pci_ids->device_id,
				pci_ids->subsystem_vendor_id,
				pci_ids->subsystem_device_id);
			pci_ids++;
			if (pci_ids->device_id)
				fprintf(ofd, ",");
			else
				fprintf(ofd, " ");
		}
		fprintf(ofd, "]}\";");
		drv = drv->next;
	}

	fclose(ofd);
}

int main(int argc, char **argv)
{
	struct elf_info info;
	int rc = 1;

	if (argc < 3) {
		fprintf(stderr, "usage: pmdinfo <object file> <c output file>\n");
		exit(127);
	}
	parse_elf(&info, argv[1]);

	locate_pmd_entries(&info);

	if (info.drivers) {
		output_pmd_info_string(&info, argv[2]);
		rc = 0;
	} else {
		fprintf(stderr, "Hmm, Appears to be a driver but no drivers registered\n");
	}

	parse_elf_finish(&info);
	exit(rc);
}
