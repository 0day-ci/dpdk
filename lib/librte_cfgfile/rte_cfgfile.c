/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
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
 *     * Neither the name of Intel Corporation nor the names of its
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <rte_common.h>

#include "rte_cfgfile.h"

struct rte_cfgfile_section {
	char name[CFG_NAME_LEN];
	int num_entries;
	int free_entries;
	int allocated_entries;
	struct rte_cfgfile_entry **entries;
};

struct rte_cfgfile {
	int flags;
	int num_sections;
	int free_sections;
	int allocated_sections;
	struct rte_cfgfile_section **sections;
};

/** when we resize a file structure, how many extra entries
 * for new sections do we add in */
#define CFG_ALLOC_SECTION_BATCH 8
/** when we resize a section structure, how many extra entries
 * for new entries do we add in */
#define CFG_ALLOC_ENTRY_BATCH 16

/**
 * Default cfgfile load parameters.
 */
static const struct rte_cfgfile_parameters default_cfgfile_params = {
	.comment_character = CFG_DEFAULT_COMMENT_CHARACTER,
};

/**
 * Defines the list of acceptable comment characters supported by this
 * library.
 */
static const char valid_comment_chars[] = {
	'!',
	'#',
	'%',
	';',
	'@'
};

static unsigned
_strip(char *str, unsigned len)
{
	int newlen = len;
	if (len == 0)
		return 0;

	if (isspace(str[len-1])) {
		/* strip trailing whitespace */
		while (newlen > 0 && isspace(str[newlen - 1]))
			str[--newlen] = '\0';
	}

	if (isspace(str[0])) {
		/* strip leading whitespace */
		int i, start = 1;
		while (isspace(str[start]) && start < newlen)
			start++
			; /* do nothing */
		newlen -= start;
		for (i = 0; i < newlen; i++)
			str[i] = str[i+start];
		str[i] = '\0';
	}
	return newlen;
}

static struct rte_cfgfile_section *
_get_section(struct rte_cfgfile *cfg, const char *sectionname)
{
	int i;

	for (i = 0; i < cfg->num_sections; i++) {
		if (strncmp(cfg->sections[i]->name, sectionname,
				sizeof(cfg->sections[0]->name)) == 0)
			return cfg->sections[i];
	}
	return NULL;
}

static int
_add_entry(struct rte_cfgfile_section *section, const char *entryname,
		const char *entryvalue)
{
	int i;

	/* resize entry structure if we don't have room for more entries */
	if (section->free_entries == 0) {

		struct rte_cfgfile_entry **n_entries =
				realloc(section->entries,
				sizeof(section->entries[0]) *
				((section->allocated_entries) +
						CFG_ALLOC_ENTRY_BATCH));

		if (n_entries == NULL)
			return -ENOMEM;

		section->entries = n_entries;

		for (i = section->allocated_entries;
				i < (section->allocated_entries) +
						CFG_ALLOC_ENTRY_BATCH; i++) {
			section->entries[i] =
				malloc(sizeof(struct rte_cfgfile_entry));

			if (section->entries[i] == NULL)
				return -ENOMEM;
		}
		section->allocated_entries += CFG_ALLOC_ENTRY_BATCH;
		section->free_entries += CFG_ALLOC_ENTRY_BATCH;
	}

	/* fill up entry fields with key name and value */
	struct rte_cfgfile_entry *curr_entry =
			section->entries[section->num_entries];

	snprintf(curr_entry->name, sizeof(curr_entry->name), "%s", entryname);
	snprintf(curr_entry->value, sizeof(curr_entry->value), "%s",
								entryvalue);
	section->num_entries++;
	section->free_entries--;

	return 0;
}

static int
rte_cfgfile_check_params(const struct rte_cfgfile_parameters *params)
{
	unsigned int valid_comment;
	unsigned int i;

	if (!params) {
		printf("Error - missing cfgfile parameters\n");
		return -EINVAL;
	}

	valid_comment = 0;
	for (i = 0; i < RTE_DIM(valid_comment_chars); i++) {
		if (params->comment_character == valid_comment_chars[i]) {
			valid_comment = 1;
			break;
		}
	}

	if (valid_comment == 0)	{
		printf("Error - invalid comment characters %c\n",
		       params->comment_character);
		return -ENOTSUP;
	}

	return 0;
}

struct rte_cfgfile *
rte_cfgfile_load(const char *filename, int flags)
{
	return rte_cfgfile_load_with_params(filename, flags,
					    &default_cfgfile_params);
}

struct rte_cfgfile *
rte_cfgfile_load_with_params(const char *filename, int flags,
			     const struct rte_cfgfile_parameters *params)
{
	int allocated_sections = CFG_ALLOC_SECTION_BATCH;
	int allocated_entries = 0;
	int curr_section = -1;
	int curr_entry = -1;
	char buffer[CFG_NAME_LEN + CFG_VALUE_LEN + 4] = {0};
	int lineno = 0;
	struct rte_cfgfile *cfg = NULL;

	if (rte_cfgfile_check_params(params))
		return NULL;

	FILE *f = fopen(filename, "r");
	if (f == NULL)
		return NULL;

	cfg = malloc(sizeof(*cfg) + sizeof(cfg->sections[0]) *
		allocated_sections);
	if (cfg == NULL)
		goto error2;

	memset(cfg->sections, 0, sizeof(cfg->sections[0]) * allocated_sections);

	if (flags & CFG_FLAG_GLOBAL_SECTION) {
		curr_section = 0;
		allocated_entries = CFG_ALLOC_ENTRY_BATCH;
		cfg->sections[curr_section] = malloc(
			sizeof(*cfg->sections[0]) +
			sizeof(cfg->sections[0]->entries[0]) *
			allocated_entries);
		if (cfg->sections[curr_section] == NULL) {
			printf("Error - no memory for global section\n");
			goto error1;
		}

		snprintf(cfg->sections[curr_section]->name,
				 sizeof(cfg->sections[0]->name), "GLOBAL");
	}

	while (fgets(buffer, sizeof(buffer), f) != NULL) {
		char *pos = NULL;
		size_t len = strnlen(buffer, sizeof(buffer));
		lineno++;
		if ((len >= sizeof(buffer) - 1) && (buffer[len-1] != '\n')) {
			printf("Error line %d - no \\n found on string. "
					"Check if line too long\n", lineno);
			goto error1;
		}
		pos = memchr(buffer, params->comment_character, len);
		if (pos != NULL) {
			*pos = '\0';
			len = pos -  buffer;
		}

		len = _strip(buffer, len);
		if (buffer[0] != '[' && memchr(buffer, '=', len) == NULL)
			continue;

		if (buffer[0] == '[') {
			/* section heading line */
			char *end = memchr(buffer, ']', len);
			if (end == NULL) {
				printf("Error line %d - no terminating '['"
					"character found\n", lineno);
				goto error1;
			}
			*end = '\0';
			_strip(&buffer[1], end - &buffer[1]);

			/* close off old section and add start new one */
			if (curr_section >= 0)
				cfg->sections[curr_section]->num_entries =
					curr_entry + 1;
			curr_section++;

			/* resize overall struct if we don't have room for more
			sections */
			if (curr_section == allocated_sections) {
				allocated_sections += CFG_ALLOC_SECTION_BATCH;
				struct rte_cfgfile *n_cfg = realloc(cfg,
					sizeof(*cfg) + sizeof(cfg->sections[0])
					* allocated_sections);
				if (n_cfg == NULL) {
					curr_section--;
					printf("Error - no more memory\n");
					goto error1;
				}
				cfg = n_cfg;
			}

			/* allocate space for new section */
			allocated_entries = CFG_ALLOC_ENTRY_BATCH;
			curr_entry = -1;
			cfg->sections[curr_section] = malloc(
				sizeof(*cfg->sections[0]) +
				sizeof(cfg->sections[0]->entries[0]) *
				allocated_entries);
			if (cfg->sections[curr_section] == NULL) {
				printf("Error - no more memory\n");
				goto error1;
			}

			snprintf(cfg->sections[curr_section]->name,
					sizeof(cfg->sections[0]->name),
					"%s", &buffer[1]);
		} else {
			/* value line */
			if (curr_section < 0) {
				printf("Error line %d - value outside of"
					"section\n", lineno);
				goto error1;
			}

			struct rte_cfgfile_section *sect =
				cfg->sections[curr_section];

			char *split[2] = {NULL};
			split[0] = buffer;
			split[1] = memchr(buffer, '=', len);

			/* when delimeter not found */
			if (split[1] == NULL) {
				printf("Error at line %d - cannot "
					"split string\n", lineno);
				goto error1;
			} else {
				/* when delimeter found */
				*split[1] = '\0';
				split[1]++;

				if (!(flags & CFG_FLAG_EMPTY_VALUES) &&
						(*split[1] == '\0')) {
					printf("Error at line %d - cannot "
						"split string\n", lineno);
					goto error1;
				}
			}

			curr_entry++;
			if (curr_entry == allocated_entries) {
				allocated_entries += CFG_ALLOC_ENTRY_BATCH;
				struct rte_cfgfile_section *n_sect = realloc(
					sect, sizeof(*sect) +
					sizeof(sect->entries[0]) *
					allocated_entries);
				if (n_sect == NULL) {
					curr_entry--;
					printf("Error - no more memory\n");
					goto error1;
				}
				sect = cfg->sections[curr_section] = n_sect;
			}

			sect->entries[curr_entry] = malloc(
				sizeof(*sect->entries[0]));
			if (sect->entries[curr_entry] == NULL) {
				printf("Error - no more memory\n");
				goto error1;
			}

			struct rte_cfgfile_entry *entry = sect->entries[
				curr_entry];
			snprintf(entry->name, sizeof(entry->name), "%s",
				split[0]);
			snprintf(entry->value, sizeof(entry->value), "%s",
				 split[1] ? split[1] : "");
			_strip(entry->name, strnlen(entry->name,
				sizeof(entry->name)));
			_strip(entry->value, strnlen(entry->value,
				sizeof(entry->value)));
		}
	}
	fclose(f);
	cfg->flags = flags;
	cfg->num_sections = curr_section + 1;
	/* curr_section will still be -1 if we have an empty file */
	if (curr_section >= 0)
		cfg->sections[curr_section]->num_entries = curr_entry + 1;
	return cfg;

error1:
	cfg->num_sections = curr_section + 1;
	if (curr_section >= 0)
		cfg->sections[curr_section]->num_entries = curr_entry + 1;
	rte_cfgfile_close(cfg);
error2:
	fclose(f);
	return NULL;
}

struct rte_cfgfile *
rte_cfgfile_create(int flags)
{
	int i, j;
	struct rte_cfgfile *cfg = NULL;

	cfg = malloc(sizeof(*cfg));
	if (cfg == NULL)
		return NULL;

	memset(cfg, 0, sizeof((*cfg)));

	cfg->flags = flags;

	/* allocate first batch of sections and entries */
	cfg->sections = malloc(sizeof(cfg->sections[0]) *
						CFG_ALLOC_SECTION_BATCH);
	if (cfg->sections == NULL)
		return NULL;

	for (i = 0; i < CFG_ALLOC_SECTION_BATCH; i++) {
		cfg->sections[i] = malloc(sizeof(struct rte_cfgfile_section));
		if (cfg->sections[i] == NULL)
			return NULL;

		memset(cfg->sections[i], 0,
					sizeof(struct rte_cfgfile_section));

		cfg->sections[i]->entries =
				malloc(sizeof(cfg->sections[i]->entries[0])
						* CFG_ALLOC_ENTRY_BATCH);
		if (cfg->sections[i]->entries == NULL)
			return NULL;

		for (j = 0; j < CFG_ALLOC_ENTRY_BATCH; j++) {
			cfg->sections[i]->entries[j] = malloc(sizeof(struct
							rte_cfgfile_entry));
			if (cfg->sections[i]->entries[j] == NULL)
				return NULL;
		}
		cfg->sections[i]->allocated_entries = CFG_ALLOC_ENTRY_BATCH;
		cfg->sections[i]->free_entries = CFG_ALLOC_ENTRY_BATCH;
	}
	cfg->allocated_sections = CFG_ALLOC_SECTION_BATCH;
	cfg->free_sections = CFG_ALLOC_SECTION_BATCH;

	if (flags & CFG_FLAG_GLOBAL_SECTION)
		rte_cfgfile_add_section(cfg, "GLOBAL");
	return cfg;
}

int
rte_cfgfile_add_section(struct rte_cfgfile *cfg, const char *sectionname)
{
	int i;
	/* resize overall struct if we don't have room for more	sections */
	if (cfg->free_sections == 0) {

		struct rte_cfgfile_section **n_sections =
				realloc(cfg->sections,
				sizeof(cfg->sections[0]) *
				((cfg->allocated_sections) +
						CFG_ALLOC_SECTION_BATCH));

		if (n_sections == NULL)
			return -ENOMEM;

		cfg->sections = n_sections;

		for (i = cfg->allocated_sections;
				i < (cfg->allocated_sections) +
						CFG_ALLOC_SECTION_BATCH; i++) {
			cfg->sections[i] =
				malloc(sizeof(struct rte_cfgfile_section));

			if (cfg->sections[i] == NULL)
				return -ENOMEM;

			memset(cfg->sections[i], 0,
					sizeof(struct rte_cfgfile_section));
		}
		cfg->allocated_sections += CFG_ALLOC_SECTION_BATCH;
		cfg->free_sections += CFG_ALLOC_SECTION_BATCH;
	}

	snprintf(cfg->sections[cfg->num_sections]->name,
			sizeof(cfg->sections[0]->name), "%s", sectionname);
	cfg->sections[cfg->num_sections]->num_entries = 0;

	cfg->num_sections++;
	cfg->free_sections--;

	return 0;
}

int rte_cfgfile_add_entry(struct rte_cfgfile *cfg,
		const char *sectionname, const char *entryname,
		const char *entryvalue)
{
	int ret;

	if (cfg == NULL)
		return -EINVAL;

	if (rte_cfgfile_has_entry(cfg, sectionname, entryname) != 0)
		return -EEXIST;

	/* search for section pointer by sectionname */
	struct rte_cfgfile_section *curr_section = _get_section(cfg,
								sectionname);
	if (curr_section == NULL)
		return -EINVAL;

	ret = _add_entry(curr_section, entryname, entryvalue);

	return ret;
}

int rte_cfgfile_set_entry(struct rte_cfgfile *cfg, const char *sectionname,
		const char *entryname, const char *entryvalue)
{
	int i;

	if (cfg == NULL)
		return -EINVAL;

	/* search for section pointer by sectionname */
	struct rte_cfgfile_section *curr_section = _get_section(cfg,
								sectionname);
	if (curr_section == NULL)
		return -EINVAL;

	if (entryvalue == NULL)
		entryvalue = "";

	for (i = 0; i < curr_section->num_entries; i++)
		if (!strcmp(curr_section->entries[i]->name, entryname)) {
			strcpy(curr_section->entries[i]->value, entryvalue);
			return 0;
		}
	return -1;
}

int rte_cfgfile_save(struct rte_cfgfile *cfg, const char *filename)
{
	char buffer[CFG_NAME_LEN + CFG_VALUE_LEN + 4] = {0};
	int i, j;

	if ((cfg == NULL) || (filename == NULL))
		return -EINVAL;

	FILE *f = fopen(filename, "w");

	if (f == NULL)
		return -EINVAL;

	for (i = 0; i < cfg->num_sections; i++) {
		snprintf(buffer, sizeof(buffer), "[%s]\n",
					cfg->sections[i]->name);
		fputs(buffer, f);

		for (j = 0; j < cfg->sections[i]->num_entries; j++) {
			snprintf(buffer, sizeof(buffer), "%s=%s\n",
				cfg->sections[i]->entries[j]->name,
				cfg->sections[i]->entries[j]->value);
			fputs(buffer, f);
		}
	}
	return fclose(f);
}

int rte_cfgfile_close(struct rte_cfgfile *cfg)
{
	int i, j;

	if (cfg == NULL)
		return -1;

	for (i = 0; i < cfg->num_sections; i++) {
		if (cfg->sections[i] != NULL) {
			if (cfg->sections[i]->num_entries) {
				for (j = 0; j < cfg->sections[i]->num_entries;
					j++) {
					if (cfg->sections[i]->entries[j] !=
						NULL)
						free(cfg->sections[i]->
							entries[j]);
				}
			}
			free(cfg->sections[i]);
		}
	}
	free(cfg);

	return 0;
}

int
rte_cfgfile_num_sections(struct rte_cfgfile *cfg, const char *sectionname,
size_t length)
{
	int i;
	int num_sections = 0;
	for (i = 0; i < cfg->num_sections; i++) {
		if (strncmp(cfg->sections[i]->name, sectionname, length) == 0)
			num_sections++;
	}
	return num_sections;
}

int
rte_cfgfile_sections(struct rte_cfgfile *cfg, char *sections[],
	int max_sections)
{
	int i;

	for (i = 0; i < cfg->num_sections && i < max_sections; i++)
		snprintf(sections[i], CFG_NAME_LEN, "%s",
		cfg->sections[i]->name);

	return i;
}


int
rte_cfgfile_has_section(struct rte_cfgfile *cfg, const char *sectionname)
{
	return _get_section(cfg, sectionname) != NULL;
}

int
rte_cfgfile_section_num_entries(struct rte_cfgfile *cfg,
	const char *sectionname)
{
	const struct rte_cfgfile_section *s = _get_section(cfg, sectionname);
	if (s == NULL)
		return -1;
	return s->num_entries;
}


int
rte_cfgfile_section_entries(struct rte_cfgfile *cfg, const char *sectionname,
		struct rte_cfgfile_entry *entries, int max_entries)
{
	int i;
	const struct rte_cfgfile_section *sect = _get_section(cfg, sectionname);
	if (sect == NULL)
		return -1;
	for (i = 0; i < max_entries && i < sect->num_entries; i++)
		entries[i] = *sect->entries[i];
	return i;
}

int
rte_cfgfile_section_entries_by_index(struct rte_cfgfile *cfg, int index,
		char *sectionname,
		struct rte_cfgfile_entry *entries, int max_entries)
{
	int i;
	const struct rte_cfgfile_section *sect;

	if (index < 0 || index >= cfg->num_sections)
		return -1;

	sect = cfg->sections[index];
	snprintf(sectionname, CFG_NAME_LEN, "%s", sect->name);
	for (i = 0; i < max_entries && i < sect->num_entries; i++)
		entries[i] = *sect->entries[i];
	return i;
}

const char *
rte_cfgfile_get_entry(struct rte_cfgfile *cfg, const char *sectionname,
		const char *entryname)
{
	int i;
	const struct rte_cfgfile_section *sect = _get_section(cfg, sectionname);
	if (sect == NULL)
		return NULL;
	for (i = 0; i < sect->num_entries; i++)
		if (strncmp(sect->entries[i]->name, entryname, CFG_NAME_LEN)
			== 0)
			return sect->entries[i]->value;
	return NULL;
}

int
rte_cfgfile_has_entry(struct rte_cfgfile *cfg, const char *sectionname,
		const char *entryname)
{
	return rte_cfgfile_get_entry(cfg, sectionname, entryname) != NULL;
}
