#ifndef __TAP_BPF_H__
#define __TAP_BPF_H__

#include <tap_autoconf.h>

#ifdef HAVE_BPF_PROG_LOAD
#include <linux/bpf.h>
#else
/* BPF_MAP_UPDATE_ELEM command flags */
#define	BPF_ANY	0 /* create a new element or update an existing */

/* BPF architecture instruction struct */
struct bpf_insn {
	__u8	code;
	__u8	dst_reg:4;
	__u8	src_reg:4;
	__s16	off;
	__s32	imm; /* immediate value */
};

/* BPF program types */
enum bpf_prog_type {
	BPF_PROG_TYPE_UNSPEC,
	BPF_PROG_TYPE_SOCKET_FILTER,
	BPF_PROG_TYPE_KPROBE,
	BPF_PROG_TYPE_SCHED_CLS,
	BPF_PROG_TYPE_SCHED_ACT,
};

/* BPF commands types */
enum bpf_cmd {
	BPF_MAP_CREATE,
	BPF_MAP_LOOKUP_ELEM,
	BPF_MAP_UPDATE_ELEM,
	BPF_MAP_DELETE_ELEM,
	BPF_MAP_GET_NEXT_KEY,
	BPF_PROG_LOAD,
};

/* BPF maps types */
enum bpf_map_type {
	BPF_MAP_TYPE_UNSPEC,
	BPF_MAP_TYPE_HASH,
};

/* union of anonymous structs used with TAP BPF commands */
union bpf_attr {
	/* BPF_MAP_CREATE command */
	struct {
		__u32	map_type;
		__u32	key_size;
		__u32	value_size;
		__u32	max_entries;
		__u32	map_flags;
		__u32	inner_map_fd;
	};

	/* BPF_MAP_UPDATE_ELEM, BPF_MAP_DELETE_ELEM commands */
	struct {
		__u32		map_fd;
		__aligned_u64	key;
		union {
			__aligned_u64 value;
			__aligned_u64 next_key;
		};
		__u64		flags;
	};

	/* BPF_PROG_LOAD command */
	struct {
		__u32		prog_type;
		__u32		insn_cnt;
		__aligned_u64	insns;
		__aligned_u64	license;
		__u32		log_level;
		__u32		log_size;
		__aligned_u64	log_buf;
		__u32		kern_version;
		__u32		prog_flags;
	};
} __attribute__((aligned(8)));
#endif

enum {
	BPF_MAP_ID_KEY,
	BPF_MAP_ID_SIMPLE,
};

static int bpf_load(enum bpf_prog_type type, const struct bpf_insn *insns,
		size_t insns_cnt, const char *license);

#endif /* __TAP_BPF_H__ */
