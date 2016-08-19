/*-
 *   BSD LICENSE
 *
 *   Copyright 2016 6WIND S.A.
 *   Copyright 2016 Mellanox.
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
 *     * Neither the name of 6WIND S.A. nor the names of its
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

#ifndef RTE_FLOW_H_
#define RTE_FLOW_H_

/**
 * @file
 * RTE generic flow API
 *
 * This interface provides the ability to program packet matching and
 * associated actions in hardware through flow rules.
 */

#include <rte_arp.h>
#include <rte_ether.h>
#include <rte_icmp.h>
#include <rte_ip.h>
#include <rte_sctp.h>
#include <rte_tcp.h>
#include <rte_udp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Flow rule attributes.
 *
 * Priorities are set on two levels: per group and per rule within groups.
 *
 * Lower values denote higher priority, the highest priority for both levels
 * is 0, so that a rule with priority 0 in group 8 is always matched after a
 * rule with priority 8 in group 0.
 *
 * Although optional, applications are encouraged to group similar rules as
 * much as possible to fully take advantage of hardware capabilities
 * (e.g. optimized matching) and work around limitations (e.g. a single
 * pattern type possibly allowed in a given group).
 *
 * Group and priority levels are arbitrary and up to the application, they
 * do not need to be contiguous nor start from 0, however the maximum number
 * varies between devices and may be affected by existing flow rules.
 *
 * If a packet is matched by several rules of a given group for a given
 * priority level, the outcome is undefined. It can take any path, may be
 * duplicated or even cause unrecoverable errors.
 *
 * Note that support for more than a single group and priority level is not
 * guaranteed.
 *
 * Flow rules can apply to inbound and/or outbound traffic (ingress/egress).
 *
 * Several pattern items and actions are valid and can be used in both
 * directions. Those valid for only one direction are described as such.
 *
 * Specifying both directions at once is not recommended but may be valid in
 * some cases, such as incrementing the same counter twice.
 *
 * Not specifying any direction is currently an error.
 */
struct rte_flow_attr {
	uint32_t group; /**< Priority group. */
	uint32_t priority; /**< Priority level within group. */
	uint32_t ingress:1; /**< Rule applies to ingress traffic. */
	uint32_t egress:1; /**< Rule applies to egress traffic. */
	uint32_t reserved:30; /**< Reserved, must be zero. */
};

/**
 * Matching pattern item types.
 *
 * Items are arranged in a list to form a matching pattern for packets.
 * They fall in two categories:
 *
 * - Protocol matching (ANY, RAW, ETH, IPV4, IPV6, ICMP, UDP, TCP, SCTP,
 *   VXLAN and so on), usually associated with a specification
 *   structure. These must be stacked in the same order as the protocol
 *   layers to match, starting from L2.
 *
 * - Affecting how the pattern is processed (END, VOID, INVERT, PF, VF, PORT
 *   and so on), often without a specification structure. Since they are
 *   meta data that does not match packet contents, these can be specified
 *   anywhere within item lists without affecting the protocol matching
 *   items.
 *
 * See the description of individual types for more information. Those
 * marked with [META] fall into the second category.
 */
enum rte_flow_item_type {
	/**
	 * [META]
	 *
	 * End marker for item lists. Prevents further processing of items,
	 * thereby ending the pattern.
	 *
	 * No associated specification structure.
	 */
	RTE_FLOW_ITEM_TYPE_END,

	/**
	 * [META]
	 *
	 * Used as a placeholder for convenience. It is ignored and simply
	 * discarded by PMDs.
	 *
	 * No associated specification structure.
	 */
	RTE_FLOW_ITEM_TYPE_VOID,

	/**
	 * [META]
	 *
	 * Inverted matching, i.e. process packets that do not match the
	 * pattern.
	 *
	 * No associated specification structure.
	 */
	RTE_FLOW_ITEM_TYPE_INVERT,

	/**
	 * Matches any protocol in place of the current layer, a single ANY
	 * may also stand for several protocol layers.
	 *
	 * See struct rte_flow_item_any.
	 */
	RTE_FLOW_ITEM_TYPE_ANY,

	/**
	 * [META]
	 *
	 * Matches packets addressed to the physical function of the device.
	 *
	 * If the underlying device function differs from the one that would
	 * normally receive the matched traffic, specifying this item
	 * prevents it from reaching that device unless the flow rule
	 * contains a PF action. Packets are not duplicated between device
	 * instances by default.
	 *
	 * No associated specification structure.
	 */
	RTE_FLOW_ITEM_TYPE_PF,

	/**
	 * [META]
	 *
	 * Matches packets addressed to a virtual function ID of the device.
	 *
	 * If the underlying device function differs from the one that would
	 * normally receive the matched traffic, specifying this item
	 * prevents it from reaching that device unless the flow rule
	 * contains a VF action. Packets are not duplicated between device
	 * instances by default.
	 *
	 * See struct rte_flow_item_vf.
	 */
	RTE_FLOW_ITEM_TYPE_VF,

	/**
	 * [META]
	 *
	 * Matches packets coming from the specified physical port of the
	 * underlying device.
	 *
	 * The first PORT item overrides the physical port normally
	 * associated with the specified DPDK input port (port_id). This
	 * item can be provided several times to match additional physical
	 * ports.
	 *
	 * See struct rte_flow_item_port.
	 */
	RTE_FLOW_ITEM_TYPE_PORT,

	/**
	 * Matches a byte string of a given length at a given offset.
	 *
	 * See struct rte_flow_item_raw.
	 */
	RTE_FLOW_ITEM_TYPE_RAW,

	/**
	 * Matches an Ethernet header.
	 *
	 * See struct rte_flow_item_eth.
	 */
	RTE_FLOW_ITEM_TYPE_ETH,

	/**
	 * Matches an IPv4 header.
	 *
	 * See struct rte_flow_item_ipv4.
	 */
	RTE_FLOW_ITEM_TYPE_IPV4,

	/**
	 * Matches an IPv6 header.
	 *
	 * See struct rte_flow_item_ipv6.
	 */
	RTE_FLOW_ITEM_TYPE_IPV6,

	/**
	 * Matches an ICMP header.
	 *
	 * See struct rte_flow_item_icmp.
	 */
	RTE_FLOW_ITEM_TYPE_ICMP,

	/**
	 * Matches a UDP header.
	 *
	 * See struct rte_flow_item_udp.
	 */
	RTE_FLOW_ITEM_TYPE_UDP,

	/**
	 * Matches a TCP header.
	 *
	 * See struct rte_flow_item_tcp.
	 */
	RTE_FLOW_ITEM_TYPE_TCP,

	/**
	 * Matches a SCTP header.
	 *
	 * See struct rte_flow_item_sctp.
	 */
	RTE_FLOW_ITEM_TYPE_SCTP,

	/**
	 * Matches a VXLAN header.
	 *
	 * See struct rte_flow_item_vxlan.
	 */
	RTE_FLOW_ITEM_TYPE_VXLAN,
};

/**
 * RTE_FLOW_ITEM_TYPE_ANY
 *
 * Matches any protocol in place of the current layer, a single ANY may also
 * stand for several protocol layers.
 *
 * This is usually specified as the first pattern item when looking for a
 * protocol anywhere in a packet.
 *
 * A maximum value of 0 requests matching any number of protocol layers
 * above or equal to the minimum value, a maximum value lower than the
 * minimum one is otherwise invalid.
 *
 * Layer mask is ignored.
 */
struct rte_flow_item_any {
	uint16_t min; /**< Minimum number of layers covered. */
	uint16_t max; /**< Maximum number of layers covered, 0 for infinity. */
};

/**
 * RTE_FLOW_ITEM_TYPE_VF
 *
 * Matches packets addressed to a virtual function ID of the device.
 *
 * If the underlying device function differs from the one that would
 * normally receive the matched traffic, specifying this item prevents it
 * from reaching that device unless the flow rule contains a VF
 * action. Packets are not duplicated between device instances by default.
 *
 * - Likely to return an error or never match any traffic if this causes a
 *   VF device to match traffic addressed to a different VF.
 * - Can be specified multiple times to match traffic addressed to several
 *   specific VFs.
 * - Can be combined with a PF item to match both PF and VF traffic.
 *
 * Layer mask is ignored.
 */
struct rte_flow_item_vf {
	uint32_t any:1; /**< Ignore the specified VF ID. */
	uint32_t reserved:31; /**< Reserved, must be zero. */
	uint32_t vf; /**< Destination VF ID. */
};

/**
 * RTE_FLOW_ITEM_TYPE_PORT
 *
 * Matches packets coming from the specified physical port of the underlying
 * device.
 *
 * The first PORT item overrides the physical port normally associated with
 * the specified DPDK input port (port_id). This item can be provided
 * several times to match additional physical ports.
 *
 * Layer mask is ignored.
 *
 * Note that physical ports are not necessarily tied to DPDK input ports
 * (port_id) when those are not under DPDK control. Possible values are
 * specific to each device, they are not necessarily indexed from zero and
 * may not be contiguous.
 *
 * As a device property, the list of allowed values as well as the value
 * associated with a port_id should be retrieved by other means.
 */
struct rte_flow_item_port {
	uint32_t index; /**< Physical port index. */
};

/**
 * RTE_FLOW_ITEM_TYPE_RAW
 *
 * Matches a byte string of a given length at a given offset.
 *
 * Offset is either absolute (using the start of the packet) or relative to
 * the end of the previous matched item in the stack, in which case negative
 * values are allowed.
 *
 * If search is enabled, offset is used as the starting point. The search
 * area can be delimited by setting limit to a nonzero value, which is the
 * maximum number of bytes after offset where the pattern may start.
 *
 * Matching a zero-length pattern is allowed, doing so resets the relative
 * offset for subsequent items.
 *
 * The mask only affects the pattern field.
 */
struct rte_flow_item_raw {
	uint32_t relative:1; /**< Look for pattern after the previous item. */
	uint32_t search:1; /**< Search pattern from offset (see also limit). */
	uint32_t reserved:30; /**< Reserved, must be set to zero. */
	int32_t offset; /**< Absolute or relative offset for pattern. */
	uint16_t limit; /**< Search area limit for start of pattern. */
	uint16_t length; /**< Pattern length. */
	uint8_t pattern[]; /**< Byte string to look for. */
};

/**
 * RTE_FLOW_ITEM_TYPE_ETH
 *
 * Matches an Ethernet header.
 */
struct rte_flow_item_eth {
	struct ether_addr dst; /**< Destination MAC. */
	struct ether_addr src; /**< Source MAC. */
	unsigned int type; /**< EtherType. */
	unsigned int tags; /**< Number of 802.1Q/ad tags defined. */
	struct {
		uint16_t tpid; /**< Tag protocol identifier. */
		uint16_t tci; /**< Tag control information. */
	} tag[]; /**< 802.1Q/ad tag definitions, outermost first. */
};

/**
 * RTE_FLOW_ITEM_TYPE_IPV4
 *
 * Matches an IPv4 header.
 *
 * Note: IPv4 options are handled by dedicated pattern items.
 */
struct rte_flow_item_ipv4 {
	struct ipv4_hdr hdr; /**< IPv4 header definition. */
};

/**
 * RTE_FLOW_ITEM_TYPE_IPV6.
 *
 * Matches an IPv6 header.
 *
 * Note: IPv6 options are handled by dedicated pattern items.
 */
struct rte_flow_item_ipv6 {
	struct ipv6_hdr hdr; /**< IPv6 header definition. */
};

/**
 * RTE_FLOW_ITEM_TYPE_ICMP.
 *
 * Matches an ICMP header.
 */
struct rte_flow_item_icmp {
	struct icmp_hdr hdr; /**< ICMP header definition. */
};

/**
 * RTE_FLOW_ITEM_TYPE_UDP.
 *
 * Matches a UDP header.
 */
struct rte_flow_item_udp {
	struct udp_hdr hdr; /**< UDP header definition. */
};

/**
 * RTE_FLOW_ITEM_TYPE_TCP.
 *
 * Matches a TCP header.
 */
struct rte_flow_item_tcp {
	struct tcp_hdr hdr; /**< TCP header definition. */
};

/**
 * RTE_FLOW_ITEM_TYPE_SCTP.
 *
 * Matches a SCTP header.
 */
struct rte_flow_item_sctp {
	struct sctp_hdr hdr; /**< SCTP header definition. */
};

/**
 * RTE_FLOW_ITEM_TYPE_VXLAN.
 *
 * Matches a VXLAN header (RFC 7348).
 */
struct rte_flow_item_vxlan {
	uint32_t flags:8; /**< Normally 0x08 (I flag). */
	uint32_t rsvd0:24; /**< Reserved, normally 0x000000. */
	uint32_t vni:24; /**< VXLAN network identifier. */
	uint32_t rsvd1:8; /**< Reserved, normally 0x00. */
};

/**
 * Matching pattern item definition.
 *
 * Except for meta types that do not need one, spec must be a valid pointer
 * to a structure of the related item type. A mask of the same type can be
 * provided to tell which bits in spec are to be matched.
 *
 * A mask is normally only needed for spec fields matching packet data,
 * ignored otherwise. See individual item types for more information.
 *
 * A NULL mask pointer is allowed and is similar to matching with a full
 * mask (all ones) spec fields supported by hardware, the remaining fields
 * are ignored (all zero), there is thus no error checking for unsupported
 * fields.
 */
struct rte_flow_item {
	enum rte_flow_item_type type; /**< Item type. */
	const void *spec; /**< Pointer to item specification structure. */
	const void *mask; /**< Mask for item specification. */
};

/**
 * Matching pattern definition.
 *
 * A pattern is formed by stacking items starting from the lowest protocol
 * layer to match. This stacking restriction does not apply to meta items
 * which can be placed anywhere in the stack with no effect on the meaning
 * of the resulting pattern.
 *
 * The end of the item[] stack is detected either by reaching max or a END
 * item, whichever comes first.
 */
struct rte_flow_pattern {
	uint32_t max; /**< Maximum number of entries in item[]. */
	struct rte_flow_item item[]; /**< Stacked items. */
};

/**
 * Action types.
 *
 * Each possible action is represented by a type. Some have associated
 * configuration structures. Several actions combined in a list can be
 * affected to a flow rule. That list is not ordered.
 *
 * They fall in three categories:
 *
 * - Terminating actions (such as QUEUE, DROP, RSS, PF, VF) that prevent
 *   processing matched packets by subsequent flow rules, unless overridden
 *   with PASSTHRU.
 *
 * - Non terminating actions (PASSTHRU, DUP) that leave matched packets up
 *   for additional processing by subsequent flow rules.
 *
 * - Other non terminating meta actions that do not affect the fate of
 *   packets (END, VOID, MARK, FLAG, COUNT).
 *
 * When several actions are combined in a flow rule, they should all have
 * different types (e.g. dropping a packet twice is not possible). The
 * defined behavior is for PMDs to only take into account the last action of
 * a given type found in the list. PMDs still perform error checking on the
 * entire list.
 *
 * Note that PASSTHRU is the only action able to override a terminating
 * rule.
 */
enum rte_flow_action_type {
	/**
	 * [META]
	 *
	 * End marker for action lists. Prevents further processing of
	 * actions, thereby ending the list.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_END,

	/**
	 * [META]
	 *
	 * Used as a placeholder for convenience. It is ignored and simply
	 * discarded by PMDs.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_VOID,

	/**
	 * Leaves packets up for additional processing by subsequent flow
	 * rules. This is the default when a rule does not contain a
	 * terminating action, but can be specified to force a rule to
	 * become non-terminating.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_PASSTHRU,

	/**
	 * [META]
	 *
	 * Attaches a 32 bit value to packets.
	 *
	 * See struct rte_flow_action_mark.
	 */
	RTE_FLOW_ACTION_TYPE_MARK,

	/**
	 * [META]
	 *
	 * Flag packets. Similar to MARK but only affects ol_flags.
	 *
	 * Note: a distinctive flag must be defined for it.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_FLAG,

	/**
	 * Assigns packets to a given queue index.
	 *
	 * See struct rte_flow_action_queue.
	 */
	RTE_FLOW_ACTION_TYPE_QUEUE,

	/**
	 * Drops packets.
	 *
	 * PASSTHRU overrides this action if both are specified.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_DROP,

	/**
	 * [META]
	 *
	 * Enables counters for this rule.
	 *
	 * These counters can be retrieved and reset through rte_flow_query(),
	 * see struct rte_flow_query_count.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_COUNT,

	/**
	 * Duplicates packets to a given queue index.
	 *
	 * This is normally combined with QUEUE, however when used alone, it
	 * is actually similar to QUEUE + PASSTHRU.
	 *
	 * See struct rte_flow_action_dup.
	 */
	RTE_FLOW_ACTION_TYPE_DUP,

	/**
	 * Similar to QUEUE, except RSS is additionally performed on packets
	 * to spread them among several queues according to the provided
	 * parameters.
	 *
	 * See struct rte_flow_action_rss.
	 */
	RTE_FLOW_ACTION_TYPE_RSS,

	/**
	 * Redirects packets to the physical function (PF) of the current
	 * device.
	 *
	 * No associated configuration structure.
	 */
	RTE_FLOW_ACTION_TYPE_PF,

	/**
	 * Redirects packets to the virtual function (VF) of the current
	 * device with the specified ID.
	 *
	 * See struct rte_flow_action_vf.
	 */
	RTE_FLOW_ACTION_TYPE_VF,
};

/**
 * RTE_FLOW_ACTION_TYPE_MARK
 *
 * Attaches a 32 bit value to packets.
 *
 * This value is arbitrary and application-defined. For compatibility with
 * FDIR it is returned in the hash.fdir.hi mbuf field. PKT_RX_FDIR_ID is
 * also set in ol_flags.
 */
struct rte_flow_action_id {
	uint32_t id; /**< 32 bit value to return with packets. */
};

/**
 * RTE_FLOW_ACTION_TYPE_QUEUE
 *
 * Assign packets to a given queue index.
 *
 * Terminating by default.
 */
struct rte_flow_action_queue {
	uint16_t queue; /**< Queue index to use. */
};

/**
 * RTE_FLOW_ACTION_TYPE_COUNT (query)
 *
 * Query structure to retrieve and reset flow rule counters.
 */
struct rte_flow_query_count {
	uint32_t reset:1; /**< Reset counters after query [in]. */
	uint32_t hits_set:1; /**< hits field is set [out]. */
	uint32_t bytes_set:1; /**< bytes field is set [out]. */
	uint32_t reserved:29; /**< Reserved, must be zero [in, out]. */
	uint64_t hits; /**< Number of hits for this rule [out]. */
	uint64_t bytes; /**< Number of bytes through this rule [out]. */
};

/**
 * RTE_FLOW_ACTION_TYPE_DUP
 *
 * Duplicates packets to a given queue index.
 *
 * This is normally combined with QUEUE, however when used alone, it is
 * actually similar to QUEUE + PASSTHRU.
 *
 * Non-terminating by default.
 */
struct rte_flow_action_dup {
	uint16_t queue; /**< Queue index to duplicate packet to. */
};

/**
 * RTE_FLOW_ACTION_TYPE_RSS
 *
 * Similar to QUEUE, except RSS is additionally performed on packets to
 * spread them among several queues according to the provided parameters.
 *
 * Note: RSS hash result is normally stored in the hash.rss mbuf field,
 * however it conflicts with the MARK action as they share the same
 * space. When both actions are specified, the RSS hash is discarded and
 * PKT_RX_RSS_HASH is not set in ol_flags. MARK has priority. The mbuf
 * structure should eventually evolve to store both.
 *
 * Terminating by default.
 */
struct rte_flow_action_rss {
	struct rte_eth_rss_conf *rss_conf; /**< RSS parameters. */
	uint16_t queues; /**< Number of entries in queue[]. */
	uint16_t queue[]; /**< Queues indices to use. */
};

/**
 * RTE_FLOW_ACTION_TYPE_VF
 *
 * Redirects packets to a virtual function (VF) of the current device.
 *
 * Packets matched by a VF pattern item can be redirected to their original
 * VF ID instead of the specified one. This parameter may not be available
 * and is not guaranteed to work properly if the VF part is matched by a
 * prior flow rule or if packets are not addressed to a VF in the first
 * place.
 *
 * Terminating by default.
 */
struct rte_flow_action_vf {
	uint32_t original:1; /**< Use original VF ID if possible. */
	uint32_t reserved:31; /**< Reserved, must be zero. */
	uint16_t vf; /**< VF ID to redirect packets to. */
};

/**
 * Definition of a single action.
 *
 * For simple actions without a configuration structure, conf remains NULL.
 */
struct rte_flow_action {
	enum rte_flow_action_type type; /**< Action type. */
	const void *conf; /**< Pointer to action configuration structure. */
};

/**
 * List of actions to associate with a flow.
 *
 * The end of the action[] list is detected either by reaching max or a END
 * action, whichever comes first.
 */
struct rte_flow_actions {
	uint32_t max; /**< Maximum number of entries in action[]. */
	struct rte_flow_action action[]; /**< Actions to perform. */
};

/**
 * Opaque type returned after successfully creating a flow.
 *
 * This handle can be used to manage and query the related flow (e.g. to
 * destroy it or retrieve counters).
 */
struct rte_flow;

/**
 * Verbose error types.
 *
 * Most of them provide the type of the object referenced by struct
 * rte_flow_error.cause.
 */
enum rte_flow_error_type {
	RTE_FLOW_ERROR_TYPE_NONE, /**< No error. */
	RTE_FLOW_ERROR_TYPE_UNDEFINED, /**< Cause is undefined. */
	RTE_FLOW_ERROR_TYPE_HANDLE, /**< Flow rule (handle). */
	RTE_FLOW_ERROR_TYPE_ATTR_GROUP, /**< Group field. */
	RTE_FLOW_ERROR_TYPE_ATTR_PRIORITY, /**< Priority field. */
	RTE_FLOW_ERROR_TYPE_ATTR_INGRESS, /**< field. */
	RTE_FLOW_ERROR_TYPE_ATTR_EGRESS, /**< field. */
	RTE_FLOW_ERROR_TYPE_ATTR, /**< Attributes structure itself. */
	RTE_FLOW_ERROR_TYPE_PATTERN_MAX, /**< Pattern length (max field). */
	RTE_FLOW_ERROR_TYPE_PATTERN_ITEM, /**< Specific pattern item. */
	RTE_FLOW_ERROR_TYPE_PATTERN, /**< Pattern structure itself. */
	RTE_FLOW_ERROR_TYPE_ACTION_MAX, /**< Number of actions (max field). */
	RTE_FLOW_ERROR_TYPE_ACTION, /**< Specific action. */
	RTE_FLOW_ERROR_TYPE_ACTIONS, /**< Actions structure itself. */
};

/**
 * Verbose error structure definition.
 *
 * This object is normally allocated by applications and set by PMDs, the
 * message points to a constant string which does not need to be freed by
 * the application, however its pointer can be considered valid only as long
 * as its associated DPDK port remains configured. Closing the underlying
 * device or unloading the PMD invalidates it.
 *
 * Both cause and message may be NULL regardless of the error type.
 */
struct rte_flow_error {
	enum rte_flow_error_type type; /**< Cause field and error types. */
	void *cause; /**< Object responsible for the error. */
	const char *message; /**< Human-readable error message. */
};

/**
 * Check whether a flow rule can be created on a given port.
 *
 * While this function has no effect on the target device, the flow rule is
 * validated against its current configuration state and the returned value
 * should be considered valid by the caller for that state only.
 *
 * The returned value is guaranteed to remain valid only as long as no
 * successful calls to rte_flow_create() or rte_flow_destroy() are made in
 * the meantime and no device parameter affecting flow rules in any way are
 * modified, due to possible collisions or resource limitations (although in
 * such cases EINVAL should not be returned).
 *
 * @param port_id
 *   Port identifier of Ethernet device.
 * @param[in] attr
 *   Flow rule attributes.
 * @param[in] pattern
 *   Pattern specification.
 * @param[in] actions
 *   Actions associated with the flow definition.
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 if flow rule is valid and can be created. A negative errno value
 *   otherwise (rte_errno is also set), the following errors are defined:
 *
 *   -ENOSYS: underlying device does not support this functionality.
 *
 *   -EINVAL: unknown or invalid rule specification.
 *
 *   -ENOTSUP: valid but unsupported rule specification (e.g. partial
 *   bit-masks are unsupported).
 *
 *   -EEXIST: collision with an existing rule.
 *
 *   -ENOMEM: not enough resources.
 *
 *   -EBUSY: action cannot be performed due to busy device resources, may
 *   succeed if the affected queues or even the entire port are in a stopped
 *   state (see rte_eth_dev_rx_queue_stop() and rte_eth_dev_stop()).
 */
int
rte_flow_validate(uint8_t port_id,
		  const struct rte_flow_attr *attr,
		  const struct rte_flow_pattern *pattern,
		  const struct rte_flow_actions *actions,
		  struct rte_flow_error *error);

/**
 * Create a flow rule on a given port.
 *
 * @param port_id
 *   Port identifier of Ethernet device.
 * @param[in] attr
 *   Flow rule attributes.
 * @param[in] pattern
 *   Pattern specification.
 * @param[in] actions
 *   Actions associated with the flow definition.
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   A valid handle in case of success, NULL otherwise and rte_errno is set
 *   to the positive version of one of the error codes defined for
 *   rte_flow_validate().
 */
struct rte_flow *
rte_flow_create(uint8_t port_id,
		const struct rte_flow_attr *attr,
		const struct rte_flow_pattern *pattern,
		const struct rte_flow_actions *actions,
		struct rte_flow_error *error);

/**
 * Destroy a flow rule on a given port.
 *
 * Failure to destroy a flow rule handle may occur when other flow rules
 * depend on it, and destroying it would result in an inconsistent state.
 *
 * This function is only guaranteed to succeed if handles are destroyed in
 * reverse order of their creation.
 *
 * @param port_id
 *   Port identifier of Ethernet device.
 * @param flow
 *   Flow rule handle to destroy.
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
rte_flow_destroy(uint8_t port_id,
		 struct rte_flow *flow,
		 struct rte_flow_error *error);

/**
 * Destroy all flow rules associated with a port.
 *
 * In the unlikely event of failure, handles are still considered destroyed
 * and no longer valid but the port must be assumed to be in an inconsistent
 * state.
 *
 * @param port_id
 *   Port identifier of Ethernet device.
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
rte_flow_flush(uint8_t port_id,
	       struct rte_flow_error *error);

/**
 * Query an existing flow rule.
 *
 * This function allows retrieving flow-specific data such as counters.
 * Data is gathered by special actions which must be present in the flow
 * rule definition.
 *
 * @param port_id
 *   Port identifier of Ethernet device.
 * @param flow
 *   Flow rule handle to query.
 * @param action
 *   Action type to query.
 * @param[in, out] data
 *   Pointer to storage for the associated query data type.
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
rte_flow_query(uint8_t port_id,
	       struct rte_flow *flow,
	       enum rte_flow_action_type action,
	       void *data,
	       struct rte_flow_error *error);

#ifdef __cplusplus
}
#endif

#endif /* RTE_FLOW_H_ */
