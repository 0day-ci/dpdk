#ifndef _RTE_SECURITY_H_
#define _RTE_SECURITY_H_

/**
 * @file rte_security.h
 *
 * RTE Security Common Definitions
 *
 */

#ifdef __cplusplus
extern "C" {
#endif


#include <rte_mbuf.h>
#include <rte_memory.h>
#include <rte_mempool.h>
#include <rte_common.h>

#include <rte_crypto.h>
#include <rte_cryptodev.h>

/** Security Assosiation direction for IPSec */
enum rte_security_conf_ipsec_sa_dir {
	RTE_SECURITY_IPSEC_SA_DIR_INGRESS,
	/**< Ingress path */
	RTE_SECURITY_IPSEC_SA_DIR_EGRESS
	/**< Egress path */
};

/** IPSec protocol mode */
enum rte_security_conf_ipsec_sa_mode {
	RTE_SECURITY_IPSEC_SA_MODE_TRANSPORT,
	/**< IPSec Transport mode */
	RTE_SECURITY_IPSEC_SA_MODE_TUNNEL
	/**< IPSec Tunnel mode */
};

/** IPSec Protocol */
enum rte_security_conf_ipsec_sa_protocol {
	RTE_SECURITY_IPSEC_SA_PROTO_AH,
	/**< AH protocol */
	RTE_SECURITY_IPSEC_SA_PROTO_ESP
	/**< ESP protocol */
};

/** IPSEC tunnel type */
enum rte_security_ipsec_tunnel_type {
	RTE_SECURITY_IPSEC_TUNNEL_IPV4 = 0,
	/**< Outer header is IPv4 */
	RTE_SECURITY_IPSEC_TUNNEL_IPV6
	/**< Outer header is IPv6 */
};

/**
 * IPSEC tunnel parameters
 *
 * These parameters are used to build outbound tunnel headers.
 */
struct rte_security_ipsec_tunnel_param {
	struct rte_security_ipsec_tunnel_type type;
	/**< Tunnel type: IPv4 or IPv6 */

	union {
		struct {
			struct ipaddr src_ip;
			/**< IPv4 source address */
			struct ipaddr dst_ip;
			/**< IPv4 destination address */
			uint8_t dscp;
			/**< IPv4 Differentiated Services Code Point */
			uint8_t df;
			/**< IPv4 Don't Fragment bit */
			uint8_t ttl;
			/**< IPv4 Time To Live */
		} ipv4; /**< IPv4 header parameters */

		struct {
			struct ip6addr *src_addr;
			/**< IPv6 source address */
			struct ip6addr *dst_addr;
			/**< IPv6 destination address */
			uint8_t dscp;
			/**< IPv6 Differentiated Services Code Point */
			uint32_t flabel;
			/**< IPv6 flow label */
			uint8_t hlimit;
			/**< IPv6 hop limit */
		} ipv6; /**< IPv6 header parameters */
	}; /**< Various tunnel parameters */
};

/**
 * IPSEC SA option flags
 */
struct rte_security_ipsec_sa_options {
	/** Extended Sequence Numbers (ESN)
	  *
	  * * 1: Use extended (64 bit) sequence numbers
	  * * 0: Use normal sequence numbers
	  */
	uint32_t esn : 1;

	/** UDP encapsulation
	  *
	  * * 1: Do UDP encapsulation/decapsulation so that IPSEC packets can
	  *      traverse through NAT boxes.
	  * * 0: No UDP encapsulation
	  */
	uint32_t udp_encap : 1;

	/** Copy DSCP bits
	  *
	  * * 1: Copy IPv4 or IPv6 DSCP bits from inner IP header to
	  *      the outer IP header in encapsulation, and vice versa in
	  *      decapsulation.
	  * * 0: Use values from odp_ipsec_tunnel_param_t in encapsulation and
	  *      do not change DSCP field in decapsulation.
	  */
	uint32_t copy_dscp : 1;

	/** Copy IPv6 Flow Label
	  *
	  * * 1: Copy IPv6 flow label from inner IPv6 header to the
	  *      outer IPv6 header.
	  * * 0: Use value from odp_ipsec_tunnel_param_t
	  */
	uint32_t copy_flabel : 1;

	/** Copy IPv4 Don't Fragment bit
	  *
	  * * 1: Copy the DF bit from the inner IPv4 header to the outer
	  *      IPv4 header.
	  * * 0: Use value from odp_ipsec_tunnel_param_t
	  */
	uint32_t copy_df : 1;

	/** Decrement inner packet Time To Live (TTL) field
	  *
	  * * 1: In tunnel mode, decrement inner packet IPv4 TTL or
	  *      IPv6 Hop Limit after tunnel decapsulation, or before tunnel
	  *      encapsulation.
	  * * 0: Inner packet is not modified.
	  */
	uint32_t dec_ttl : 1;

};

/** IPSec Operations */
enum rte_security_ipsec_operation {
	RTE_SECURITY_IPSEC_OP_ENCAP,
	/**< Encrypt and generate digest */
	RTE_SECURITY_IPSEC_OP_DECAP
	/**< Verify digest and decrypt */
};

/**
 * IPSec Setup Data.
 *
 * This structure contains data relating to IPSec
 * used to create a session.
 */
struct rte_security_ipsec_xform {
	enum rte_security_ipsec_operation op;
	/**< IPSec operation - Encap or Decap */
	enum rte_crypto_cipher_algorithm cipher_alg;
	/**< Cipher Algorithm */
	struct {
		uint8_t *data;  /**< pointer to key data */
		size_t length;   /**< key length in bytes */
	} cipher_key;
	enum rte_crypto_auth_algorithm auth_alg;
	/**< Authentication Algorithm */
	struct {
		uint8_t *data;  /**< pointer to key data */
		size_t length;   /**< key length in bytes */
	} auth_key;
	uint32_t salt;	/**< salt for this SA */
};

/** IPsec Security Session Configuration */
struct rte_security_conf_ipsec_sa {
	unsigned int spi;
	/**< SA security parameter index */

	enum rte_security_conf_ipsec_sa_dir sa_dir;
	/**< IPsec SA direction - ingress / egress */

	enum rte_security_conf_ipsec_sa_mode mode;
	/**< IPsec SA Mode - transport/tunnel */

	enum rte_security_conf_ipsec_sa_protocol proto;
	/**< IPsec SA Protocol - AH/ESP */

	struct rte_security_ipsec_xform *ipsec_xform;
	/**< IPsec Transform */

	struct rte_security_ipsec_tunnel_param *tunnel;
	/**< Tunnel parameters, NULL for transport mode */

	struct rte_security_ipsec_sa_options *options;
	/**< various SA options */
};

/** IPsec Security Session Configuration */
struct rte_security_conf_dtls {
	/** To be Filled */
};

/** IPsec Security Session Configuration */
struct rte_security_conf_macsec {
	/** To be Filled */
};

/**< Security Session action type */
enum rte_security_session_action_type {
	RTE_SECURITY_SESS_ETH_INLINE_CRYPTO,
	/**< Crypto operations are performed by Network interface */
	RTE_SECURITY_SESS_ETH_PROTO_OFFLOAD,
	/**< Crypto operations with protocol support are performed
	 * by Network/ethernet device.
	 */
	RTE_SECURITY_SESS_CRYPTO_PROTO_OFFLOAD,
	/**< Crypto operations with protocol support are performed
	 * by Crypto device.
	 */
	RTE_SECURITY_SESS_NONE
	/**< Non protocol offload. Application need to manage everything */
};

/** Security Session Protocols */
enum rte_security_sess_protocol {
	RTE_SEC_CONF_DTLS,
	/**< DTLS Protocol */
	RTE_SEC_CONF_IPSEC,
	/**< IPSec Protocol */
	RTE_SEC_CONF_MACSEC
	/**< MACSec Protocol */
};
/**
 * Security Session Configuration
 */
struct rte_security_sess_conf {
	enum rte_security_session_action_type action_type;
	/**< Type of action to be performed on the session */
	enum rte_security_sess_protocol protocol;
	/**< Security protocol to be configured */

	union {
		struct rte_security_conf_ipsec_sa ipsec_sa;
		struct rte_security_conf_dtls dtls;
		struct rte_security_conf_macsec macsec;
	};
	/**< Configuration parameters for security session */
};

struct rte_security_session {
	__extension__ void *sess_private_data[0];
	/**< Private session material */
};

/**
 * Configure device(crypto/ethernet) to enable Security operations
 *
 * @param   dev_id    Device id on which Security operations need to be enabled
 * @param   dev_name  Device name - crypto or ethernet device
 * @return
 *  - On success, zero
 *  - On failure, a negative value.
 */
int
rte_security_configure(uint16_t dev_id, char *dev_name);

/**
 * Create Security session header (generic with no private data)
 *
 * @param   mempool    Session mempool to allocate session objects from
 * @return
 *  - On success, pointer to session
 *  - On failure, NULL
 */
struct rte_security_session *
rte_security_session_create(struct rte_mempool *mempool);

/**
 * Fill out private data for the device, based on its device id and name.
 *
 * @param   dev_id   Device id that we want the session to be used on
 * @param   dev_name Device name for which session is to be used
 * @param   sess     Session where the private data will be attached to
 * @param   conf     Security config to apply on flow
 *                   processed with this session
 * @param   mempool  Mempool where the private data is allocated.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
int
rte_security_session_init(uint16_t dev_id, char *dev_name,
			  struct rte_security_session *sess,
			  struct rte_security_sess_conf *conf,
			  struct rte_mempool *mempool);

/**
 * Frees Security session header, after checking that all
 * the device private data has been freed, returning it
 * to its original mempool.
 *
 * @param   sess     Session header to be freed.
 *
 * @return
 *  - 0 if successful.
 *  - -EINVAL if session is NULL.
 *  - -EBUSY if not all device private data has been freed.
 */
int
rte_security_session_free(struct rte_security_session *sess);

/**
 * Frees private data for the device id, based on its device name,
 * returning it to its mempool.
 *
 * @param   dev_id   ID of device that uses the session.
 * @param   dev_name Name of device that uses the session.
 * @param   sess     Session containing the reference to the private data
 *
 * @return
 *  - 0 if successful.
 *  - -EINVAL if device is invalid or session is NULL.
 */
int
rte_security_session_clear(uint8_t dev_id, char *dev_name,
			   struct rte_security_session *sess);


/**
 * Attach a session to a crypto operation.
 * This API is needed only in case of RTE_SECURITY_SESS_CRYPTO_PROTO_OFFLOAD
 * For other rte_security_session_action_type, ol_flags in rte_mbuf may be
 * defined to perform security operations.
 *
 * @param	op	crypto operation
 * @param	sess	security session
 */
static inline int
rte_security_attach_session(struct rte_crypto_op *op,
			    struct rte_security_session *sess);

/**
 * Security Capability
 */
struct rte_security_capability {
	enum rte_security_sess_protocol protocol;
	RTE_STD_C11
	union {
		struct {
			enum rte_crypto_auth_algorithm algo;
			/**< authentication algorithm */
			enum rte_crypto_cipher_algorithm algo;
			/**< cipher algorithm */
			uint16_t block_size;
			/**< algorithm block size */
			struct rte_crypto_param_range c_key_size;
			/**< cipher key size range */
			struct rte_crypto_param_range a_key_size;
			/**< auth key size range */
			struct rte_crypto_param_range digest_size;
			/**< digest size range */
			struct rte_crypto_param_range iv_size;
			/**< Initialisation vector data size range */
		} ipsec;
		/**< IPSEC transform capabilities */
		struct {
			/* To be Filled */
		} dtls;
		/**< DTLS transform capabilities */
		struct {
			/* To be Filled */
		} macsec;
		/**< MACSEC transform capabilities */
	};
};

/**
 *  Provide capabilities available for defined device and algorithm
 *
 * @param	dev_id		The identifier of the device.
 * @param	dev_name	Device name for which capability is needed
 * @param	protocol	Protocol for which capability is required
 *
 * @return
 *   - Return description of the security capability if exist.
 *   - Return NULL if the capability not exist.
 */
const struct rte_security_capability *
rte_security_capability_get(uint8_t dev_id, char *dev_name,
		enum rte_security_sess_protocol protocol);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_SECURITY_H_ */
