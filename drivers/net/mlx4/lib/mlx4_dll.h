/*-
 *   BSD LICENSE
 *
 *   Copyright 2015 6WIND S.A.
 *   Copyright 2015 Mellanox.
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
#ifndef RTE_PMD_MLX4_IBVERBS_H_
#define RTE_PMD_MLX4_IBVERBS_H_
#include <infiniband/verbs.h>
#include <infiniband/mlx4dv.h>

void *mlx4_lib_load(const char *name);
int mlx4_lverbs_function_register(void *handle);
int mlx4_lmlx4_function_register(void *handle);
int mlx4_load_libs(void);
#undef ibv_query_port

struct {
	int size;
	int version;
	int (*ibv_destroy_qp)(struct ibv_qp *qp);
	int (*ibv_destroy_cq)(struct ibv_cq *cq);
	int (*ibv_dealloc_pd)(struct ibv_pd *pd);
	struct ibv_device **(*ibv_get_device_list)(int *num_devices);
	int (*ibv_query_device)(struct ibv_context *context,
							struct ibv_device_attr *device_attr);
	int (*ibv_close_device)(struct ibv_context *context);
	void (*ibv_free_device_list)(struct ibv_device **list);
	struct ibv_context *(*ibv_open_device)(struct ibv_device *device);
	struct ibv_pd *(*ibv_alloc_pd)(struct ibv_context *context);
	const char *(*ibv_get_device_name)(struct ibv_device *device);
	int (*ibv_fork_init)(void);
	int (*ibv_destroy_comp_channel)(struct ibv_comp_channel *channel);
	int (*ibv_dereg_mr)(struct ibv_mr *mr);
	struct ibv_comp_channel *(*ibv_create_comp_channel)(
							  struct ibv_context *context);
	int(*ibv_get_cq_event)(struct ibv_comp_channel *channel,
						   struct ibv_cq **cq, void **cq_context);
	void (*ibv_ack_cq_events)(struct ibv_cq *cq, unsigned int nevents);
	struct ibv_qp *(*ibv_create_qp)(struct ibv_pd *pd,
									struct ibv_qp_init_attr *qp_init_attr);
	void (*ibv_ack_async_event)(struct ibv_async_event *event);
	int (*ibv_get_async_event)(struct ibv_context *context,
							   struct ibv_async_event *event);
	struct ibv_mr *(*ibv_reg_mr)(struct ibv_pd *pd, void *addr,
								 size_t length,
								 int access);
	struct ibv_cq *(*ibv_create_cq)(struct ibv_context *context,
									int cqe, void *cq_context,
									struct ibv_comp_channel *channel,
									int comp_vector);
	int (*ibv_modify_qp)(struct ibv_qp *qp, struct ibv_qp_attr *attr,
						 int attr_mask);
	int (*ibv_query_port)(struct ibv_context *context, uint8_t port_num,
						struct ibv_port_attr *port_attr);
}mlx4_libverbs;

struct {
	int size;
	int version;
	int (*mlx4dv_init_obj)(struct mlx4dv_obj *obj, uint64_t obj_type);
	int (*mlx4dv_query_device)(struct ibv_context *ctx_in,
							   struct mlx4dv_context *attrs_out);
	int (*mlx4dv_set_context_attr)(struct ibv_context *context,
								enum mlx4dv_set_ctx_attr_type type,
								void *attr);
}mlx4_lmlx4;
#endif /* RTE_PMD_MLX4_IBVERBS_H_ */
