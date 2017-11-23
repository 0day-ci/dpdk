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
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <rte_log.h>
#include <mlx5_utils.h>
#include "mlx5_dll.h"

#define VERBS_LIB_DIR "/usr/lib64/libibverbs"
#define MLX5_LIB_DIR "/usr/lib64/libmlx5"
#define DIR_LENGTH 25
/**
 * Load a libibverbs and libmlx5 symbols table.
 *
 * @return
 *   0 on success.
 */
int mlx5_load_libs(void)
{
	void *dlhandle;
	int ret;

	dlhandle = mlx5_lib_load("ibverbs.so");
	if (dlhandle == NULL) {
		ERROR("cannot load ibverbs.so");
		return -1;
	}
	ret = mlx5_lverbs_function_register(dlhandle);
	if (ret == -1) {
		ERROR("cannot register a function in libverbs.so ");
		return ret;
	}
	dlhandle = mlx5_lib_load("mlx5.so");
	if (dlhandle == NULL) {
		ERROR("cannot load mlx5.so");
		return -1;
	}
	ret = mlx5_lmlx5_function_register(dlhandle);
	if (ret == -1) {
		ERROR("cannot register a function in lmlx5.so  ");
		return ret;
	}
	return 0;
}
/**
 * Load a libibverbs or libmlx5 symbols table.
 *
 * @param name[in]
 *   The library name.
 * @return
 *   dlhandle on success.
 */
void *mlx5_lib_load(const char *name)
{
	char *so_name;
	void *dlhandle;
	char lib[DIR_LENGTH];

	if (strcmp(name, "ibverbs.so"))
		strcpy(lib, VERBS_LIB_DIR);
	else
		strcpy(lib, MLX5_LIB_DIR);
	/* If the name is an absolute path then open that path after appending
	 * the trailer suffix
	 */
	if (name[0] == '/') {
		if (asprintf(&so_name, "%s", name) < 0)
			goto out_asprintf;
		dlhandle = dlopen(so_name, RTLD_LAZY);
		if (!dlhandle)
			goto out_dlopen;
		free(so_name);
		return dlhandle;
	}
	/* If configured with a provider plugin path then try that next */
	if (strlen(lib) > 1) {
		if (asprintf(&so_name, "%s/lib%s", lib, name) < 0)
			goto out_asprintf;
		dlhandle = dlopen(so_name, RTLD_LAZY);
		free(so_name);
		if (dlhandle)
			return dlhandle;
	}
	/* Otherwise use the system libary search path. This is the historical
	 * behavior of libibverbs
	 */
	if (asprintf(&so_name, "lib%s", name) < 0)
		goto out_asprintf;
	dlhandle = dlopen(so_name, RTLD_LAZY);
	if (!dlhandle)
		goto out_dlopen;
	free(so_name);
	return dlhandle;
out_asprintf:
	ERROR("Warning: couldn't load driver '%s'.\n", name);
	return NULL;
out_dlopen:
	ERROR("Warning: couldn't load driver '%s': %s\n", so_name, dlerror());
	free(so_name);
	return NULL;
}

/**
 * Register libibverbs functions apis .
 *
 * @param handle[in]
 *   The library handle.
 * @return
 *   0 on success.
 */
int mlx5_lverbs_function_register(void *handle)
{
	char *error;
	int ret = 0;

	*(void **)(&mlx5_libverbs.ibv_destroy_qp) = dlsym(handle,
													  "ibv_destroy_qp");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_destroy_cq) = dlsym(handle,
													  "ibv_destroy_cq");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_dealloc_pd) = dlsym(handle,
													  "ibv_dealloc_pd");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_get_device_list) =
									dlsym(handle, "ibv_get_device_list");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_open_device) = dlsym(handle,
													   "ibv_open_device");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_query_device) = dlsym(handle,
														"ibv_query_device");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_close_device) = dlsym(handle,
														"ibv_close_device");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_free_device_list) =
										dlsym(handle, "ibv_free_device_list");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_alloc_pd) = dlsym(handle, "ibv_alloc_pd");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_get_device_name) =
									dlsym(handle, "ibv_get_device_name");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_fork_init) = dlsym(handle,
													 "ibv_fork_init");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_destroy_comp_channel) =
									dlsym(handle, "ibv_destroy_comp_channel");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_dereg_mr) = dlsym(handle, "ibv_dereg_mr");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_create_comp_channel) =
									dlsym(handle, "ibv_create_comp_channel");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_get_cq_event) = dlsym(handle,
														"ibv_get_cq_event");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_create_qp) = dlsym(handle,
													 "ibv_create_qp");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_ack_async_event) =
									dlsym(handle, "ibv_ack_async_event");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_get_async_event) =
									dlsym(handle, "ibv_get_async_event");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_reg_mr) = dlsym(handle, "ibv_reg_mr");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_create_cq) = dlsym(handle,
													 "ibv_create_cq");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_modify_qp) = dlsym(handle,
													 "ibv_modify_qp");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_libverbs.ibv_query_port) = dlsym(handle,
													  "ibv_query_port");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	return 0;
exit:
	return -ret;
}

/**
 * Register libmlx5 functions apis .
 *
 * @param handle[in]
 *   The library handle.
 * @return
 *   0 on success.
 */
int mlx5_lmlx5_function_register(void *handle)
{
	char *error;
	int ret = 0;

	*(void **)(&mlx5_lmlx5.mlx5dv_create_cq) = dlsym(handle,
													 "mlx5dv_create_cq");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_lmlx5.mlx5dv_init_obj) = dlsym(handle,
													"mlx5dv_init_obj");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_lmlx5.mlx5dv_query_device) = dlsym(handle,
														"mlx5dv_query_device");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	*(void **)(&mlx5_lmlx5.mlx5dv_set_context_attr) =
								 dlsym(handle, "mlx5dv_set_context_attr");
	error = dlerror();
	if ((error) != NULL) {
		ERROR("%s\n", error);
		ret = EINVAL;
		goto exit;
	}
	return 0;
exit:
	return -ret;
}

/**
 * Function pointer calls mlx5 API .
 *
 * @param context[in]
 *   The ibv context.
 * @param cq_attr[in]
 *   The completion queue attribute.
 * @param mlx5_cq_attr[in]
 *   The mlx5 completion queue attribute.
 * @return
 *   a pointer on success.
 */
struct ibv_cq_ex *mlx5dv_create_cq(struct ibv_context *context,
								   struct ibv_cq_init_attr_ex *cq_attr,
								   struct mlx5dv_cq_init_attr *mlx5_cq_attr)
{
	return mlx5_lmlx5.mlx5dv_create_cq(context, cq_attr, mlx5_cq_attr);
}
/**
 * Function pointer calls mlx5 API .
 *
 * @param mlx5dv_obj[in]
 *   The mlx5 object.
 * @param obj_type[in]
 *   The object type.
 * @return
 *   a pointer on success.
 */
int mlx5dv_init_obj(struct mlx5dv_obj *obj, uint64_t obj_type)
{
	return mlx5_lmlx5.mlx5dv_init_obj(obj, obj_type);
}
/**
 * Function pointer calls mlx5 API .
 *
 * @param ctx_in[in]
 *   The ibverbs context.
 * @param attrs_out[out]
 *   The mlx5 context.
 * @return
 *  0 on success.
 */
int mlx5dv_query_device(struct ibv_context *ctx_in,
						struct mlx5dv_context *attrs_out)
{
	return mlx5_lmlx5.mlx5dv_query_device(ctx_in, attrs_out);
}
/**
 * Function pointer calls mlx5 API .
 *
 * @param context[in]
 *   The ibverbs context.
 * @param type[in]
 *   The mlx5 context attribute type.
 * @param attr[in]
 *   The mlx5 context attribute.
 * @return
 *  0 on success.
 */
int mlx5dv_set_context_attr(struct ibv_context *context,
							enum mlx5dv_set_ctx_attr_type type,
							void *attr)
{
	return mlx5_lmlx5.mlx5dv_set_context_attr(context, type, attr);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param qp[in]
 *   Queue pair pointer.
 * @return
 *  0 on success.
 */
int ibv_destroy_qp(struct ibv_qp *qp)
{
	return mlx5_libverbs.ibv_destroy_qp(qp);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param cq[in]
 *   Completion queue pointer.
 * @return
 *  0 on success.
 */
int ibv_destroy_cq(struct ibv_cq *cq)
{
	return mlx5_libverbs.ibv_destroy_cq(cq);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param pd[in]
 *   Protection domain.
 * @return
 *  0 on success.
 */
int ibv_dealloc_pd(struct ibv_pd *pd)
{
	return mlx5_libverbs.ibv_dealloc_pd(pd);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param num_devices[in]
 *   The number of devices.
 * @return
 *  0 on success.
 */
struct ibv_device **ibv_get_device_list(int *num_devices)
{
	return mlx5_libverbs.ibv_get_device_list(num_devices);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   The ibverbs context.
 * @param device_attr[in]
 *   The device attribute.
 * @return
 *  0 on success.
 */
int ibv_query_device(struct ibv_context *context,
					 struct ibv_device_attr *device_attr)
{
	return mlx5_libverbs.ibv_query_device(context, device_attr);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   The ibverbs context.
 * @return
 *  0 on success.
 */
int ibv_close_device(struct ibv_context *context)
{
	return mlx5_libverbs.ibv_close_device(context);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param list[in]
 *   The list of devices.
 */
void ibv_free_device_list(struct ibv_device **list)
{
	return mlx5_libverbs.ibv_free_device_list(list);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param device[in]
 *   A device pointer.
 * @return
 *  The context on success.
 */
struct ibv_context *ibv_open_device(struct ibv_device *device)
{
	return mlx5_libverbs.ibv_open_device(device);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   An ibverbs context.
 * @return
 *  protection domain pointer on success.
 */
struct ibv_pd *ibv_alloc_pd(struct ibv_context *context)
{
	return mlx5_libverbs.ibv_alloc_pd(context);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param device[in]
 *   An ibverbs device pointer.
 * @return
 *  device name on success.
 */
const char *ibv_get_device_name(struct ibv_device *device)
{
	return mlx5_libverbs.ibv_get_device_name(device);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @return
 *  0 on success.
 */
int ibv_fork_init(void)
{
	return mlx5_libverbs.ibv_fork_init();
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param channel[in]
 *   A completion channel.
 * @return
 *  0 on success.
 */
int ibv_destroy_comp_channel(struct ibv_comp_channel *channel)
{
	return mlx5_libverbs.ibv_destroy_comp_channel(channel);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param mr[in]
 *   A pointer to memory region.
 * @return
 *  0 on success.
 */
int ibv_dereg_mr(struct ibv_mr *mr)
{
	return mlx5_libverbs.ibv_dereg_mr(mr);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   A pointer to ibverbs context.
 * @return
 *  0 on success.
 */
struct ibv_comp_channel *ibv_create_comp_channel(
											  struct ibv_context *context)
{
	return mlx5_libverbs.ibv_create_comp_channel(context);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param channel[in]
 *   A pointer to completion channel.
 * @param cq[in]
 *   A pointer to completion queue.
 * @param cq_context[in]
 *   A pointer to completion queue context.
 * @return
 *  0 on success.
 */
int ibv_get_cq_event(struct ibv_comp_channel *channel,
					 struct ibv_cq **cq, void **cq_context)
{
	return mlx5_libverbs.ibv_get_cq_event(channel, cq, cq_context);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param cq[in]
 *   A pointer to completion queue.
 * @param nevents[in]
 *   The number of events.
 * @return
 *  0 on success.
 */
void ibv_ack_cq_events(struct ibv_cq *cq, unsigned int nevents)
{
	return mlx5_libverbs.ibv_ack_cq_events(cq, nevents);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param pd[in]
 *   The protection domain pointer.
 * @param qp_init_attr[in]
 *   The queue pair attributes.
 * @return
 *  A pointer to queue pair on success.
 */
struct ibv_qp *ibv_create_qp(struct ibv_pd *pd,
							 struct ibv_qp_init_attr *qp_init_attr)
{
	return mlx5_libverbs.ibv_create_qp(pd, qp_init_attr);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param event[in]
 *   A pointer to a-synchronic event.
 */
void ibv_ack_async_event(struct ibv_async_event *event)
{
	return mlx5_libverbs.ibv_ack_async_event(event);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   A ibverbs context.
 * @param event[in]
 *   A pointer to a-synchronic event.
 */
int ibv_get_async_event(struct ibv_context *context,
						struct ibv_async_event *event)
{
	return mlx5_libverbs.ibv_get_async_event(context, event);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param pd[in]
 *   The protection domain pointer.
 * @param addr[in]
 *   The address.
 * @param length[in]
 *   The length.
 * @param access[in]
 *   The access type.
 * @return
 *  A pointer to memory region on success.
 */
struct ibv_mr *ibv_reg_mr(struct ibv_pd *pd, void *addr,
						  size_t length, int access)
{
	return mlx5_libverbs.ibv_reg_mr(pd, addr, length, access);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   Ibverbs context.
 * @param cqe[in]
 *   A completion queue entry.
 * @param cq_context[in]
 *   Completion queue context.
 * @param channel[in]
 *   The completion channel.
 * @param comp_vector[in]
 *   Completion vector.
 * @return
 *  0 on success.
 */
struct ibv_cq *ibv_create_cq(struct ibv_context *context, int cqe,
							 void *cq_context,
							 struct ibv_comp_channel *channel,
							 int comp_vector)
{
	return mlx5_libverbs.ibv_create_cq(context, cqe, cq_context, channel,
									   comp_vector);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param qp[in]
 *   Queue pair.
 * @param attr[in]
 *   The queue pair attribute.
 * @param attr_mask[in]
 *   Attribute mask.
 * @return
 *  0 on success.
 */
int ibv_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
				  int attr_mask)
{
	return mlx5_libverbs.ibv_modify_qp(qp, attr, attr_mask);
}
/**
 * Function pointer calls libibverbs API .
 *
 * @param context[in]
 *   An ibverbs context.
 * @param port_num[in]
 *   The port number.
 * @param port_attr[in]
 *   Port attribute.
 * @return
 *  0 on success.
 */
int ibv_query_port(struct ibv_context *context, uint8_t port_num,
				   struct ibv_port_attr *port_attr)
{
	port_attr->link_layer = IBV_LINK_LAYER_UNSPECIFIED;
	port_attr->reserved   = 0;
	return mlx5_libverbs.ibv_query_port(context, port_num, port_attr);
}
