#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stdio.h>

#include <rte_common.h>
#include <rte_eal.h>
#include <rte_log.h>
#include <rte_mempool.h>

class MemoryPool : public ::testing::Test {

protected:

	virtual void SetUp() {
		mp = NULL;
		mempool_elements = (rte_lcore_count() * (lcore_cache_size +
				RTE_MEMPOOL_CACHE_MAX_SIZE) -1);
	}

	virtual void TearDown() {
		if (mp)
			rte_mempool_free(mp);

	}

	/*
	 * save the object number in the first 4 bytes of object data. All
	 * other bytes are set to 0.
	 */
	static void
	obj_initializer_fn(struct rte_mempool *mp, void *arg __rte_unused,
			void *obj, unsigned i)
	{
		uint32_t *objnum = (uint32_t *)obj;

		memset(obj, 0, mp->elt_size);
		*objnum = i;
	}

	struct rte_mempool *mp;

	static const unsigned element_size = 2048;
	static const uint32_t lcore_cache_size = 16;

	unsigned mempool_elements;
};

TEST_F(MemoryPool, CreateWithNoCache)
{
	mp = rte_mempool_create("test_nocache", mempool_elements, element_size,
			0, 0, NULL, NULL, obj_initializer_fn, NULL,
			SOCKET_ID_ANY, 0);

	ASSERT_NE(mp, (void *)NULL);

	EXPECT_EQ(mp, rte_mempool_lookup("test_nocache"));
	EXPECT_EQ(rte_mempool_avail_count(mp), mempool_elements);
}

TEST_F(MemoryPool, CreateWithCache)
{
	mp = rte_mempool_create("test_cache", mempool_elements, element_size,
			RTE_MEMPOOL_CACHE_MAX_SIZE, 0, NULL, NULL,
			obj_initializer_fn, NULL, SOCKET_ID_ANY, 0);

	ASSERT_NE(mp, (void *)NULL);

	EXPECT_EQ(mp, rte_mempool_lookup("test_cache"));
	EXPECT_EQ(rte_mempool_avail_count(mp), mempool_elements);
}

TEST_F(MemoryPool, ExceedMaxCacheSize)
{
	mp = rte_mempool_create("test_exceed_max_cache_sz",
			mempool_elements, element_size,
			RTE_MEMPOOL_CACHE_MAX_SIZE + 1, 0, NULL, NULL,
			obj_initializer_fn, NULL, SOCKET_ID_ANY, 0);

	ASSERT_EQ(mp, (void *)NULL);
}

TEST_F(MemoryPool, MempoolNameCollosion) {
	mp = rte_mempool_create("test_name_collosion", mempool_elements,
			element_size, RTE_MEMPOOL_CACHE_MAX_SIZE, 0, NULL, NULL,
			obj_initializer_fn, NULL, SOCKET_ID_ANY, 0);

	ASSERT_NE(mp, (void *)NULL);

	ASSERT_EQ(rte_mempool_create("test_name_collosion", mempool_elements,
			element_size, RTE_MEMPOOL_CACHE_MAX_SIZE, 0, NULL, NULL,
			obj_initializer_fn, NULL, SOCKET_ID_ANY, 0),
			(struct rte_mempool *)NULL);
}

TEST_F(MemoryPool, ExternalMemorySize) {
	size_t sz;
	ssize_t usz;

	uint32_t obj_size = rte_mempool_calc_obj_size(element_size, 0, NULL);

	sz = rte_mempool_xmem_size(mempool_elements, obj_size,
			MEMPOOL_PG_SHIFT_MAX);

	usz = rte_mempool_xmem_usage(NULL, mempool_elements, obj_size, 0, 1,
			MEMPOOL_PG_SHIFT_MAX);

	EXPECT_EQ(sz, (size_t)usz);
}


class MempoolMock {
public:

	MOCK_METHOD2(MempoolInitializerMockFn,
			void(struct rte_mempool *, void *));

	MOCK_METHOD4(MempoolObjInitializerMockFn,
			void(struct rte_mempool *, void *arg, void *, unsigned));

	static MempoolMock *obj;

	static void mempool_initializer_fn(struct rte_mempool *mp, void * arg)
	{
		obj->MempoolInitializerMockFn(mp, arg);
	}

	static void mempool_obj_initializer_fn(struct rte_mempool *mp,
			void *arg, void *m_obj, unsigned i)
	{
		obj->MempoolObjInitializerMockFn(mp, arg, m_obj, i);
	}
};


MempoolMock * MempoolMock::obj;

TEST_F(MemoryPool, MempoolInitialization)
{
	MempoolMock mock;
	MempoolMock::obj = &mock;


	EXPECT_CALL(mock, MempoolInitializerMockFn(::testing::_, ::testing::_))
		.Times(1);

	EXPECT_CALL(mock, MempoolObjInitializerMockFn(
			::testing::_, ::testing::_, ::testing::_, ::testing::_))
			.Times(mempool_elements);

	mp = rte_mempool_create("test_nocache", mempool_elements,
			element_size, 0, 0,
			MempoolMock::mempool_initializer_fn, NULL,
			MempoolMock::mempool_obj_initializer_fn, NULL,
			SOCKET_ID_ANY, 0);
	ASSERT_NE(mp, (void *)NULL);

	EXPECT_EQ(rte_mempool_full(mp), 1);
}


class MemoryPoolWithNoCache : public MemoryPool {
protected :
	virtual void SetUp() {
		mempool_elements = (rte_lcore_count() * (lcore_cache_size +
				RTE_MEMPOOL_CACHE_MAX_SIZE) -1);

		objs = (void **)malloc(mempool_elements * sizeof(void *));

		mp = rte_mempool_create("test_nocache", mempool_elements,
				element_size, 0, 0, NULL, NULL,
				obj_initializer_fn, NULL, SOCKET_ID_ANY, 0);
		ASSERT_NE(mp, (void *)NULL);

		EXPECT_EQ(rte_mempool_full(mp), 1);

	}

	virtual void TearDown() {
		if (mp)
			rte_mempool_free(mp);

		if (objs)
			free(objs);

	}

	void **objs;
	int i;
};

TEST_F(MemoryPoolWithNoCache, GetOneObjectPutOneObject)
{
	/* Get an object for mempool */
	ASSERT_EQ(rte_mempool_get(mp, &objs[0]), 0);

	/* Verify object count in mempool */
	EXPECT_EQ(rte_mempool_avail_count(mp), mempool_elements -1);

	/* Verify physical address of an object */

#ifndef RTE_EXEC_ENV_BSDAPP
	/* rte_mem_virt2phy() not supported on BSD */
	EXPECT_EQ(rte_mempool_virt2phy(mp, objs[0]), rte_mem_virt2phy(objs[0]));
#endif

	/* put the object back */
	rte_mempool_put(mp, objs[0]);

	EXPECT_EQ(rte_mempool_avail_count(mp), mempool_elements);
}

TEST_F(MemoryPoolWithNoCache, GetAllObjectsPutAllObjects)
{
	/* Get all objects from mempool */
	for (i = 0; i < mempool_elements; i++) {
		EXPECT_EQ(rte_mempool_get(mp, &objs[i]), 0);
		/* Verify physical address of an object */
#ifndef RTE_EXEC_ENV_BSDAPP
		EXPECT_EQ(rte_mempool_virt2phy(mp, objs[i]),
				rte_mem_virt2phy(objs[i]));
#endif
	}

	/* Verify object count in mempool */
	EXPECT_EQ(rte_mempool_avail_count(mp), 0);


	/* Put the objects back */
	for (i = 0; i < mempool_elements; i++) {
		rte_mempool_put(mp, objs[i]);
	}

	EXPECT_EQ(rte_mempool_avail_count(mp), mempool_elements);
}

TEST_F(MemoryPoolWithNoCache, GetOjectFromEmptyMempool)
{
	void *err_obj = NULL;

	for (i = 0; i < mempool_elements; i++)
		EXPECT_EQ(rte_mempool_get(mp, &objs[i]), 0);

	EXPECT_EQ(rte_mempool_empty(mp), 1);

	EXPECT_LT(rte_mempool_get(mp, &err_obj), 0);
	EXPECT_EQ(err_obj, (void *)NULL);

	for (i = 0; i < mempool_elements; i++)
		rte_mempool_put(mp, objs[i]);

	EXPECT_EQ(rte_mempool_full(mp), 1);
	// printf("test_mempool_basic_ex the mempool should be full\n");
}



class MemoryPoolSingleProducerSingleConsumer : public MemoryPool {
protected:
	virtual void SetUp() {
		lcore_id = rte_lcore_id();

		mempool_elements = (rte_lcore_count() * (lcore_cache_size +
				RTE_MEMPOOL_CACHE_MAX_SIZE) -1);

		mp = rte_mempool_create("single_producer_consumer",
				mempool_elements, element_size, 0, 0,
				NULL, NULL,
				obj_initializer_fn, NULL,
				SOCKET_ID_ANY,
				MEMPOOL_F_NO_CACHE_ALIGN |
				MEMPOOL_F_SP_PUT |
				MEMPOOL_F_SC_GET);

		ASSERT_NE(mp, (void *)NULL);

		EXPECT_EQ(rte_mempool_full(mp), 1);
	}

	unsigned lcore_id, lcore_next;
};
