/*
 * Copyright (c) 2017,2018 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DMABUF_CONTAINER_H__
#define __DMABUF_CONTAINER_H__

struct dma_buf;

#ifdef CONFIG_DMABUF_CONTAINER
bool is_dmabuf_container(struct dma_buf *dmabuf);
int dmabuf_container_get_count(struct dma_buf *dmabuf);
struct dma_buf *dmabuf_container_get_buffer(struct dma_buf *dmabuf, int index);
#else
static inline bool is_dmabuf_container(struct dma_buf *dmabuf)
{
	return false;
}
static inline int dmabuf_container_get_count(struct dma_buf *dmabuf)
{
	return 0;
}
static inline struct dma_buf *dmabuf_container_get_buffer(struct dma_buf *dbuf,
							  int index)
{
	return NULL;
}
#endif

#endif /* __DMABUF_CONTAINER_H__ */
