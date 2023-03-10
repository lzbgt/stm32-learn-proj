/**
 * @file ringbuf_adaptor.h
 * @author Bruce.Lu (lzbgt@icloud.com)
 * @brief
 * @version 0.1
 * @date 2023-03-10
 *
 * Copyright (C) 2023
 *
 */

#include "stdint.h"

#ifndef _BLU_RING_BUF_
#define _BLU_RING_BUF_

struct RingBuf;

typedef struct RingBuf
{
  uint8_t *src;
  int16_t size;
  int16_t last;
  int full;
  int (*read)(struct RingBuf *self, uint8_t *dst, int16_t curr);
} RingBuf;

RingBuf ringbuf_new(uint8_t *src, int16_t size);

#endif