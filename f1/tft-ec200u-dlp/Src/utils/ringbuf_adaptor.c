/**
 * @file ringbuf_adaptor.c
 * @author Bruce.Lu (lzbgt@icloud.com)
 * @brief
 * @version 0.1
 * @date 2023-03-10
 *
 * Copyright (C) 2023
 *
 */
#include "ringbuf_adaptor.h"
#include <string.h>

static int read(RingBuf *self, uint8_t *dst, int16_t curr)
{
  int16_t sz = 0;
  if (curr == self->last && !self->full)
    return 0;

  if (curr > self->last)
  {
    sz = curr - self->last;
    memcpy(dst, self->src + self->last, sz);
    self->last = curr;
    return sz;
  }

  self->full = 1;
  sz = self->size - self->last;
  memcpy(dst, self->src + self->last, sz);
  memcpy(dst + sz, self->src, curr);
  sz += curr;
  dst[sz] = 0;

  self->last = curr;
  return sz;
}

RingBuf ringbuf_new(uint8_t *src, int16_t size)
{

  return (struct RingBuf){
      .src = src,
      .size = size,
      .full = 0,
      .last = 0,
      .read = read};
}

void ringbuf_free()
{
}