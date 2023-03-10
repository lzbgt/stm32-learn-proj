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

static int read(RingBuf *self, uint8_t *dst)
{
  uint16_t sz = 0;
  if (self->curr == self->last && self->last == 0)
    return 0;

  if (self->curr > self->last)
  {
    sz = self->curr - self->last;
    memcpy(dst, self->src + self->last, sz);
    self->last = self->curr;
    return sz;
  }

  sz = self->size - self->last;
  memcpy(dst, self->src + self->last, sz);
  if (self->last != 0 && self->curr != 0)
  {
    memcpy(dst + sz, self->src, self->curr);
    sz += self->curr;
  }

  self->last = self->curr;
  return sz;
}

static int setcursor(RingBuf *self, uint16_t curr)
{
  self->curr = curr;
}

RingBuf ringbuf_new(uint8_t *src, uint16_t size)
{

  return (struct RingBuf){
      .src = src,
      .size = size,
      .curr = 0,
      .last = 0,
      .read = read,
      .setcursor = setcursor};
}

void ringbuf_free()
{
}