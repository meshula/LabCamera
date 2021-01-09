
#ifndef INCLUDED_TINY_GIZMO_H
#define INCLUDED_TINY_GIZMO_H

#include <stdint.h>

#if defined(__cplusplus) && !defined(EXTERNC)
#   define EXTERNC extern "C"
#else
#   define EXTERNC
#endif

EXTERNC void draw_gizmo(const float* v_t, const float* mvp, int triangle_count, uint32_t* indices, float* vertices);

#endif
