

#include "imgui.h"
//#define SOKOL_GLCORE33 --- needs to be defined globally

#define SOKOL_IMGUI_IMPL
#define SOKOL_GFX_IMGUI_IMPL

#define SOKOL_GL_IMPL
#define SOKOL_IMPL
#define SOKOL_TRACE_HOOKS
#define SOKOL_WIN32_FORCE_MAIN

#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_glue.h"
#include "sokol_time.h"
#include "sokol_gl.h"
#include "sokol_imgui.h"
#include "sokol_gfx_imgui.h"

#ifdef SOKOL_GL_IMPL_INCLUDED
#endif
