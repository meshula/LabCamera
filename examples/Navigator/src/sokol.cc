

#include "imgui.h"
//#define SOKOL_GLCORE33 --- needs to be defined globally
//#define HAVE_SOKOL_DBGUI
#ifndef HAVE_SOKOL_DBGUI
    #ifndef USE_DBG_UI
        #define USE_DBG_UI
    #endif

    // these two are defined in dbgui.cc
    #define SOKOL_IMGUI_IMPL
    #define SOKOL_GFX_IMGUI_IMPL
#endif

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

#ifdef HAVE_SOKOL_DBGUI
    #include "dbgui.h"
#endif

#ifdef SOKOL_GL_IMPL_INCLUDED
#endif
