//------------------------------------------------------------------------------
//  instancing.c
//  Demonstrate simple hardware-instancing using a static geometry buffer
//  and a dynamic instance-data buffer.
//------------------------------------------------------------------------------
#include <stdlib.h> /* rand() */
#include "sokol_app.h"
#include "sokol_gfx.h"
#define HANDMADE_MATH_IMPLEMENTATION
#define HANDMADE_MATH_NO_SSE
#include "HandmadeMath.h"
#include "instancing_glsl.h"

#include <mutex>

#define MAX_PARTICLES (512 * 1024)
#define NUM_PARTICLES_EMITTED_PER_FRAME (10)

/* a pass-action to clear to black */
static sg_pass_action pass_action;
static sg_pipeline pip;
static sg_bindings bind;
static float ry;

/* particle positions and velocity */
static int cur_num_particles = 0;
static hmm_vec3 pos[MAX_PARTICLES];
static hmm_vec3 vel[MAX_PARTICLES];

void init_particles() 
{
    pass_action.colors[0] = { SG_ACTION_CLEAR, { 0.0f, 0.0f, 0.0f, 1.0f } };

    /* vertex buffer for static geometry, goes into vertex-buffer-slot 0 */
    const float r = 0.05f;
    const float vertices[] = {
        // positions            colors
        0.0f,   -r, 0.0f,       1.0f, 0.0f, 0.0f, 1.0f,
           r, 0.0f, r,          0.0f, 1.0f, 0.0f, 1.0f,
           r, 0.0f, -r,         0.0f, 0.0f, 1.0f, 1.0f,
          -r, 0.0f, -r,         1.0f, 1.0f, 0.0f, 1.0f,
          -r, 0.0f, r,          0.0f, 1.0f, 1.0f, 1.0f,
        0.0f,    r, 0.0f,       1.0f, 0.0f, 1.0f, 1.0f
    };

    sg_buffer_desc buffer_desc;
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = _SG_BUFFERTYPE_DEFAULT;
    buffer_desc.size = sizeof(vertices);
    buffer_desc.content = vertices;
    buffer_desc.label = "geometry-vertices";
    bind.vertex_buffers[0] = sg_make_buffer(&buffer_desc);

    /* index buffer for static geometry */
    const uint16_t indices[] = {
        0, 1, 2,    0, 2, 3,    0, 3, 4,    0, 4, 1,
        5, 1, 2,    5, 2, 3,    5, 3, 4,    5, 4, 1
    };
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = SG_BUFFERTYPE_INDEXBUFFER;
    buffer_desc.size = sizeof(indices);
    buffer_desc.content = indices;
    buffer_desc.label = "geometry-indices";
    bind.index_buffer = sg_make_buffer(&buffer_desc);

    /* empty, dynamic instance-data vertex buffer, goes into vertex-buffer-slot 1 */
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = _SG_BUFFERTYPE_DEFAULT;
    buffer_desc.size = MAX_PARTICLES * sizeof(hmm_vec3);
    buffer_desc.usage = SG_USAGE_STREAM;
    buffer_desc.label = "instance-data";
    bind.vertex_buffers[1] = sg_make_buffer(&buffer_desc);

    /* a shader */
    sg_shader shd = sg_make_shader(instancing_shader_desc());

    /* a pipeline object */
    sg_pipeline_desc pipeline_desc;
    memset(&pipeline_desc, 0, sizeof(sg_pipeline_desc));
    /* vertex buffer at slot 1 must step per instance */
    pipeline_desc.layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE;
    pipeline_desc.layout.attrs[ATTR_vs_pos] = { 0,0, SG_VERTEXFORMAT_FLOAT3 }; // buffer_index, offset, format
    pipeline_desc.layout.attrs[ATTR_vs_color0] = { 0,0, SG_VERTEXFORMAT_FLOAT4 };
    pipeline_desc.layout.attrs[ATTR_vs_inst_pos] = { 1,0, SG_VERTEXFORMAT_FLOAT3 };
    pipeline_desc.shader = shd;
    pipeline_desc.index_type = SG_INDEXTYPE_UINT16;
    pipeline_desc.depth_stencil.depth_compare_func = SG_COMPAREFUNC_LESS_EQUAL;
    pipeline_desc.depth_stencil.depth_write_enabled = true;
    pipeline_desc.rasterizer.cull_mode = SG_CULLMODE_BACK;
    pipeline_desc.rasterizer.sample_count = sapp_sample_count();
    pipeline_desc.label = "instancing-pipeline";
    pip = sg_make_pipeline(&pipeline_desc);
}

std::once_flag init_particles_flag;

void draw_particles(float frame_time)
{
    std::call_once(init_particles_flag, []() { init_particles(); });

    /* emit new particles */
    for (int i = 0; i < NUM_PARTICLES_EMITTED_PER_FRAME; i++) {
        if (cur_num_particles < MAX_PARTICLES) {
            pos[cur_num_particles] = HMM_Vec3(0.0, 0.0, 0.0);
            vel[cur_num_particles] = HMM_Vec3(
                ((float)(rand() & 0x7FFF) / 0x7FFF) - 0.5f,
                ((float)(rand() & 0x7FFF) / 0x7FFF) * 0.5f + 2.0f,
                ((float)(rand() & 0x7FFF) / 0x7FFF) - 0.5f);
            cur_num_particles++;
        }
        else {
            break;
        }
    }
    
    /* update particle positions */
    for (int i = 0; i < cur_num_particles; i++) {
        vel[i].Y -= 1.0f * frame_time;
        pos[i].X += vel[i].X * frame_time;
        pos[i].Y += vel[i].Y * frame_time;
        pos[i].Z += vel[i].Z * frame_time;
        /* bounce back from 'ground' */
        if (pos[i].Y < -2.0f) {
            pos[i].Y = -1.8f;
            vel[i].Y = -vel[i].Y;
            vel[i].X *= 0.8f; vel[i].Y *= 0.8f; vel[i].Z *= 0.8f;
        }
    }

    /* update instance data */
    sg_update_buffer(bind.vertex_buffers[1], pos, cur_num_particles*sizeof(hmm_vec3));

    /* model-view-projection matrix */
    hmm_mat4 proj = HMM_Perspective(60.0f, (float)sapp_width()/(float)sapp_height(), 0.01f, 50.0f);
    hmm_mat4 view = HMM_LookAt(HMM_Vec3(0.0f, 1.5f, 12.0f), HMM_Vec3(0.0f, 0.0f, 0.0f), HMM_Vec3(0.0f, 1.0f, 0.0f));
    hmm_mat4 view_proj = HMM_MultiplyMat4(proj, view);
    //ry += 1.0f;
    vs_params_t vs_params;
    vs_params.mvp = HMM_MultiplyMat4(view_proj, HMM_Rotate(ry, HMM_Vec3(0.0f, 1.0f, 0.0f)));;

    /* ...and draw */
//    sg_begin_default_pass(&pass_action, sapp_width(), sapp_height());
    sg_apply_pipeline(pip);
    sg_apply_bindings(&bind);
    sg_apply_uniforms(SG_SHADERSTAGE_VS, SLOT_vs_params, &vs_params, sizeof(vs_params));
    sg_draw(0, 24, cur_num_particles);
//    sg_end_pass();
//    sg_commit();
}
