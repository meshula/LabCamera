
#include "LabVfxRender.h"

#include "sokol_app.h"
#include "sokol_gfx.h"
#define HANDMADE_MATH_NO_SSE
#include "HandmadeMath.h"
#include "instancing_glsl.h"
#include "billboard_instances_glsl.h"

namespace lab { namespace vfx {

struct BoundObject
{
    BoundObject() 
    {
        memset(&pip, 0, sizeof(pip));
        memset(&pass_action, 0, sizeof(pass_action));
        memset(&bind, 0, sizeof(bind));

        pass_action.colors[0] = { SG_ACTION_CLEAR, { 0.0f, 0.0f, 0.0f, 1.0f } };
    }

    sg_pipeline pip;
    sg_pass_action pass_action;
    sg_bindings bind;
};

struct RenderGraphicsData
{
    RenderGraphicsData(std::shared_ptr<Effect> effect);
    void render(const float* v_t, const float* mvp, float t, float dt);
    std::shared_ptr<Effect> _effect;

    BoundObject _diamond;
};

void CreateDiamond(BoundObject& o, size_t max_instance_count)
{
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
    buffer_desc.label = "diamond-vertices";
    o.bind.vertex_buffers[0] = sg_make_buffer(&buffer_desc);

    /* index buffer for static geometry */
    const uint16_t indices[] = {
        0, 1, 2,    0, 2, 3,    0, 3, 4,    0, 4, 1,
        5, 1, 2,    5, 2, 3,    5, 3, 4,    5, 4, 1
    };
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = SG_BUFFERTYPE_INDEXBUFFER;
    buffer_desc.size = sizeof(indices);
    buffer_desc.content = indices;
    buffer_desc.label = "diamond-indices";
    o.bind.index_buffer = sg_make_buffer(&buffer_desc);

    /* empty, dynamic instance-data vertex buffer, goes into vertex-buffer-slot 1 */
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = _SG_BUFFERTYPE_DEFAULT;
    buffer_desc.size = static_cast<int>(max_instance_count * sizeof(hmm_vec3));
    buffer_desc.usage = SG_USAGE_STREAM;
    buffer_desc.label = "diamond-instance";
    o.bind.vertex_buffers[1] = sg_make_buffer(&buffer_desc);

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
    pipeline_desc.label = "labvfx_render-pipeline";
    o.pip = sg_make_pipeline(&pipeline_desc);
}


void CreateQuad(BoundObject& o, size_t max_instance_count)
{
    /* vertex buffer for static geometry, goes into vertex-buffer-slot 0 */
    const float r = 0.1f; // @TODO should be radius input
    const float vertices[] = {

        // positions            colors
          -r,   -r, 0.0f,       1.0f, 0.0f, 0.0f, 1.0f,
          -r,    r, 0.0f,       0.0f, 1.0f, 0.0f, 1.0f,
           r,    r, 0.0f,       0.0f, 0.0f, 1.0f, 1.0f,
           r,   -r, 0.0f,       1.0f, 1.0f, 0.0f, 1.0f,
    };

    sg_buffer_desc buffer_desc;
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = _SG_BUFFERTYPE_DEFAULT;
    buffer_desc.size = sizeof(vertices);
    buffer_desc.content = vertices;
    buffer_desc.label = "quad-vertices";
    o.bind.vertex_buffers[0] = sg_make_buffer(&buffer_desc);

    /* index buffer for static geometry */
    const uint16_t indices[] = {
        0, 1, 2,    0, 2, 3
    };
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = SG_BUFFERTYPE_INDEXBUFFER;
    buffer_desc.size = sizeof(indices);
    buffer_desc.content = indices;
    buffer_desc.label = "geometry-indices";
    o.bind.index_buffer = sg_make_buffer(&buffer_desc);

    /* empty, dynamic instance-data vertex buffer, goes into vertex-buffer-slot 1 */
    memset(&buffer_desc, 0, sizeof(sg_buffer_desc));
    buffer_desc.type = _SG_BUFFERTYPE_DEFAULT;
    buffer_desc.size = static_cast<int>(max_instance_count * sizeof(hmm_vec3));
    buffer_desc.usage = SG_USAGE_STREAM;
    buffer_desc.label = "geometry-instance";
    o.bind.vertex_buffers[1] = sg_make_buffer(&buffer_desc);

    /* a shader */
    sg_shader shd = sg_make_shader(billboard_instances_shader_desc());

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
    pipeline_desc.label = "labvfx_render-pipeline";
    o.pip = sg_make_pipeline(&pipeline_desc);
}

RenderGraphicsData::RenderGraphicsData(std::shared_ptr<Effect> effect)
: _effect(effect)
{
    CreateQuad(_diamond, _effect->dataStripes()->stripe_capacity());
}

Render::Render(std::shared_ptr<Effect> p)
{
    _data = new RenderGraphicsData(p); 
}
Render::~Render()
{
    delete _data;
}

void RenderGraphicsData::render(const float* v_t, const float* mvp, float t, float dt)
{
    /* update instance data */
    auto pos = _effect->dataStripes()->get("pos");
    float* pos_data = pos->data<float>();
    size_t cur_num_particles = _effect->dataStripes()->activeCount();
    if (!cur_num_particles)
        return;

    /* special case when the buffer is full */
    if (cur_num_particles == _effect->dataStripes()->stripe_capacity())
        sg_update_buffer(_diamond.bind.vertex_buffers[1], pos_data, static_cast<int>(cur_num_particles * sizeof(float)*3));
    else {
        /* prepare a buffer with just the active particles */
        std::vector<float> pos_buff;
        pos_buff.reserve(cur_num_particles);
        auto& reindex = _effect->dataStripes()->redirect();
        size_t stride = pos->stride() / sizeof(float);
        for (size_t i = 0; i < cur_num_particles; ++i) {
            int index = reindex[i] * stride;
            pos_buff.push_back(pos_data[index]);
            pos_buff.push_back(pos_data[index+1]);
            pos_buff.push_back(pos_data[index+2]);
        }
        sg_update_buffer(_diamond.bind.vertex_buffers[1], &pos_buff[0], static_cast<int>(cur_num_particles * sizeof(float)*3));
    }

    billboard_instances_vs_params_t vs_params;
    vs_params.mvp = *((hmm_mat4*) mvp);
    vs_params.v_t = *((hmm_mat4*) v_t);

    /* ...and draw */
    sg_apply_pipeline(_diamond.pip);
    sg_apply_bindings(&_diamond.bind);
    sg_apply_uniforms(SG_SHADERSTAGE_VS, SLOT_vs_params, &vs_params, sizeof(vs_params));
    sg_draw(0, 24, cur_num_particles);
}
 
void Render::render(const float* v_t, const float* mvp, float t, float dt)
{
    _data->render(v_t, mvp, t, dt);
}

}}

#if 0

#include "Particles.h"
#include "gl4.h"

#include <iostream>

struct Temp {
    Temp() : geomData(logThrowOnError), needInit(true), valid(false) { }
    
    Buffer<vec3> verts;
    VAO geomData;
    Shader pointShader;
    bool needInit;
    bool valid;
    
    void init() {
        try {
            valid = false;
            needInit = false;
            pointShader.vertexShader(glsl(
                uniform mat4 mvp;
                in vec3 vertex;
                void main() {
                    gl_Position = mvp * vec4(vertex, 1);
                }
            )).fragmentShader(glsl(
                out vec4 fragColor;
                void main() {
                    fragColor = vec4(1,1,1,1);
                }
            )).link();
            verts.upload();
            geomData.create(pointShader, verts).attribute<float>("vertex", 3).check();
            valid = true;
        }
        catch(std::exception& exc) {
            std::cout << exc.what() << std::endl;
        }
    }
};

Temp temp;

void Render::render(const float* mvp) {
    if (!_particles)
        return;
    
    return;
    
    const DataStripes& stripes = _particles->dataStripes();
    int count = stripes.activeCount();
    if (!count)
        return;
    
    if (temp.needInit)
        temp.init();
    
    if (!temp.valid)
        return;

    const std::vector<int>& redirect = _particles->dataStripes().redirect();

    std::shared_ptr<DataStripe> pos = _particles->dataStripes().get("pos");
    float* data = pos->data<float>(DataStripe::kFloat32_3);
    int stride = pos->stride();
    if (!data)
        return;
    
    // todo pack data interleaved
    // cf. http://developer.apple.com/library/ios/#DOCUMENTATION/3DDrawing/Conceptual/OpenGLES_ProgrammingGuide/TechniquesforWorkingwithVertexData/TechniquesforWorkingwithVertexData.html

    temp.verts.data.clear();
    temp.verts.data.reserve(count);
    for (int i = 0; i < count; ++i) {
        int index = redirect[i];
        float* p = &data[index * stride];
        temp.verts << vec3(p[0], p[1], p[2]);
    }
    
    // todo stream buffer should be double buffered
    // cf. http://developer.apple.com/library/ios/#DOCUMENTATION/3DDrawing/Conceptual/OpenGLES_ProgrammingGuide/OpenGLESApplicationDesign/OpenGLESApplicationDesign.html#//apple_ref/doc/uid/TP40008793-CH6-SW5

    temp.verts.upload(GL_ARRAY_BUFFER, GL_STREAM_DRAW);
    if (temp.needInit)
        temp.init();
    
    temp.pointShader.use();
    temp.pointShader.uniform("mvp", *(mat4*)mvp);
    temp.geomData.draw(GL_POINTS);
    temp.pointShader.unuse();
}

#endif
