
#ifndef LABVFX_RENDER_H
#define LABVFX_RENDER_H

#include <LabVfx/LabVfx.h>
#include <memory>

namespace lab { namespace vfx {

struct RenderGraphicsData;

class Render {
public:
    Render(std::shared_ptr<Effect> p);
    ~Render();

    void render(const float* v_t, const float* mvp, float t, float dt);
    
private:
    RenderGraphicsData* _data;
};

}}

#endif
