
#include "LabVfx/LabVfx.h"
#include "LabVfxRender.h"

std::shared_ptr<lab::vfx::Effect> effect;
std::shared_ptr<lab::vfx::ForceIntegrator> integrator;
std::shared_ptr<lab::vfx::Emitter> emitter;
std::shared_ptr<lab::vfx::AccelerationField> acc;
std::shared_ptr<lab::vfx::Zero> zero;
std::shared_ptr<lab::vfx::Render> render;
std::shared_ptr<lab::vfx::AgeRule> life;

void simple_effect(const float* v_t, const float* mvp, float t, float dt)
{
    static bool once = true;
    if (once)
    {
        once = false;

        effect = std::make_shared<lab::vfx::Effect>();
        effect->dataStripes()->addStripe(lab::vfx::DataStripe::kFloat32_3, "pos");
        effect->dataStripes()->addStripe(lab::vfx::DataStripe::kFloat32_3, "vel");
        effect->dataStripes()->addStripe(lab::vfx::DataStripe::kFloat32_3, "facc");
        effect->dataStripes()->addStripe(lab::vfx::DataStripe::kFloat32_2, "life");
        integrator = std::make_shared<lab::vfx::ForceIntegrator>(effect->dataStripes());
        emitter = std::make_shared<lab::vfx::Emitter>();

        // acc zeroes out the force accumulator, first parameter
        acc = std::make_shared<lab::vfx::AccelerationField>(effect->dataStripes(), true);
        acc->setDirection(0, 0, 1);
        acc->setForceOutOData(effect->dataStripes()->get("facc"));

        integrator->setForceIData(effect->dataStripes()->get("facc"));
        integrator->setPositionIOData(effect->dataStripes()->get("pos"));
        integrator->setVelocityIOData(effect->dataStripes()->get("vel"));

        life = std::make_shared<lab::vfx::AgeRule>(effect->dataStripes());
        life->setAgeIOData(effect->dataStripes()->get("life"));

        effect->addOperator(acc);
        effect->addIntegrator(integrator);
        effect->addEmitter(emitter);
        effect->addLifetimeRule(life);
        
        effect->dataStripes()->resize(64);

        render = std::make_shared<lab::vfx::Render>(effect);
    }

    effect->update(t, dt);
    render->render(v_t, mvp, t, dt);
}
