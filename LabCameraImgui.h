
/*
 Copyright (c) 2020 Nick Porcino, All rights reserved.
 License is MIT: http://opensource.org/licenses/MIT

 LabCameraImgui has no external dependencies besides LabCamera and Dear ImGui.
 Include LabCamera.cpp, and LabCameraImgui.cpp in your project.
*/

#ifndef LABCAMERAIMGUI_H
#define LABCAMERAIMGUI_H

#include "LabCamera.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LCNav_Inactive = 0,
    LCNav_ModeChange,
    LCNav_RollInitiated, LCNav_RollContinued, LCNav_RollEnded,
    LCNav_TumbleInitiated, LCNav_TumbleContinued, LCNav_TumbleEnded
} LabCameraNavigatorPanelInteraction;

typedef enum
{
    LCNav_Mode_PanTilt, LCNav_Mode_Arcball
} LCNav_PanelMode;


/*
 * The navigator panel takes a viewport, and a camera to mutate.
 * The viewport should be the viewport size in device coordinates.
 */

struct LCNav_Panel;

LabCameraNavigatorPanelInteraction
run_navigator_panel(LCNav_Panel* navigator_panel, lc_camera& camera, float dt);

LCNav_Panel* create_navigator_panel();
void release_navigator_panel(LCNav_Panel*);

lc_interaction* LCNav_Panel_interaction_controller(const LCNav_Panel*);
lc_i_Mode LCNav_Panel_interaction_mode(const LCNav_Panel*);
lc_radians LCNav_Panel_roll(const LCNav_Panel*);

/*
 * minimap is not ready for primetime
 */

void camera_minimap(int w, int h, const lc_rigid_transform* cam, const lc_v3f lookat);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
