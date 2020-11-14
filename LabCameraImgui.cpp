
/*
 Copyright (c) 2013 Nick Porcino, All rights reserved.
 License is MIT: http://opensource.org/licenses/MIT

 LabCameraImgui has no external dependencies besides LabCamera and Dear ImGui.
 Include LabCamera.cpp, and LabCameraImgui.cpp in your project.
*/


/*-----------------------------------------------------------------------------
     Demonstrate navigation via a virtual joystick hosted in a little window
 */

#include "LabCameraImgui.h"
#define TRACE_INTERACTION 0
#include "imgui.h"
#include "imgui_internal.h"
#include <stdio.h>

struct LCNav_MouseState
{
    float initial_mousex{ 0 };          // set when mouse transitions from not dragging to dragging
    float initial_mousey{ 0 };
    float mousex{ 0 };                  // current mouse position in window space
    float mousey{ 0 };
    bool  click_initiated{ false };     // true only on the frame when the mouse transitioned from not dragging to dragging
    bool  dragging{ false };            // true as long as the button is held
    bool  click_ended{ false };         // true only on the frame when the mouse transitioned from dragging to not dragging
};

struct LCNav_Panel : public LCNav_PanelState
{
    LCNav_Panel()
    {
        pan_tilt.set_speed(0.01f);
        home();
    }

    LCNav_PanelMode mode = LCNav_Mode_PanTilt;
    LCNav_MouseState mouse_state;
    const lab::camera::v2f size = { 256, 170 };
    const lab::camera::v2f trackball_size = { 128, 170 };
    float roll = 0;
    bool trackball_interacting = false;

    void home()
    {
        auto up = pan_tilt.world_up_constraint();
        pan_tilt.set_position_constraint({ 0.f, 0.2f, nav_radius });
        pan_tilt.set_orbit_center_constraint({ 0,0,0 });
    }
};

LCNav_PanelState* create_navigator_panel()
{
    return new LCNav_Panel();
}

void release_navigator_panel(LCNav_PanelState* p)
{
    LCNav_Panel* ptr = reinterpret_cast<LCNav_Panel*>(p);
    delete ptr;
}


void LCNav_mouse_state_update(LCNav_MouseState* ms,
    float mousex, float mousey, bool left_button_down)
{
    ms->click_ended = ms->dragging && !left_button_down;
    ms->click_initiated = !ms->dragging && left_button_down;
    ms->dragging = left_button_down;
    ms->mousex = mousex;
    ms->mousey = mousey;

    if (ms->click_initiated)
    {
        ms->initial_mousex = mousex;
        ms->initial_mousey = mousey;
    };

    if (TRACE_INTERACTION && ms->click_ended)
        printf("button released\n");
    if (TRACE_INTERACTION && ms->click_initiated)
        printf("button clicked\n");
}


bool update_mouseStatus_in_current_imgui_window(LCNav_MouseState* mouse)
{
    // assuming the 3d viewport is the current window, fetch the content region
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 imgui_cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorScreenPos(edit_rect.Min);

    // detect that the mouse is in the content region
    bool click_finished = ImGui::InvisibleButton("###NAVGIZMOREGION", edit_rect.GetSize());
    bool in_canvas = click_finished || ImGui::IsItemHovered();

    ImVec2 mouse_pos = io.MousePos - ImGui::GetCurrentWindow()->Pos - imgui_cursor_pos;
    LCNav_mouse_state_update(mouse, mouse_pos.x, mouse_pos.y, io.MouseDown[0] && io.MouseDownOwned[0]);

    // restore the ImGui state
    ImGui::SetCursorPos(imgui_cursor_pos);
    return in_canvas;
}


LabCameraNavigatorPanelInteraction
run_navigator_panel(LCNav_PanelState* navigator_panel_, lab::camera::Camera& camera)
{
    LCNav_Panel* navigator_panel = reinterpret_cast<LCNav_Panel*>(navigator_panel_);
    LabCameraNavigatorPanelInteraction result = LCNav_None;
    ImGui::SetNextWindowSize(ImVec2(navigator_panel->size.x, navigator_panel->size.y), ImGuiCond_FirstUseEver);
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - navigator_panel->size.x, 10));

    static bool visible = true;
    ImGui::Begin("Navigator###LCNav", &visible,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();

    ImGui::Columns(2, "###NavCol", true);
    ImGui::SetColumnWidth(0, navigator_panel->trackball_size.x);

    bool mouse_in_panel = update_mouseStatus_in_current_imgui_window(&navigator_panel->mouse_state);
    auto& ms = navigator_panel->mouse_state;

    //printf("%f %f\n", navigator_panel->mouse_state.mousex, navigator_panel->mouse_state.mousey);
    //printf("%f %f\n", cursor_screen_pos.x, cursor_screen_pos.y);
    //ImVec2 foo = ImGui::GetWindowContentRegionMin();
    //printf("%f %f\n", foo.x, foo.y);

    if (mouse_in_panel)
    {
        if (ms.click_initiated)
        {
            navigator_panel->trackball_interacting = true;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("CLICKED");
            result = LCNav_TumbleInitiated;
        }
        else if (ms.dragging)
        {
            navigator_panel->trackball_interacting = true;
            result = LCNav_TumbleContinued;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("DRAGGING");
        }
        else if (ms.click_ended)
        {
            result = LCNav_TumbleEnded;
            navigator_panel->trackball_interacting = false;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("RELEASED");
        }
        else 
        {
            result = LCNav_None;
            navigator_panel->trackball_interacting = false;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("HOVERED");
        }
    }
    else if (navigator_panel->trackball_interacting)
    {
        if (ms.dragging)
        {
            result = LCNav_TumbleContinued;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("DRAGGING OUTSIDE");
        }
        else if (ms.click_ended)
        {
            result = LCNav_TumbleEnded;
            navigator_panel->trackball_interacting = false;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("RELEASED OUTSIDE");
        }
    }
    else
    {
        navigator_panel->trackball_interacting = false;
        ImGui::SetCursorScreenPos(cursor_screen_pos);
        ImGui::TextUnformatted("IDLE");
    }

    ImGui::NextColumn();

    if (ImGui::Button("Home###NavHome")) {
        auto up = navigator_panel->pan_tilt.world_up_constraint();
        navigator_panel->pan_tilt.set_position_constraint({ 0.f, 0.2f, navigator_panel->nav_radius });
        navigator_panel->pan_tilt.set_orbit_center_constraint({ 0,0,0 });
        camera.mount.look_at(navigator_panel->pan_tilt.position_constraint(),
            navigator_panel->pan_tilt.orbit_center_constraint(),
            navigator_panel->pan_tilt.world_up_constraint());
        result = LCNav_None;
    }
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Crane ? "-Crane-" : " Crane ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::Crane;
        result = LCNav_ModeChange;
    }
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Dolly ? "-Dolly-" : " Dolly ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::Dolly;
        result = LCNav_ModeChange;
    }
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::TurnTableOrbit ? "-Orbit-" : " Orbit ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::TurnTableOrbit;
        result = LCNav_ModeChange;
    }
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Gimbal ? "-Gimbal-" : " Gimbal ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::Gimbal;
        result = LCNav_ModeChange;
    }
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Arcball ? "-Arcball-" : " Arcball ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::Arcball;
        result = LCNav_ModeChange;
    }

    static float zoom = 0.f;
    if (ImGui::SliderAngle("###Dutch", &zoom, -180.f, 180.f, "%.0f"))
    {
        navigator_panel->roll = zoom;
        result = LCNav_RollUpdated;
    }
    else
        zoom = 0.f;     // mouse released, reset

    // end of columns.

    ImGui::End();

    if (navigator_panel->trackball_interacting)
    {
        lab::camera::InteractionPhase phase = lab::camera::InteractionPhase::Continue;
        if (ms.click_initiated)
            phase = lab::camera::InteractionPhase::Start;
        else if (ms.click_ended)
            phase = lab::camera::InteractionPhase::Finish;

        switch (result)
        {
        case LCNav_RollUpdated:
        {
            lab::camera::v3f ypr = camera.mount.ypr();
            lab::camera::v3f pos = camera.mount.position();
            ypr.z = navigator_panel->roll;
            camera.mount.set_view_transform_ypr_pos(ypr, pos);
        }
        break;

        case LCNav_TumbleInitiated:
            ImGui::CaptureMouseFromApp(true);
        case LCNav_TumbleEnded:
        case LCNav_TumbleContinued:
        {
            if (navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Arcball)
            {
                lab::camera::InteractionToken tok = navigator_panel->pan_tilt.begin_interaction(navigator_panel->trackball_size);
                navigator_panel->pan_tilt.ttl_interaction(
                    camera, 
                    tok, phase, navigator_panel->camera_interaction_mode, 
                    { navigator_panel->mouse_state.mousex, navigator_panel->mouse_state.mousey });
                navigator_panel->pan_tilt.end_interaction(tok);
            }
            else
            {
                const float speed_scaler = 10.f;
                float scale = speed_scaler / navigator_panel->trackball_size.x;
                float dx = (navigator_panel->mouse_state.mousex - navigator_panel->mouse_state.initial_mousex) * scale;
                float dy = (navigator_panel->mouse_state.mousey - navigator_panel->mouse_state.initial_mousey) * -scale;
                lab::camera::InteractionToken tok = navigator_panel->pan_tilt.begin_interaction(navigator_panel->trackball_size);
                navigator_panel->pan_tilt.joystick_interaction(
                    camera, 
                    tok, phase, navigator_panel->camera_interaction_mode, { dx, dy });
                navigator_panel->pan_tilt.end_interaction(tok);
            }
        }
        break;

        default:
            break;
        }
    }

    return result;
}
