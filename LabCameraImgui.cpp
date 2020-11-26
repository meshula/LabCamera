
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
#include <algorithm>
#include <stdio.h>


static bool DialControl(const char* label, float* p_value)
{
    ImGuiIO& io = ImGui::GetIO();
    ImGuiStyle& style = ImGui::GetStyle();

    float radius_outer = 20.0f;
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImVec2 center = ImVec2(pos.x + radius_outer, pos.y + radius_outer);
    float line_height = ImGui::GetTextLineHeight();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    ImGui::InvisibleButton(label, ImVec2(radius_outer * 2, radius_outer * 2 + line_height + style.ItemInnerSpacing.y));

    bool value_changed = false;
    bool is_active = ImGui::IsItemActive();
    bool is_hovered = ImGui::IsItemActive();
    if (is_active && (io.MouseDelta.x != 0.f || io.MouseDelta.y != 0.f))
    {
        ImVec2 mouse_pos = io.MousePos - center;
        *p_value = atan2f(mouse_pos.x, -mouse_pos.y);
        value_changed = true;
    }

    float angle_cos = cosf(*p_value - 3.141592f * 0.5f), angle_sin = sinf(*p_value - 3.141592f * 0.5f);
    float radius_inner = radius_outer * 0.40f;
    draw_list->AddCircleFilled(center, radius_outer, ImGui::GetColorU32(ImGuiCol_FrameBg), 16);
    draw_list->AddLine(ImVec2(center.x + angle_cos * radius_inner, center.y + angle_sin * radius_inner), 
                       ImVec2(center.x + angle_cos * (radius_outer - 2), center.y + angle_sin * (radius_outer - 2)), 
                       ImGui::GetColorU32(ImGuiCol_SliderGrabActive), 2.0f);
    draw_list->AddCircleFilled(center, radius_inner, ImGui::GetColorU32(is_active ? ImGuiCol_FrameBgActive : is_hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), 16);
    draw_list->AddText(ImVec2(pos.x, pos.y + radius_outer * 2 + style.ItemInnerSpacing.y), ImGui::GetColorU32(ImGuiCol_Text), label);

    if (is_active || is_hovered)
    {
        ImGui::SetNextWindowPos(ImVec2(pos.x - style.WindowPadding.x, pos.y - line_height - style.ItemInnerSpacing.y - style.WindowPadding.y));
        ImGui::BeginTooltip();
        ImGui::Text("%d", (int) (*p_value * 180.f / 3.141592f));
        ImGui::EndTooltip();
    }

    return value_changed;
}


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
    }

    LCNav_PanelMode mode = LCNav_Mode_PanTilt;
    LCNav_MouseState mouse_state;
    const lab::camera::v2f size = { 256, 256 };
    const lab::camera::v2f trackball_size = { 128, 128 };
    float roll = 0;
    bool trackball_interacting = false;
    bool roll_interacting = false;
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


bool update_mouseStatus_in_current_imgui_window(LCNav_MouseState* mouse, const ImVec2& sz)
{
    // assuming the 3d viewport is the current window, fetch the content region
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 imgui_cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorScreenPos(edit_rect.Min);

    // detect that the mouse is in the content region
    bool click_finished = ImGui::InvisibleButton("###NAVGIZMOREGION", sz);
    bool in_canvas = click_finished || ImGui::IsItemHovered();

    ImVec2 mouse_pos = io.MousePos - ImGui::GetCurrentWindow()->Pos - imgui_cursor_pos;
    LCNav_mouse_state_update(mouse, mouse_pos.x, mouse_pos.y, io.MouseDown[0] && io.MouseDownOwned[0]);
    return in_canvas;
}


LabCameraNavigatorPanelInteraction
run_navigator_panel(LCNav_PanelState* navigator_panel_, lab::camera::Camera& camera, float dt)
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

    bool mouse_in_panel = update_mouseStatus_in_current_imgui_window(&navigator_panel->mouse_state,
        { navigator_panel->trackball_size.x, navigator_panel->trackball_size.y });
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

    ImGui::SetCursorScreenPos({ cursor_screen_pos.x, cursor_screen_pos.y + navigator_panel->trackball_size.y });

    lab::camera::v3f ypr = camera.mount.ypr();
    static float roll = 0.f;
    if (DialControl("Roll", &roll))
    {
#if 0
        if (!navigator_panel->trackball_interacting)
            result = LCNav_TumbleInitiated;
        else
            result = LCNav_TumbleContinued;

        navigator_panel->roll_interacting = true;
        navigator_panel->roll = roll;
        lab::camera::InteractionToken tok = navigator_panel->pan_tilt.begin_interaction(navigator_panel->trackball_size);
        navigator_panel->pan_tilt.set_roll(camera, tok, lab::camera::radians{ roll });
        navigator_panel->pan_tilt.end_interaction(tok);

        navigator_panel->trackball_interacting = false;
#endif
    }
    else
    {
#if 0
        if (navigator_panel->trackball_interacting)
            result = LCNav_TumbleEnded;
        navigator_panel->roll_interacting = false;
        roll = ypr.z;
#endif
    }

    ImGui::NextColumn();

    if (ImGui::Button("Home###NavHome")) {
        auto up = navigator_panel->pan_tilt.world_up_constraint();
        camera.mount.look_at({ 0.f, 0.2f, navigator_panel->nav_radius }, { 0, 0, 0 }, navigator_panel->pan_tilt.world_up_constraint());
        navigator_panel->pan_tilt.set_orbit_center_constraint({ 0,0,0 });
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
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::PanTilt ? "-PanTilt-" : " PanTilt ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::PanTilt;
        result = LCNav_ModeChange;
    }
    if (ImGui::Button(navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Arcball ? "-Arcball-" : " Arcball ")) {
        navigator_panel->camera_interaction_mode = lab::camera::InteractionMode::Arcball;
        result = LCNav_ModeChange;
    }


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

        case LCNav_TumbleInitiated:
            ImGui::CaptureMouseFromApp(true);
        case LCNav_TumbleEnded:
            [[fallthrough]];
        case LCNav_TumbleContinued:
        {
            if (navigator_panel->camera_interaction_mode == lab::camera::InteractionMode::Arcball)
            {
                lab::camera::InteractionToken tok = navigator_panel->pan_tilt.begin_interaction(navigator_panel->trackball_size);
                navigator_panel->pan_tilt.ttl_interaction(
                    camera, 
                    tok, phase, navigator_panel->camera_interaction_mode, 
                    { navigator_panel->mouse_state.mousex, navigator_panel->mouse_state.mousey }, dt);
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
                    tok, phase, navigator_panel->camera_interaction_mode, { dx, dy }, dt);
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

static float len(float x, float y)
{
    return sqrtf(x * x + y * y);
}

void FX_minimap(ImDrawList* d, ImVec2 a, ImVec2 b, ImVec2 sz, ImVec4 mouse, float t, const lab::camera::rigid_transform* cam, const lab::camera::v3f lookat)
{
    using lab::camera::v3f;
    float min_dim = std::min(sz.x, sz.y);
    v3f eye = cam->position;
    v3f relative_eye = { eye.x - lookat.x, eye.y - lookat.y, eye.z - lookat.z };
    float scale = 1.f / 10.f;
    ImVec2 p_eye = { relative_eye.x, relative_eye.z };
    p_eye *= scale;
    p_eye *= min_dim * 0.5f;
    p_eye += a;
    p_eye += sz * 0.5f;

    ImVec2 p_lookat = a + sz * 0.5f;
    d->AddRectFilled(p_lookat - ImVec2{ 4,4 }, p_lookat + ImVec2{ 4,4 }, 0xffffd050);
    d->AddRectFilled(p_eye - ImVec2{ 4,4 }, p_eye + ImVec2{ 4,4 }, 0xffffd050);
}

// this is a canonical way to create an ImGui rendering window with an invisible button
// to extract a drawable region covering the whole window.
// https://gist.github.com/ocornut/51367cc7dfd2c41d607bb0acfa6caf66
//
void camera_minimap(int w, int h, const lab::camera::rigid_transform* cam, const lab::camera::v3f lookat)
{
    ImGuiIO& io = ImGui::GetIO();
    ImGui::Begin("Camera Minimap", NULL, ImGuiWindowFlags_AlwaysAutoResize);
    ImVec2 size{ float(w), float(h) };
    ImGui::InvisibleButton("cm_canvas", size);
    ImVec2 p0 = ImGui::GetItemRectMin();
    ImVec2 p1 = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->PushClipRect(p0, p1);

    ImVec4 mouse_data;
    mouse_data.x = (io.MousePos.x - p0.x) / size.x;
    mouse_data.y = (io.MousePos.y - p0.y) / size.y;
    mouse_data.z = io.MouseDownDuration[0];
    mouse_data.w = io.MouseDownDuration[1];

    {
        // draw here
        //FX_glass(draw_list, p0, p1, size, mouse_data, (float)ImGui::GetTime());
        FX_minimap(draw_list, p0, p1, size, mouse_data, (float)ImGui::GetTime(), cam, lookat);
    }

    draw_list->PopClipRect();
    ImGui::End();
}

