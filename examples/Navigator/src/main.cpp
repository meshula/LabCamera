
/*-----------------------------------------------------------------------------
    Demonstrating the use of lab::Camera with ImGui and sokol.
 */

#include "LabCamera.h"

#define SOKOL_TRACE_HOOKS
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_time.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_imgui.h"
#include "sokol_gfx_imgui.h"
#include "sokol_time.h"
#include "sokol_glue.h"
#include "sokol_gl.h"
#include "gizmo.h"
#include <tiny-gizmo.hpp>
#include <mutex>
#include <vector>


#define TRACE_INTERACTION 0

/*-----------------------------------------------------------------------------
    Random utility
 */

static bool intersect_ray_plane(const lab::camera::Ray& ray, const lab::camera::v3f& point, const lab::camera::v3f& normal,
    lab::camera::v3f* intersection = nullptr, float* outT = nullptr)
{
    const float PLANE_EPSILON = 0.001f;
    const float d = ray.dir.x * normal.x + ray.dir.y * normal.y + ray.dir.z * normal.z;

    // Make sure we're not parallel to the plane
    if (std::abs(d) > PLANE_EPSILON)
    {
        float w = normal.x * point.x + normal.y * point.y + normal.z * point.z;
        w = -w;

        float distance = ray.pos.x * normal.x + ray.pos.y * normal.y + ray.pos.z * normal.z + w;
        float t = -distance / d;

        if (t >= PLANE_EPSILON)
        {
            if (outT) *outT = t;
            if (intersection)
            {
                lab::camera::v3f result = ray.pos;
                result.x += t * ray.dir.x;
                result.y += t * ray.dir.y;
                result.z += t * ray.dir.z;
                *intersection = result;
            }
            return true;
        }
    }
    if (outT) *outT = std::numeric_limits<float>::max();
    return false;
}

/*-----------------------------------------------------------------------------
    Application State
 */

enum class UIStateMachine
{
    None = 0, UI, Gizmo, DeltaCamera, TTLCamera
};

const char* name_state(UIStateMachine s)
{
    switch (s)
    {
    case UIStateMachine::None: return "None";
    case UIStateMachine::UI: return "UI";
    case UIStateMachine::Gizmo: return "Gizmo";
    case UIStateMachine::DeltaCamera: return "DeltaCamera";
    case UIStateMachine::TTLCamera: return "TTLCamera";
    }
    return "";
}

struct AppState
{
    std::string g_app_path;

    lab::camera::Camera camera;
    lab::camera::v2f initial_mouse_position = { 0, 0 };
    lab::camera::v3f initial_hit_point;
    UIStateMachine ui_state = UIStateMachine::UI;

    tinygizmo::m44f gizmo_transform;
    tinygizmo::gizmo_application_state gizmo_state;
    tinygizmo::gizmo_context gizmo_ctx;

    uint64_t last_time = 0;

    bool show_navigator = true;
    bool show_look_at = true;
    bool show_state = true;
    bool show_view_plane_intersect = false;
    bool show_manip_plane_intersect = false;
    bool quit = false;

    sgl_pipeline gl_pipelne;
    sg_imgui_t sg_imgui;
    sg_pass_action pass_action;
} gApp;

struct MouseState
{
    bool click_ended{ false };
    bool click_initiated{ false };
    bool in_canvas{ false };
    bool dragging_node{ false };
    bool resizing_node{ false };
    bool dragging{ false };
    ImVec2 mouse_ws{ 0, 0 };
    ImVec2 initial_click_pos_ws{ 0, 0 };
} mouse;


bool update_mouseStatus_in_viewport(ImVec2 canvas_offset)
{
    // assuming the 3d viewport is the current window, fetch the content region
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;
    ImGuiIO& io = ImGui::GetIO();
    float width = ImGui::GetContentRegionAvailWidth();
    float height = ImGui::GetContentRegionAvail().y;

    // detect that the mouse is in the content region
    ImVec2 imgui_cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorScreenPos(edit_rect.Min);
    bool click_finished = ImGui::InvisibleButton("###GIZMOREGION", edit_rect.GetSize());
    mouse.in_canvas = click_finished || ImGui::IsItemHovered();

    //---------------------------------------------------------------------
    // determine hovered, dragging, pressed, and released, as well as
    // window local coordinate and canvas local coordinate
    //
    mouse.click_ended = false;
    mouse.click_initiated = false;
    if (mouse.in_canvas)
    {
        if (click_finished)
        {
            if (TRACE_INTERACTION)
                printf("button released\n");

            mouse.dragging_node = false;
            mouse.resizing_node = false;
            mouse.click_ended = true;
        }

        mouse.mouse_ws = io.MousePos - ImGui::GetCurrentWindow()->Pos;

        if (io.MouseDown[0] && io.MouseDownOwned[0])
        {
            if (!mouse.dragging)
            {
                if (TRACE_INTERACTION)
                    printf("button clicked\n");

                mouse.click_initiated = true;
                mouse.initial_click_pos_ws = io.MousePos;
                mouse.dragging = true;
            }
        }
        else
            mouse.dragging = false;
    }
    else
        mouse.dragging = false;

    // restore the ImGui state
    ImGui::SetCursorPos(imgui_cursor_pos);
    return mouse.in_canvas;
}

/*-----------------------------------------------------------------------------
    Miscellaneous graphics helpers
 */

static void start_gl_rendering()
{
    static bool once = true;
    if (once) {
        /* setup sokol-gl */
        sgl_desc_t sgl_desc;
        memset(&sgl_desc, 0, sizeof(sgl_desc_t));
        sgl_desc.sample_count = sapp_sample_count();
        sgl_setup(&sgl_desc);

        /* a pipeline object with less-equal depth-testing */
        sg_pipeline_desc sg_p;
        memset(&sg_p, 0, sizeof(sg_pipeline_desc));
        sg_p.depth_stencil.depth_write_enabled = true;
        sg_p.depth_stencil.depth_compare_func = SG_COMPAREFUNC_LESS_EQUAL;
        gApp.gl_pipelne = sgl_make_pipeline(&sg_p);

        once = false;
    }

    sgl_defaults();
    sgl_push_pipeline();
    sgl_load_pipeline(gApp.gl_pipelne);
}

static void end_gl_rendering()
{
    sgl_pop_pipeline();
    sgl_draw();
}

static void draw_grid(float y, const lab::camera::m44f& m)
{
    sgl_matrix_mode_projection();

    lab::camera::m44f proj = gApp.camera.perspective();
    sgl_load_matrix(&proj.x.x);

    lab::camera::m44f view = gApp.camera.mount.view_transform();
    lab::camera::m44f mv = gApp.camera.mount.model_view_transform(m);

    sgl_matrix_mode_modelview();
    sgl_load_matrix(&mv.x.x);

    sgl_c3f(1.0f, 0.0f, 1.0f);

    const float num = 32;
    const float step = 1.0f;
    sgl_begin_lines();
    for (float x = -num; x < num; x += step) {
        sgl_v3f(x, y, -num * step);
        sgl_v3f(x, y,  num * step);
    }
    for (float z = -num; z < num; z += step) {
        sgl_v3f(-num * step, y, z);
        sgl_v3f( num * step, y, z);
    }
    sgl_end();
}

static void draw_jack(float s, const lab::camera::m44f& m)
{
    lab::camera::m44f proj = gApp.camera.perspective();
    sgl_matrix_mode_projection();
    sgl_load_matrix(&proj.x.x);

    gApp.camera.mount.foo();
    lab::camera::m44f mv = gApp.camera.mount.model_view_transform(&m.x.x);
    sgl_matrix_mode_modelview();
    sgl_load_matrix(&mv.x.x);

    sgl_begin_lines();
    sgl_c3f( 1,  0,  0);
    sgl_v3f(-s,  0,  0);
    sgl_v3f( s,  0,  0);
    sgl_c3f( 0,  1,  0);
    sgl_v3f( 0, -s,  0);
    sgl_v3f( 0,  s,  0);
    sgl_c3f( 0,  0,  1);
    sgl_v3f( 0,  0, -s);
    sgl_v3f( 0,  0,  s);
    sgl_end();
}

void initialize_graphics()
{
    // setup sokol-gfx, sokol-time and sokol-imgui
    sg_desc desc = { };
    desc.context = sapp_sgcontext();
    sg_setup(&desc);
    stm_setup();

    // setup debug inspection header(s)
    sg_imgui_init(&gApp.sg_imgui);

    // setup sokol-imgui, but provide our own font
    simgui_desc_t simgui_desc = { };
    simgui_desc.no_default_font = true;
    simgui_desc.sample_count = sapp_sample_count();
    simgui_desc.dpi_scale = sapp_dpi_scale();
    simgui_setup(&simgui_desc);

    // initial clear color
    gApp.pass_action.colors[0].action = SG_ACTION_CLEAR;
    gApp.pass_action.colors[0].val[0] = 0.0f;
    gApp.pass_action.colors[0].val[1] = 0.5f;
    gApp.pass_action.colors[0].val[2] = 0.7f;
    gApp.pass_action.colors[0].val[3] = 1.0f;

    uint8_t* data = nullptr;
    int32_t width = 0;
    int32_t height = 0;

    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->GetTexDataAsRGBA32(&data, &width, &height);

    // Upload new font texture atlas
    unsigned char* font_pixels;
    int font_width, font_height;
    io.Fonts->GetTexDataAsRGBA32(&font_pixels, &font_width, &font_height);
    sg_image_desc img_desc = { };
    img_desc.width = font_width;
    img_desc.height = font_height;
    img_desc.pixel_format = SG_PIXELFORMAT_RGBA8;
    img_desc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
    img_desc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
    img_desc.min_filter = SG_FILTER_LINEAR;
    img_desc.mag_filter = SG_FILTER_LINEAR;
    img_desc.content.subimage[0][0].ptr = font_pixels;
    img_desc.content.subimage[0][0].size = font_width * font_height * 4;
    io.Fonts->TexID = (ImTextureID)(uintptr_t)sg_make_image(&img_desc).id;
}

void shutdown_graphics() {
    simgui_shutdown();
    sg_imgui_discard(&gApp.sg_imgui);
    sgl_shutdown();
    sg_shutdown();
}


/*-----------------------------------------------------------------------------
    Run the gizmo which is tied to a virtual plane to help exercise various
    ray-intersection methods in lab::Camera
 */

struct GizmoTriangles
{
    GizmoTriangles()
    {
        indices.push_back(0);
        vertices.push_back(0);
    }
    int triangle_count = 0;
    std::vector<uint32_t> indices;
    std::vector<float> vertices;
};
static GizmoTriangles gizmo_triangles;

// return true if the gizmo was interacted
bool run_gizmo(float mouse_wsx, float mouse_wsy, float width, float height)
{
    lab::camera::v3f camera_pos = gApp.camera.mount.position();
    lab::camera::Ray ray = gApp.camera.get_ray_from_pixel({ mouse_wsx, mouse_wsy }, { 0, 0 }, { width, height });
    lab::camera::quatf camera_orientation = gApp.camera.mount.rotation();

    gApp.gizmo_state.mouse_left = mouse.click_initiated || mouse.dragging;
    gApp.gizmo_state.viewport_size = tinygizmo::v2f{ width, height };
    gApp.gizmo_state.cam.near_clip = gApp.camera.optics.znear;
    gApp.gizmo_state.cam.far_clip = gApp.camera.optics.zfar;
    gApp.gizmo_state.cam.yfov = gApp.camera.vertical_FOV().value;
    gApp.gizmo_state.cam.position = tinygizmo::v3f{ camera_pos.x, camera_pos.y, camera_pos.z };
    gApp.gizmo_state.cam.orientation = tinygizmo::v4f{ camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w };
    gApp.gizmo_state.ray_origin = tinygizmo::v3f{ ray.pos.x, ray.pos.y, ray.pos.z };
    gApp.gizmo_state.ray_direction = tinygizmo::v3f{ ray.dir.x, ray.dir.y, ray.dir.z };
    //gApp.gizmo_state.screenspace_scale = 80.f; // optional flag to draw the gizmos at a constant screen-space scale

    gApp.gizmo_ctx.begin(gApp.gizmo_state);

    static tinygizmo::rigid_transform xform_a;
    static tinygizmo::rigid_transform xform_a_last;
    static std::once_flag once;
    std::call_once(once, []() 
    {
        tinygizmo::m44f tx = xform_a.matrix();
        memcpy(&gApp.gizmo_transform, &tx, sizeof(float) * 16);
        xform_a_last = xform_a;
    });

    bool result = gApp.gizmo_ctx.transform_gizmo("first-example-gizmo", xform_a);
    if (result)
    {
        //std::cout << get_local_time_ns() << " - " << "First Gizmo Hovered..." << std::endl;
        //if (xform_a != xform_a_last) std::cout << get_local_time_ns() << " - " << "First Gizmo Changed..." << std::endl;
        xform_a_last = xform_a;
        tinygizmo::m44f tx = xform_a_last.matrix();
        memcpy(&gApp.gizmo_transform, &tx, sizeof(float) * 16);
    }

    // update index buffer
    gizmo_triangles.triangle_count = gApp.gizmo_ctx.triangles(nullptr, 0);
    if (gizmo_triangles.triangle_count > gizmo_triangles.indices.size())
        gizmo_triangles.indices.resize((size_t)gizmo_triangles.triangle_count * 3);

    gApp.gizmo_ctx.triangles(gizmo_triangles.indices.data(), gizmo_triangles.triangle_count);

    constexpr int vertex_float_count = 10;
    constexpr int vertex_byte_stride = sizeof(float) * vertex_float_count;

    // update vertex buffer
    int vertex_count = gApp.gizmo_ctx.vertices(nullptr, vertex_byte_stride, 0, 0, 0);
    int required_floats = vertex_float_count * vertex_count;
    if (required_floats > (int) gizmo_triangles.vertices.size())
        gizmo_triangles.vertices.resize((size_t)required_floats);

    gApp.gizmo_ctx.vertices(gizmo_triangles.vertices.data(),
        vertex_byte_stride,
        //0                 // position offset @TODO
        sizeof(float) * 3,  // normal offset
        sizeof(float) * 6,  // color offset
        vertex_count);

    gApp.gizmo_ctx.end(gApp.gizmo_state);
    return result;
}

/*-----------------------------------------------------------------------------
     Demonstrate navigation via a virtual joystick hosted in a little window
 */

enum class NavigatorPanelInteraction
{
    None = 0,
    ModeChange,
    TumbleInitiated, TumbleContinued, TumbleEnded
};

struct NavigatorPanel
{
    const float size_x = 256;
    const float size_y = 160;
    const float trackball_width = size_x * 0.5f;
    ImVec2 trackball_size{ trackball_width, size_y };
    float nav_radius = 6;
    lab::camera::InteractionMode camera_interaction_mode = lab::camera::InteractionMode::TurnTableOrbit;
    NavigatorPanelInteraction state = NavigatorPanelInteraction::None;
    bool trackball_interacting = false;
} navigator_panel;


NavigatorPanelInteraction run_navigator_panel()
{
    NavigatorPanelInteraction result = NavigatorPanelInteraction::None;
    ImGui::SetNextWindowSize(ImVec2(navigator_panel.size_x, navigator_panel.size_y), ImGuiCond_FirstUseEver);
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - navigator_panel.size_x, 10));

    ImGui::Begin("Navigator", &gApp.show_navigator,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();
    ImVec2 mouse_pos = ImGui::GetMousePos();

    ImGui::Columns(3, "###NavCol", true);
    ImGui::SetColumnWidth(0, navigator_panel.trackball_width);

    // was tumbling is initiated by clicking the ###Nav button,
    // it should continue until the mouse is no longer held down.

    ImGui::InvisibleButton("###Nav", navigator_panel.trackball_size);
    if (ImGui::IsItemHovered())
    {
        if (ImGui::IsMouseDown(0))
        {
            if (!navigator_panel.trackball_interacting)
            {
                navigator_panel.trackball_interacting = true;
                ImGui::SetCursorScreenPos(cursor_screen_pos);
                ImGui::TextUnformatted("CLICKED");
                gApp.initial_mouse_position = { mouse_pos.x, mouse_pos.y };
                result = NavigatorPanelInteraction::TumbleInitiated;
            }
            else
            {
                result = NavigatorPanelInteraction::TumbleContinued;
                ImGui::SetCursorScreenPos(cursor_screen_pos);
                ImGui::TextUnformatted("DRAGGING");
            }
        }
        else if (navigator_panel.trackball_interacting)
        {
            result = NavigatorPanelInteraction::TumbleEnded;
            navigator_panel.trackball_interacting = false;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("RELEASED");
        }
        else 
        {
            result = NavigatorPanelInteraction::None;
            navigator_panel.trackball_interacting = false;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("HOVERED");
        }
    }
    else if (navigator_panel.trackball_interacting)
    {
        if (ImGui::IsMouseDown(0))
        {
            result = NavigatorPanelInteraction::TumbleContinued;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("DRAGGING OUTSIDE");
        }
        else
        {
            result = NavigatorPanelInteraction::TumbleEnded;
            navigator_panel.trackball_interacting = false;
            ImGui::SetCursorScreenPos(cursor_screen_pos);
            ImGui::TextUnformatted("RELEASED OUTSIDE");
        }
    }

    ImGui::NextColumn();

    if (ImGui::Button("Home###NavHome")) {
        auto up = gApp.camera.world_up_constraint();
        gApp.camera.set_look_at_constraint({ 0.f, 0.2f, navigator_panel.nav_radius }, { 0,0,0 }, up);
        result = NavigatorPanelInteraction::ModeChange;
    }
    if (ImGui::Button(navigator_panel.camera_interaction_mode == lab::camera::InteractionMode::Crane ? "-Crane-" : " Crane ")) {
        navigator_panel.camera_interaction_mode = lab::camera::InteractionMode::Crane;
        result = NavigatorPanelInteraction::ModeChange;
    }
    if (ImGui::Button(navigator_panel.camera_interaction_mode == lab::camera::InteractionMode::Dolly ? "-Dolly-" : " Dolly ")) {
        navigator_panel.camera_interaction_mode = lab::camera::InteractionMode::Dolly;
        result = NavigatorPanelInteraction::ModeChange;
    }
    if (ImGui::Button(navigator_panel.camera_interaction_mode == lab::camera::InteractionMode::TurnTableOrbit ? "-Orbit-" : " Orbit ")) {
        navigator_panel.camera_interaction_mode = lab::camera::InteractionMode::TurnTableOrbit;
        result = NavigatorPanelInteraction::ModeChange;
    }
    if (ImGui::Button(navigator_panel.camera_interaction_mode == lab::camera::InteractionMode::Gimbal ? "-Gimbal-" : " Gimbal ")) {
        navigator_panel.camera_interaction_mode = lab::camera::InteractionMode::Gimbal;
        result = NavigatorPanelInteraction::ModeChange;
    }

    ImGui::NextColumn();

    static float zoom = 0.f;
    if (ImGui::VSliderFloat("###Nav_Zoom", ImVec2(32, navigator_panel.size_y), &zoom, -1.0f, 1.0f)) {
        navigator_panel.nav_radius += zoom;
        result = NavigatorPanelInteraction::ModeChange;
    }
    else
        zoom = 0.f;     // mouse released, reset

    // end of columns.

    ImGui::End();
    return result;
}

/*-----------------------------------------------------------------------------
    Application logic
 */


void run_application_logic()
{
    const int window_width = sapp_width();
    const int window_height = sapp_height();
    const float w = (float)sapp_width();
    const float h = (float)sapp_height();
    const double delta_time = std::max(stm_sec(stm_laptime(&gApp.last_time)), 1./60.);

    static bool once = true;
    if (once) {
        auto up = gApp.camera.world_up_constraint();
        gApp.camera.set_look_at_constraint({ 0.f, 0.2f, navigator_panel.nav_radius }, { 0,0,0 }, up);
        once = false;
    }

    float fovy = lab::camera::degrees_from_radians(gApp.camera.vertical_FOV());
    gApp.camera.optics.focal_length = gApp.camera.sensor.focal_length_from_vertical_FOV(lab::camera::radians_from_degrees(60));
    gApp.camera.optics.squeeze = w / h;
    lab::camera::m44f proj = gApp.camera.perspective();
    lab::camera::m44f view = gApp.camera.mount.view_transform();
    lab::camera::m44f view_t = gApp.camera.mount.view_transform_inv();
    lab::camera::m44f view_proj = gApp.camera.view_projection(1.f);

    lab::camera::v3f pos = gApp.camera.position_constraint();

    sg_begin_default_pass(&gApp.pass_action, window_width, window_height);

    draw_gizmo(&view_t.x.x, &view_proj.x.x, gizmo_triangles.triangle_count, gizmo_triangles.indices.data(), gizmo_triangles.vertices.data());

    start_gl_rendering();

    lab::camera::m44f identity = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    draw_grid(0, identity);
    draw_grid(0, * (const lab::camera::m44f*) &gApp.gizmo_transform);

    {
        lab::camera::m44f m = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

        // display look at
        if (gApp.show_look_at)
        {
            lab::camera::v3f lookat = gApp.camera.focus_constraint();
            m.w = { lookat.x, lookat.y, lookat.z, 1.f };
            draw_jack(1, m);
        }

        m.w = { gApp.initial_hit_point.x, gApp.initial_hit_point.y, gApp.initial_hit_point.z, 1.f };
        draw_jack(0.25, m);

        // hit point on manipulator plane
        if (gApp.show_manip_plane_intersect)
        {
            lab::camera::HitResult hit = gApp.camera.hit_test(
                { mouse.mouse_ws.x, mouse.mouse_ws.y },
                { (float)window_width, (float)window_height },
                *(lab::camera::v3f*)(&gApp.gizmo_transform.w),
                *(lab::camera::v3f*)(&gApp.gizmo_transform.y));
            if (hit.hit)
            {
                m.w = { hit.point.x, hit.point.y, hit.point.z, 1.f };
                draw_jack(0.5f, m);
            }
        }

        // intersection of mouse ray with image plane at 1 unit distance
        if (gApp.show_view_plane_intersect)
        {
            lab::camera::v3f cam_pos = gApp.camera.position_constraint();
            lab::camera::v3f cam_nrm = gApp.camera.mount.forward();
            cam_nrm.x *= -1.f;
            cam_nrm.y *= -1.f;
            cam_nrm.z *= -1.f;
            cam_pos.x += cam_nrm.x;
            cam_pos.y += cam_nrm.y;
            cam_pos.z += cam_nrm.z;

            lab::camera::HitResult hit = gApp.camera.hit_test(
                { mouse.mouse_ws.x, mouse.mouse_ws.y },
                { (float)window_width, (float)window_height },
                cam_pos, cam_nrm);

            if (hit.hit)
            {
                m.w = { hit.point.x, hit.point.y, hit.point.z, 1.f };
                draw_jack(0.1f, m);
            }
        }
    }

    end_gl_rendering();

    simgui_new_frame(window_width, window_height, delta_time);

    ImVec2 canvas_offset = ImGui::GetCursorPos();

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Playground")) {
            ImGui::MenuItem("Quit", 0, &gApp.quit);
            if (gApp.quit)
                sapp_request_quit();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Target")) {
            static bool local = true;
            static bool world = false;
            static bool translate = true;
            static bool rotate = false;
            static bool scale = false;
            if (ImGui::MenuItem("Local", 0, &local))
            {
                gApp.gizmo_ctx.set_frame(tinygizmo::reference_frame::local);
                world = false;
            }
            if (ImGui::MenuItem("World", 0, &world))
            {
                gApp.gizmo_ctx.set_frame(tinygizmo::reference_frame::global);
                local = false;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Translate", 0, &translate))
            {
                gApp.gizmo_ctx.set_mode(tinygizmo::transform_mode::translate);
                rotate = false;
                scale = false;
            }
            if (ImGui::MenuItem("Rotate", 0, &rotate))
            {
                gApp.gizmo_ctx.set_mode(tinygizmo::transform_mode::rotate);
                translate = false;
                scale = false;
            }
            if (ImGui::MenuItem("Scale", 0, &scale))
            {
                gApp.gizmo_ctx.set_mode(tinygizmo::transform_mode::scale);
                translate = false;
                rotate = false;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Show Look at point", 0, &gApp.show_look_at))
            {
            }
            if (ImGui::MenuItem("Show manip plane intersect", 0, &gApp.show_manip_plane_intersect))
            {
            }
            if (ImGui::MenuItem("Show view plane intersect", 0, &gApp.show_view_plane_intersect))
            {
            }

            //if (key == GLFW_KEY_LEFT_CONTROL) gizmo_state.hotkey_ctrl = (action != GLFW_RELEASE);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Windows")) {
            if (ImGui::MenuItem("Show Navigator", 0, &gApp.show_navigator))
            {
            }
            if (ImGui::MenuItem("Show Application State", 0, &gApp.show_state))
            {
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Sokol")) {
            ImGui::MenuItem("Buffers", 0, &gApp.sg_imgui.buffers.open);
            ImGui::MenuItem("Images", 0, &gApp.sg_imgui.images.open);
            ImGui::MenuItem("Shaders", 0, &gApp.sg_imgui.shaders.open);
            ImGui::MenuItem("Pipelines", 0, &gApp.sg_imgui.pipelines.open);
            ImGui::MenuItem("Calls", 0, &gApp.sg_imgui.capture.open);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    ImGui::SetNextWindowPos({ 0, 0 });
    ImGui::SetNextWindowSize({ (float)window_width, (float)window_height });
    static bool begin_flag = false;
    ImGui::Begin("###FULLSCREEN", &begin_flag,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImVec2 canvas_size = ImGui::GetContentRegionAvail();

    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();

    // draw a pixel ruler on the left side of the window
    for (float y = 0; y < window_height; y += 100)
    {
        char buff[32];
        sprintf(buff, "%d", (int)y);
        ImGui::SetCursorScreenPos(ImVec2{ 0, y } + canvas_offset);
        ImGui::TextUnformatted(buff);
    }

    // show where the 2d projected 3d projected 2d hit point is
    {
        lab::camera::v3f cam_pos = gApp.camera.position_constraint();
        lab::camera::v3f cam_nrm = gApp.camera.mount.forward();
        cam_nrm.x *= -1.f;
        cam_nrm.y *= -1.f;
        cam_nrm.z *= -1.f;
        cam_pos.x += cam_nrm.x;
        cam_pos.y += cam_nrm.y;
        cam_pos.z += cam_nrm.z;

        // project onto a plane one unit in front of the camera
        lab::camera::HitResult hit = gApp.camera.hit_test(
            { mouse.mouse_ws.x, mouse.mouse_ws.y },
            { (float)window_width, (float)window_height },
            cam_pos, cam_nrm);

        if (hit.hit)
        {
            lab::camera::v2f vp_sz{ (float)window_width, (float)window_height };
            lab::camera::v2f vp_or = { 0, 0 };
            lab::camera::v2f xy = gApp.camera.project_to_viewport(vp_or, vp_sz, hit.point);
            ImGui::SetCursorScreenPos(ImVec2{ xy.x, xy.y } + canvas_offset - ImVec2{ 4,4 });
            ImGui::TextUnformatted("O");
        }
    }

    ImGui::SetCursorScreenPos(cursor_screen_pos);
    ImVec2 mouse_pos = ImGui::GetMousePos();
    lab::camera::v2f viewport{ (float)canvas_size.x, (float)canvas_size.y };
    lab::camera::InteractionPhase phase = lab::camera::InteractionPhase::Continue;

    if (gApp.show_state)
    {
        ImGui::Begin("App State");
        ImGui::Text("state: %s", name_state(gApp.ui_state));
        lab::camera::v3f ypr = gApp.camera.ypr();
        ImGui::Text("ypr: (%f, %f, %f)\n", ypr.x, ypr.y, ypr.z);
        ImGui::End();
    }

    bool mouse_in_viewport = update_mouseStatus_in_viewport(canvas_offset);
    NavigatorPanelInteraction in = gApp.show_navigator ? run_navigator_panel() : NavigatorPanelInteraction::None;
    if (in > NavigatorPanelInteraction::None)
    {
        if (in > NavigatorPanelInteraction::ModeChange)
        {
            if (in == NavigatorPanelInteraction::TumbleInitiated)
                phase = lab::camera::InteractionPhase::Start;
            else if (in == NavigatorPanelInteraction::TumbleEnded)
                phase = lab::camera::InteractionPhase::Finish;

            ImGui::CaptureMouseFromApp(true);
            const float speed_scaler = 10.f;
            float scale = speed_scaler / navigator_panel.size_x;
            float dx = (mouse_pos.x - gApp.initial_mouse_position.x) * scale;
            float dy = (mouse_pos.y - gApp.initial_mouse_position.y) * -scale;

            lab::camera::InteractionToken tok = gApp.camera.begin_interaction(viewport);
            gApp.camera.joystick_interaction(tok, phase, navigator_panel.camera_interaction_mode, { dx, dy });
            gApp.camera.end_interaction(tok);

            mouse = MouseState{};
        }
        gApp.ui_state = UIStateMachine::None;
        run_gizmo( -1, -1, (float)window_width, (float)window_height);
    }
    else if (mouse_in_viewport)
    {
        if (gApp.ui_state == UIStateMachine::Gizmo)
        {
            if (!run_gizmo(mouse.mouse_ws.x, mouse.mouse_ws.y, (float)window_width, (float)window_height))
            {
                gApp.ui_state = UIStateMachine::None;
                sapp_lock_mouse(false);
            }
        }
        else if (gApp.ui_state <= UIStateMachine::UI)
        {
            if (run_gizmo(mouse.mouse_ws.x, mouse.mouse_ws.y, (float)window_width, (float)window_height))
            {
                gApp.ui_state = UIStateMachine::Gizmo;
            }
            else if (mouse.click_initiated)
            {
                // hit test versus the gizmo's plane
                lab::camera::HitResult hit = gApp.camera.hit_test(
                    { mouse.mouse_ws.x, mouse.mouse_ws.y },
                    { (float)window_width, (float)window_height },
                    *(lab::camera::v3f*)(&gApp.gizmo_transform.w),
                    *(lab::camera::v3f*)(&gApp.gizmo_transform.y));

                if (mouse.click_initiated && gApp.ui_state <= UIStateMachine::UI)
                {
                    gApp.initial_mouse_position = { mouse_pos.x, mouse_pos.y };

                    if (hit.hit)
                    {
                        gApp.initial_hit_point = hit.point;
                        gApp.ui_state = UIStateMachine::TTLCamera;
                    }
                    else
                    {
                        lab::camera::v3f cam_pos = gApp.camera.position_constraint();
                        lab::camera::v3f cam_nrm = gApp.camera.mount.forward();
                        cam_nrm.x *= -1.f;
                        cam_nrm.y *= -1.f;
                        cam_nrm.z *= -1.f;
                        cam_pos.x += cam_nrm.x;
                        cam_pos.y += cam_nrm.y;
                        cam_pos.z += cam_nrm.z;

                        hit = gApp.camera.hit_test(
                            { mouse.mouse_ws.x, mouse.mouse_ws.y },
                            { (float)window_width, (float)window_height },
                            cam_pos, cam_nrm);

                        if (hit.hit)
                        {
                            gApp.initial_hit_point = hit.point;
                            gApp.ui_state = UIStateMachine::DeltaCamera;
                        }
                    }
                }
            }
        }

        if (gApp.ui_state == UIStateMachine::DeltaCamera || gApp.ui_state == UIStateMachine::TTLCamera)
        {
            if (mouse.click_initiated)
                phase = lab::camera::InteractionPhase::Start;
            else if (mouse.click_ended)
                phase = lab::camera::InteractionPhase::Finish;

            ImGui::CaptureMouseFromApp(true);

            lab::camera::InteractionToken tok = gApp.camera.begin_interaction(viewport);
            if (gApp.ui_state == UIStateMachine::TTLCamera)
            {
                ImGui::SetCursorScreenPos(ImVec2{ mouse_pos.x, mouse_pos.y });
                ImGui::TextUnformatted("TTL+");

                // through the lens mode
                gApp.camera.constrained_ttl_interaction(
                    tok,
                    phase, navigator_panel.camera_interaction_mode,
                    { mouse_pos.x, mouse_pos.y },
                    gApp.initial_hit_point);
            }
            else
            {
                ImGui::SetCursorScreenPos(ImVec2{ mouse_pos.x, mouse_pos.y });
                ImGui::TextUnformatted("TTL");

                // virtual joystick mode
                gApp.camera.ttl_interaction(
                    tok,
                    phase, navigator_panel.camera_interaction_mode,
                    { mouse_pos.x, mouse_pos.y });
            }
            gApp.camera.end_interaction(tok);

            if (mouse.click_ended)
            {
                gApp.ui_state = UIStateMachine::None;
                sapp_lock_mouse(false);
            }
        }
    }

    ImGui::End(); // full screen

    sg_imgui_draw(&gApp.sg_imgui);
    simgui_render();

    // the sokol_gfx draw pass
    sg_end_pass();
    sg_commit();
}

void imgui_callback_handler(const sapp_event* event)
{
    simgui_handle_event(event);
}

sapp_desc sokol_main(int argc, char* argv[]) 
{
    std::string app_path(argv[0]);
    size_t index = app_path.rfind('/');
    if (index == std::string::npos)
        index = app_path.rfind('\\');
    gApp.g_app_path = app_path.substr(0, index);

    sapp_desc desc = { };
    desc.init_cb = initialize_graphics;
    desc.frame_cb = run_application_logic;
    desc.cleanup_cb = shutdown_graphics;
    desc.event_cb = imgui_callback_handler;
    desc.width = 1200;
    desc.height = 1000;
    desc.gl_force_gles2 = true;
    desc.window_title = "LabSound Playground";
    desc.ios_keyboard_resizes_canvas = false;
    return desc;
}
