//------------------------------------------------------------------------------
//  imgui-sapp.c
//
//  Demonstrates Dear ImGui UI rendering via sokol_gfx.h and
//  the utility header sokol_imgui.h
//------------------------------------------------------------------------------

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
#include "LabImGui/FontManager.h"
#include "LabImGui/Timeline.h"
#include "LabImGui/TimeTransport.h"
#include <LabMath/LabMath.h>
#include <tiny-gizmo.hpp>

#define LAB_CAMERA_DRY
#include "LabCamera.h"

lab::m44f& m44f_cast(lab::camera::m44f& m) { return *(reinterpret_cast<lab::m44f*>(&m.x.x)); }
const lab::m44f& m44f_cast(const lab::camera::m44f& m) { return *(reinterpret_cast<const lab::m44f*>(&m.x.x)); }
lab::v3f& v3f_cast(lab::camera::v3f& v) { return *(reinterpret_cast<lab::v3f*>(&v.x)); }
const lab::v3f& v3f_cast(const lab::camera::v3f& v) { return *(reinterpret_cast<const lab::v3f*>(&v.x)); }

static lab::m44f gizmo_transform;

static uint64_t last_time = 0;
static bool show_test_window = true;
static bool show_another_window = true;
static bool show_navigator = true;

static sgl_pipeline gl_pipelne;

static sg_pass_action pass_action;

lab::ImGui::FontManager* g_fm = nullptr;
std::string g_app_path;
static bool quit = false;
ImFont* g_roboto = nullptr;
lab::TimeTransport* g_time_transport = nullptr;

static sg_imgui_t sg_imgui;

tinygizmo::gizmo_application_state gizmo_state;
tinygizmo::gizmo_context gizmo_ctx;

extern "C" void draw_cube();

void simple_effect(const float* v_t, const float* mvp, float t, float dt);
static lab::camera::Camera camera;


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
};
static MouseState mouse;

float random()
{
    int r = rand() & 0x7fff;
    return (float)r / 32767.f;
}

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
        gl_pipelne = sgl_make_pipeline(&sg_p);

        once = false;
    }

    sgl_defaults();
    sgl_push_pipeline();
    sgl_load_pipeline(gl_pipelne);
}

static void end_gl_rendering()
{
    sgl_pop_pipeline();
    sgl_draw();
}

static void grid(float y, const lab::m44f& m)
{
    sgl_matrix_mode_projection();

    lab::camera::m44f proj = lab::camera::perspective(camera.sensor, camera.optics);
    sgl_load_matrix(&proj.x.x);

    lab::camera::m44f view = camera.mount.view_transform();
    lab::m44f mv = *(lab::m44f*) &view.x.x * m;

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


void init()
{
    // setup sokol-gfx, sokol-time and sokol-imgui
    sg_desc desc = { };
    desc.context = sapp_sgcontext();
    sg_setup(&desc);
    stm_setup();

    // setup debug inspection header(s)
    sg_imgui_init(&sg_imgui);

    // setup sokol-imgui, but provide our own font
    simgui_desc_t simgui_desc = { };
    simgui_desc.no_default_font = true;
    simgui_desc.sample_count = sapp_sample_count();
    simgui_desc.dpi_scale = sapp_dpi_scale();
    simgui_setup(&simgui_desc);

    // initial clear color
    pass_action.colors[0].action = SG_ACTION_CLEAR;
    pass_action.colors[0].val[0] = 0.0f;
    pass_action.colors[0].val[1] = 0.5f;
    pass_action.colors[0].val[2] = 0.7f;
    pass_action.colors[0].val[3] = 1.0f;

    std::string resource_path = g_app_path + "/Navigator_rsrc";
    g_fm = new lab::ImGui::FontManager(resource_path.c_str());
    g_roboto = g_fm->GetFont(lab::ImGui::FontName::Regular);

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

    g_time_transport = new lab::TimeTransport();
}

void update_mouse_in_viewport(ImVec2 canvas_offset)
{
    // the 3d viewport should be the current window
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;

    ImGuiIO& io = ImGui::GetIO();

    float width = ImGui::GetContentRegionAvailWidth();
    float height = ImGui::GetContentRegionAvail().y;
    ImVec2 imgui_cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorScreenPos(edit_rect.Min);
    bool click_finished = ImGui::InvisibleButton("###GIZMOREGION", edit_rect.GetSize());

    //---------------------------------------------------------------------
    // determine hovered, dragging, pressed, and released, as well as
    // window local coordinate and canvas local coordinate

    mouse.click_ended = false;
    mouse.click_initiated = false;
    mouse.in_canvas = click_finished || ImGui::IsItemHovered();
    if (mouse.in_canvas)
    {
        if (click_finished)
        {
            //printf("button released\n");
            mouse.dragging_node = false;
            mouse.resizing_node = false;
            mouse.click_ended = true;
        }

        mouse.mouse_ws = io.MousePos - ImGui::GetCurrentWindow()->Pos;

        if (io.MouseDown[0] && io.MouseDownOwned[0])
        {
            if (!mouse.dragging)
            {
                //printf("button clicked\n");
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

    if (!mouse.dragging)
    {
        // clear hover status
    }
    ImGui::SetCursorPos(imgui_cursor_pos);
}

// return true if the gizmo was interacted
bool run_gizmo(float width, float height)
{
    //g_tt->ui(*g_fm);

    lab::camera::v3f camera_pos = camera.mount.position();
    lab::camera::Ray ray = camera.get_ray_from_pixel({ mouse.mouse_ws.x, mouse.mouse_ws.y }, { 0, 0 }, { width, height });
    lab::camera::quatf camera_orientation = camera.mount.rotation();

    gizmo_state.mouse_left = mouse.click_initiated || mouse.dragging;
    gizmo_state.viewport_size = tinygizmo::v2f{ width, height };
    gizmo_state.cam.near_clip = camera.optics.znear;
    gizmo_state.cam.far_clip = camera.optics.zfar;
    gizmo_state.cam.yfov = lab::camera::vertical_FOV(camera.sensor, camera.optics).value;
    gizmo_state.cam.position = tinygizmo::v3f{ camera_pos.x, camera_pos.y, camera_pos.z };
    gizmo_state.cam.orientation = tinygizmo::v4f{ camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w };
    gizmo_state.ray_origin = tinygizmo::v3f{ ray.pos.x, ray.pos.y, ray.pos.z };
    gizmo_state.ray_direction = tinygizmo::v3f{ ray.dir.x, ray.dir.y, ray.dir.z };
    //gizmo_state.screenspace_scale = 80.f; // optional flag to draw the gizmos at a constant screen-space scale

    gizmo_ctx.begin(gizmo_state);

    static tinygizmo::rigid_transform xform_a;
    static tinygizmo::rigid_transform xform_a_last;

    bool result = gizmo_ctx.transform_gizmo("first-example-gizmo", xform_a);
    if (result)
    {
        //std::cout << get_local_time_ns() << " - " << "First Gizmo Hovered..." << std::endl;
        //if (xform_a != xform_a_last) std::cout << get_local_time_ns() << " - " << "First Gizmo Changed..." << std::endl;
        xform_a_last = xform_a;
        tinygizmo::m44f tx = xform_a_last.matrix();
        memcpy(&gizmo_transform, &tx, sizeof(float) * 16);
    }

    // update index buffer
    gizmo_triangles.triangle_count = gizmo_ctx.triangles(nullptr, 0);
    if (gizmo_triangles.triangle_count > gizmo_triangles.indices.size())
        gizmo_triangles.indices.resize((size_t)gizmo_triangles.triangle_count * 3);

    gizmo_ctx.triangles(gizmo_triangles.indices.data(), gizmo_triangles.triangle_count);

    constexpr int vertex_float_count = 10;
    constexpr int vertex_byte_stride = sizeof(float) * vertex_float_count;

    // update vertex buffer
    int vertex_count = gizmo_ctx.vertices(nullptr, vertex_byte_stride, 0, 0, 0);
    int required_floats = vertex_float_count * vertex_count;
    if (required_floats > (int) gizmo_triangles.vertices.size())
        gizmo_triangles.vertices.resize((size_t)required_floats);

    gizmo_ctx.vertices(gizmo_triangles.vertices.data(), 
        vertex_byte_stride,
        //0                 // position offset @TODO
        sizeof(float) * 3,  // normal offset
        sizeof(float) * 6,  // color offset
        vertex_count);

    gizmo_ctx.end(gizmo_state);
    return result;
}

struct NavigatorPanel
{
    const float size_x = 256;
    const float size_y = 160;
    const float trackball_width = size_x * 0.5f;
    ImVec2 trackball_size{ trackball_width, size_y };
    float capture_x = 0;
    float capture_y = 0;
    lab::camera::Camera initial_camera;
    bool tumbling = false;
    float nav_radius = 6;
    lab::camera::CameraRigMode interaction_mode = lab::camera::CameraRigMode::TurnTableOrbit;
} navigator_panel;


void run_navigator_panel()
{
    ImGui::SetNextWindowSize(ImVec2(navigator_panel.size_x, navigator_panel.size_y), ImGuiCond_FirstUseEver);
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - navigator_panel.size_x, 10));

    ImGui::Begin("Navigator", &show_navigator,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();
    ImVec2 mouse_pos = ImGui::GetMousePos();
    static char msg[256];

    ImGui::Columns(3, "###NavCol", true);
    ImGui::SetColumnWidth(0, navigator_panel.trackball_width);

    // was tumbling is initiated by clicking the ###Nav button,
    // it should continue until the mouse is no longer held down.
    static bool tumbling = false;
    ImGui::InvisibleButton("###Nav", navigator_panel.trackball_size);
    if (ImGui::IsItemActive()) {
        ImGui::SetCursorScreenPos(cursor_screen_pos);

        if (navigator_panel.tumbling)
            ImGui::TextUnformatted("DRAGGING");
        else
        {
            ImGui::TextUnformatted("CLICKED");
            navigator_panel.capture_x = mouse_pos.x;
            navigator_panel.capture_y = mouse_pos.y;
            navigator_panel.tumbling = true;
        }
    }
    else if (ImGui::IsItemHovered()) {
        ImGui::SetCursorScreenPos(cursor_screen_pos);
        ImGui::TextUnformatted("HOVERED");
        ImGui::TextUnformatted(msg);
    }

    // if the lmb is released, tumbling should be cancelled
    if (!ImGui::IsMouseDown(0)) {
        navigator_panel.tumbling = false;
    }

    ImGui::NextColumn();

    if (ImGui::Button("Home###NavHome")) {
        camera.position = { 0.f, 0.f, navigator_panel.nav_radius };
        camera.focus_point = { 0, 0, 0 };
        camera.mount.look_at(camera.position, camera.focus_point, camera.world_up);
    }
    if (ImGui::Button(navigator_panel.interaction_mode == lab::camera::CameraRigMode::Crane ? "-Crane-" : " Crane ")) {
        navigator_panel.interaction_mode = lab::camera::CameraRigMode::Crane;
    }
    if (ImGui::Button(navigator_panel.interaction_mode == lab::camera::CameraRigMode::Dolly ? "-Dolly-" : " Dolly ")) {
        navigator_panel.interaction_mode = lab::camera::CameraRigMode::Dolly;
    }
    if (ImGui::Button(navigator_panel.interaction_mode == lab::camera::CameraRigMode::TurnTableOrbit ? "-Orbit-" : " Orbit ")) {
        navigator_panel.interaction_mode = lab::camera::CameraRigMode::TurnTableOrbit;
    }
    if (ImGui::Button(navigator_panel.interaction_mode == lab::camera::CameraRigMode::Gimbal ? "-Gimbal-" : " Gimbal ")) {
        navigator_panel.interaction_mode = lab::camera::CameraRigMode::Gimbal;
    }

    ImGui::NextColumn();

    static float zoom = 0.f;
    if (ImGui::VSliderFloat("###Nav_Zoom", ImVec2(32, navigator_panel.size_y), &zoom, -1.0f, 1.0f)) {
        navigator_panel.nav_radius += zoom;
    }
    else
        zoom = 0.f;     // mouse released, reset

    // end of columns.

    ImGui::End();
}


void frame()
{
    const int window_width = sapp_width();
    const int window_height = sapp_height();
    const float w = (float)sapp_width();
    const float h = (float)sapp_height();
    const double delta_time = std::max(stm_sec(stm_laptime(&last_time)), 1./60.);

    static bool once = true;
    if (once) {
        camera.position = { 0.f, 0.f, navigator_panel.nav_radius };
        camera.focus_point = { 0, 0, 0 };
        camera.mount.look_at(camera.position, camera.focus_point, camera.world_up);
        once = false;
    }

    float fovy = lab::camera::degrees_from_radians(lab::camera::vertical_FOV(camera.sensor, camera.optics));
    camera.optics.focal_length = camera.sensor.focal_length_from_FOV(lab::camera::radians_from_degrees(60));
    camera.optics.squeeze = w / h;
    lab::m44f proj = m44f_cast(lab::camera::perspective(camera.sensor, camera.optics));
    lab::m44f view = m44f_cast(camera.mount.view_transform());
    lab::m44f view_proj = lab::matrix_multiply(proj, view);
    lab::m44f view_t = lab::matrix_transpose(view);

    lab::camera::v3f pos = camera.position;

    sg_begin_default_pass(&pass_action, window_width, window_height);


    //draw_cube();

    draw_gizmo(&view_t.x.x, &view_proj.x.x, gizmo_triangles.triangle_count, gizmo_triangles.indices.data(), gizmo_triangles.vertices.data());

    //SimpleEffect(&view_t.x.x, &view_proj.x.x, static_cast<float>(stm_sec(stm_now())), static_cast<float>(delta_time));
    start_gl_rendering();

    grid(0, lab::m44f_identity);
    grid(0, gizmo_transform);

    end_gl_rendering();

    simgui_new_frame(window_width, window_height, delta_time);

    ImVec2 canvas_offset = ImGui::GetCursorPos();

    ImGui::PushFont(g_roboto);

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Playground")) {
            ImGui::MenuItem("Quit", 0, &quit);
            if (quit)
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
                gizmo_ctx.set_frame(tinygizmo::reference_frame::local);
                world = false;
            }
            if (ImGui::MenuItem("World", 0, &world))
            {
                gizmo_ctx.set_frame(tinygizmo::reference_frame::global);
                local = false;
            }
            if (ImGui::MenuItem("Translate", 0, &translate))
            {
                gizmo_ctx.set_mode(tinygizmo::transform_mode::translate);
                rotate = false;
                scale = false;
            }
            if (ImGui::MenuItem("Rotate", 0, &rotate))
            {
                gizmo_ctx.set_mode(tinygizmo::transform_mode::rotate);
                translate = false;
                scale = false;
            }
            if (ImGui::MenuItem("Scale", 0, &scale))
            {
                gizmo_ctx.set_mode(tinygizmo::transform_mode::scale);
                translate = false;
                rotate = false;
            }

            //if (key == GLFW_KEY_LEFT_CONTROL) gizmo_state.hotkey_ctrl = (action != GLFW_RELEASE);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Sokol")) {
            ImGui::MenuItem("Buffers", 0, &sg_imgui.buffers.open);
            ImGui::MenuItem("Images", 0, &sg_imgui.images.open);
            ImGui::MenuItem("Shaders", 0, &sg_imgui.shaders.open);
            ImGui::MenuItem("Pipelines", 0, &sg_imgui.pipelines.open);
            ImGui::MenuItem("Calls", 0, &sg_imgui.capture.open);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    ImGui::SetNextWindowPos({ 0, 0 });
    ImGui::SetNextWindowSize({ (float) window_width, (float) window_height });
    ImGui::Begin("###FULLSCREEN", false,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse |ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImVec2 canvas_size = ImGui::GetContentRegionAvail();

    ImVec2 cursor_screen_pos = ImGui::GetCursorScreenPos();

    for (float y = 0; y < window_height; y += 100)
    {
        char buff[32];
        sprintf(buff, "%d", (int) y);
        ImGui::SetCursorScreenPos(ImVec2{ 0, y } + canvas_offset);
        ImGui::TextUnformatted(buff);
    }

    ImGui::SetCursorScreenPos(cursor_screen_pos);

    update_mouse_in_viewport(canvas_offset);

    bool gizmo_interacted = run_gizmo((float)window_width, (float)window_height);

    if (show_navigator)
        run_navigator_panel();

    if (navigator_panel.tumbling) 
    {
        ImGui::CaptureMouseFromApp(true);

        //float dx = (2.f * mouse.x - trackball_width) / (2.f * trackball_width);
        //float dy = (2.f * mouse.y - trackball_width) / (2.f * trackball_width);
        ImVec2 mouse_pos = ImGui::GetMousePos();
        float dx = (mouse_pos.x - navigator_panel.capture_x) * 0.1f;
        float dy = (mouse_pos.y - navigator_panel.capture_y) * -0.1f;

        camera.rig_interact(navigator_panel.interaction_mode, { dx, dy });
    }
    else if (!gizmo_interacted && (mouse.click_initiated || mouse.dragging))
    {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        if (mouse.click_initiated)
        {
            navigator_panel.capture_x = mouse_pos.x;
            navigator_panel.capture_y = mouse_pos.y;
            navigator_panel.initial_camera = camera;
        }

        ImGui::CaptureMouseFromApp(true);
        camera.rig_interact(navigator_panel.interaction_mode, 
            { (float)canvas_size.x, (float)canvas_size.y }, 
            navigator_panel.initial_camera,
            { navigator_panel.capture_x, navigator_panel.capture_y }, { mouse_pos.x, mouse_pos.y });
    }

    ImGui::PopFont();

    ImGui::End(); // full screen

    sg_imgui_draw(&sg_imgui);
    simgui_render();

    // the sokol_gfx draw pass
    sg_end_pass();
    sg_commit();
}

void cleanup(void) {
    simgui_shutdown();
    sg_imgui_discard(&sg_imgui);
    sgl_shutdown();
    sg_shutdown();
}

void input(const sapp_event* event) {
    simgui_handle_event(event);
}

sapp_desc sokol_main(int argc, char* argv[]) 
{
    std::string app_path(argv[0]);
    size_t index = app_path.rfind('/');
    if (index == std::string::npos)
        index = app_path.rfind('\\');
    g_app_path = app_path.substr(0, index);

    sapp_desc desc = { };
    desc.init_cb = init;
    desc.frame_cb = frame;
    desc.cleanup_cb = cleanup;
    desc.event_cb = input;
    desc.width = 1200;
    desc.height = 1000;
    desc.gl_force_gles2 = true;
    desc.window_title = "LabSound Playground";
    desc.ios_keyboard_resizes_canvas = false;
    return desc;
}
