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

static uint64_t last_time = 0;
static bool show_test_window = true;
static bool show_another_window = true;
static bool show_navigator = true;

static sg_pass_action pass_action;

extern "C" void draw_cube();
extern "C" void draw_gizmo();

void SimpleEffect(const float* v_t, const float* mvp, float t, float dt);


static lab::camera::Camera camera;


static void grid(float y, uint32_t frame_count) {
    static sgl_pipeline depth_test_pip;
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
        depth_test_pip = sgl_make_pipeline(&sg_p);

        once = false;
    }

    sgl_defaults();
    sgl_push_pipeline();
    sgl_load_pipeline(depth_test_pip);

    sgl_matrix_mode_projection();

    lab::camera::m44f proj = lab::camera::perspective(camera.sensor, camera.optics);
    sgl_load_matrix(&proj.x.x);

    lab::camera::m44f view = camera.mount.viewTransform();
    sgl_matrix_mode_modelview();
    sgl_load_matrix(&view.x.x);

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

    sgl_pop_pipeline();
}

lab::ImGui::FontManager* g_fm = nullptr;
std::string g_app_path;
static bool quit = false;
ImFont* g_roboto = nullptr;
lab::TimeTransport* g_time_transport = nullptr;

static sg_imgui_t sg_imgui;

tinygizmo::gizmo_application_state gizmo_state;
tinygizmo::gizmo_context gizmo_ctx;

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


void run_gizmo()
{
    ImGui::BeginChild("###Gizmo");
    ImGuiIO& io = ImGui::GetIO();
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;
    float width = edit_rect.Max.x - edit_rect.Min.x;
    float height = edit_rect.Max.y - edit_rect.Min.y;

    bool rv = ImGui::BeginChild("###Gizmo_Content", ImVec2(0, height), false,
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse);

    //---------------------------------------------------------------------
    // determine hovered, dragging, pressed, and released, as well as
    // window local coordinate and canvas local coordinate

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

    mouse.click_ended = false;
    mouse.click_initiated = false;
    mouse.in_canvas = false;
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
    {
        ImGui::SetCursorScreenPos(edit_rect.Min);
        ImGui::PushID("###InteractiveRegion");
        bool result = ImGui::InvisibleButton("3dView", edit_rect.GetSize());
        ImGui::PopID();
        if (result)
        {
            //printf("button released\n");
            mouse.dragging_node = false;
            mouse.resizing_node = false;
            mouse.click_ended = true;
        }
        mouse.in_canvas = ImGui::IsItemHovered();

        if (mouse.in_canvas)
        {
            mouse.mouse_ws = io.MousePos - ImGui::GetCurrentWindow()->Pos;

            if (io.MouseDown[0] && io.MouseDownOwned[0])
            {
                if (!mouse.dragging)
                {
                    //printf("button clicked\n");
                    mouse.click_initiated = true;
                    mouse.initial_click_pos_ws = io.MousePos;
                }

                mouse.dragging = true;
            }
            else
                mouse.dragging = false;
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

    //g_tt->ui(*g_fm);

    ImGui::EndChild();
    ImGui::EndChild();

    gizmo_state.mouse_left = mouse.click_initiated || mouse.dragging;

    // Gizmo input interaction state populated via win->on_input(...) callback above. Update app parameters: 
    lab::camera::v3f camera_pos = camera.mount.position();
    lab::camera::Ray ray = lab::camera::get_ray_from_pixel(camera, { mouse.mouse_ws.x, mouse.mouse_ws.y }, { 0, 0 }, { width, height });
    lab::camera::quatf camera_orientation = camera.mount.rotation();

    gizmo_state.viewport_size = tinygizmo::v2f{ width, height };
    gizmo_state.cam.near_clip = camera.optics.znear;
    gizmo_state.cam.far_clip = camera.optics.zfar;
    gizmo_state.cam.yfov = lab::camera::verticalFOV(camera.sensor, camera.optics);
    gizmo_state.cam.position = tinygizmo::v3f{ camera_pos.x, camera_pos.y, camera_pos.z };
    gizmo_state.cam.orientation = tinygizmo::v4f{ camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w };
    gizmo_state.ray_origin = gizmo_state.cam.position;
    gizmo_state.ray_direction = tinygizmo::v3f{ ray.dir.x, ray.dir.y, ray.dir.z };
    //gizmo_state.screenspace_scale = 80.f; // optional flag to draw the gizmos at a constant screen-space scale

    gizmo_ctx.begin(gizmo_state);

    static tinygizmo::rigid_transform xform_a;
    static tinygizmo::rigid_transform xform_a_last;

    if (gizmo_ctx.transform_gizmo("first-example-gizmo", xform_a))
    {
        //std::cout << get_local_time_ns() << " - " << "First Gizmo Hovered..." << std::endl;
        //if (xform_a != xform_a_last) std::cout << get_local_time_ns() << " - " << "First Gizmo Changed..." << std::endl;
        xform_a_last = xform_a;
    }


    // update index buffer
    int triangle_count = gizmo_ctx.triangles(gizmo_triangles.indices.data(), (int) gizmo_triangles.indices.size());
    if (triangle_count > gizmo_triangles.indices.size())
    {
        gizmo_triangles.indices.resize(triangle_count * 3);
        triangle_count = gizmo_ctx.triangles(gizmo_triangles.indices.data(), (int) gizmo_triangles.indices.size());
    }
    else if (triangle_count < gizmo_triangles.indices.size())
    {
        gizmo_triangles.indices.resize(triangle_count);
    }

    // update vertex buffer
    int vertex_count = gizmo_ctx.vertices(gizmo_triangles.vertices.data(), sizeof(float)*9,
        sizeof(float) * 3, sizeof(float) * 6, (int) gizmo_triangles.vertices.size());
    if (vertex_count * 9 > gizmo_triangles.vertices.size())
    {
        gizmo_triangles.vertices.resize(vertex_count * 9);
        size_t vertex_count = gizmo_ctx.vertices(gizmo_triangles.vertices.data(), sizeof(float) * 9,
            sizeof(float) * 3, sizeof(float) * 6, (int) gizmo_triangles.vertices.size());
    }
    else if (vertex_count < gizmo_triangles.vertices.size())
    {
        gizmo_triangles.vertices.resize(vertex_count);
    }

//    upload_mesh(r, gizmoEditorMesh);
    gizmo_ctx.end(gizmo_state);
}


void frame()
{
    const int width = sapp_width();
    const int height = sapp_height();
    const float w = (float)sapp_width();
    const float h = (float)sapp_height();
    const double delta_time = std::max(stm_sec(stm_laptime(&last_time)), 1./60.);
    static float nav_radius = 6;

    static bool once = true;
    if (once) {
        camera.position = { 0.f, 0.f, nav_radius };
        camera.focusPoint = { 0, 0, 0 };
        camera.updateViewTransform();
        once = false;
    }

    camera.optics.focalLength = 31.f;//60 degrees
    camera.optics.squeeze = w / h;
    lab::m44f proj = m44f_cast(lab::camera::perspective(camera.sensor, camera.optics));
    lab::m44f view = m44f_cast(camera.mount.viewTransform());
    lab::m44f view_proj = lab::matrix_multiply(proj, view);
    lab::m44f view_t = lab::matrix_transpose(view);

    sg_begin_default_pass(&pass_action, width, height);

    grid(0, 0);

    draw_cube();
    //draw_gizmo();
    SimpleEffect(&view_t.x.x, &view_proj.x.x, static_cast<float>(stm_sec(stm_now())), static_cast<float>(delta_time));


    {
        //draw_mesh(wireframeShader, gizmoEditorMesh, cam.position, cam.get_viewproj_matrix((float)windowSize.x / (float)windowSize.y), identity4x4);
    }


    sgl_draw();

    simgui_new_frame(width, height, delta_time);

    ImGui::PushFont(g_roboto);

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Playground")) {
            ImGui::MenuItem("Quit", 0, &quit);
            if (quit)
                sapp_request_quit();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Target")) {
            static bool local_world = false;
            static bool translate = false;
            static bool rotate = false;
            static bool scale = false;
            ImGui::MenuItem("Local/World", 0, &local_world);
            ImGui::MenuItem("Translate", 0, &translate);
            ImGui::MenuItem("Rotate", 0, &rotate);
            ImGui::MenuItem("Scale", 0, &scale);
            gizmo_state.hotkey_local = local_world;
            gizmo_state.hotkey_translate = translate;
            gizmo_state.hotkey_rotate = rotate;
            gizmo_state.hotkey_scale = scale;
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

   // run_gizmo();

    if (show_navigator)
    {
        static float nav_size_x = 256;
        static float nav_size_y = 128;
        static float trackball_width = 128;
        ImVec2 trackball_size{ 128, 128 };
        ImGui::SetNextWindowSize(ImVec2(nav_size_x, nav_size_y), ImGuiCond_FirstUseEver);
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - nav_size_x, 0));

        ImGui::Begin("Navigator", &show_navigator,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse);

        ImVec2 controlPos = ImGui::GetCursorScreenPos();
        ImVec2 mouse = ImGui::GetMousePos() - controlPos;
        static char msg[256];

        float dx = (2.f * mouse.x - trackball_width) / (2.f * trackball_width);
        float dy = (2.f * mouse.y - trackball_width) / (2.f * trackball_width);

        ImGui::Columns(3, "###NavCol", true);
        ImGui::SetColumnWidth(0, trackball_width);

        static bool pan = false;
        static bool dolly = false;
        static bool orbit = true;

        // was tumbling is initiated by clicking the ###Nav button,
        // it should continue until the mouse is no longer held down.
        static bool tumbling = false;
        ImGui::InvisibleButton("###Nav", trackball_size);
        if (ImGui::IsItemActive()) {
            ImGui::SetCursorScreenPos(controlPos);
            ImGui::TextUnformatted("CLICKED");
            sprintf(msg, "%03f, %03f", dx, dy);
            ImGui::TextUnformatted(msg);
            tumbling = true;
        }
        else if (ImGui::IsItemHovered()) {
            ImGui::SetCursorScreenPos(controlPos);
            ImGui::TextUnformatted("HOVERED");
            sprintf(msg, "%03f, %03f", dx, dy);
            ImGui::TextUnformatted(msg);
        }

        // if the lmb is down, tumbling should continue even if the mouse leaves the widget
        if (!ImGui::IsMouseDown(0)) {
            tumbling = false;
        }

        if (tumbling) {
            ImGui::CaptureMouseFromApp(true);
            if (pan)
                lab::camera::cameraRig_interact(camera, lab::camera::CameraRigMode::Crane, { dx, dy });
            else if (dolly)
                lab::camera::cameraRig_interact(camera, lab::camera::CameraRigMode::Dolly, { dx, dy });
            else if (orbit)
                lab::camera::cameraRig_interact(camera, lab::camera::CameraRigMode::TurnTableOrbit, { dx, dy });
        }

        ImGui::NextColumn();

        if (ImGui::Button("Home###NavHome")) {
            camera.position = { 0.f, 0.f, nav_radius };
            camera.focusPoint = { 0, 0, 0 };
            camera.updateViewTransform();
        }
        if (ImGui::Button(pan? " -Pan- " : "  Pan  ")) {
            pan = true; dolly = false; orbit = false;
        }
        if (ImGui::Button(dolly? "-Dolly-" : " Dolly ")) {
            pan = false; dolly = true; orbit = false;
        }
        if (ImGui::Button(orbit? "-Orbit-" : " Orbit ")) {
            pan = false; dolly = false; orbit = true;
        }

        ImGui::NextColumn();

        static float zoom = 0.f;
        if (ImGui::VSliderFloat("###Nav_Zoom", ImVec2(32, nav_size_y), &zoom, -1.0f, 1.0f)) {
            nav_radius += zoom;
        }
        else
            zoom = 0.f;     // mouse released, reset

        // end of columns.

        // this quat constructor is w, xyz,
        // guts are xyz, w.
        //static quat qt = quat(1.f, 0.f, 0.f, 0.f);
        //if (ImGui::gizmo3D("##gizmo1", qt /*, size,  mode */)) { 
        //    camera.mount.setViewTransform(*(reinterpret_cast<lab::quatf*>(&qt.x)), 6.f);
        //}

        //float camDistance = 8.f;
        //ImGuizmo::SetDrawlist();
        //ImGuizmo::ViewManipulate(&view.Elements[0][0], camDistance, ImVec2(io.DisplaySize.x - 128, 0), ImVec2(128, 128), 0x10101010);
        ImGui::End();
    }

    ImGui::PopFont();

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
