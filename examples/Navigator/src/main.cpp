
/*-----------------------------------------------------------------------------
    Demonstrating the use of lab::Camera with ImGui and sokol.
 */

#include "lab_sokol_config.h"

#include "LabCamera.h"
#include "LabCameraImgui.h"

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
    Utility
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

struct MouseState
{
    float initial_mousex{ 0 };          // set when mouse transitions from not dragging to dragging
    float initial_mousey{ 0 };
    float mousex{ 0 };                  // current mouse position in window space
    float mousey{ 0 };
    bool  click_initiated{ false };     // true only on the frame when the mouse transitioned from not dragging to dragging
    bool  dragging{ false };            // true as long as the button is held
    bool  click_ended{ false };         // true only on the frame when the mouse transitioned from dragging to not dragging
};

void mouse_state_update(MouseState* ms,
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

struct AppState
{
    // camera and application state
    lab::camera::Camera camera;
    lab::camera::v3f initial_hit_point;
    lab::camera::PanTiltController main_pan_tilt;
    UIStateMachine ui_state = UIStateMachine::UI;
    uint64_t last_time = 0;
    MouseState mouse;
    LCNav_PanelState* navigator_panel = nullptr;

    // gizmo state
    tinygizmo::m44f gizmo_transform;
    tinygizmo::gizmo_application_state gizmo_state;
    tinygizmo::gizmo_context gizmo_ctx;

    // UI elements and flags
    bool show_navigator = true;
    bool show_look_at = true;
    bool show_state = true;
    bool show_view_plane_intersect = false;
    bool show_manip_plane_intersect = false;
    bool quit = false;

    // sokol
    sgl_pipeline gl_pipelne;
    sg_imgui_t sg_imgui;
    sg_pass_action pass_action;
} gApp;


bool update_mouseStatus_in_current_imgui_window(MouseState* mouse)
{
    // assuming the 3d viewport is the current window, fetch the content region
    ImGuiWindow* win = ImGui::GetCurrentWindow();
    ImRect edit_rect = win->ContentRegionRect;
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 imgui_cursor_pos = ImGui::GetCursorPos();
    ImGui::SetCursorScreenPos(edit_rect.Min);

    // detect that the mouse is in the content region
    bool click_finished = ImGui::InvisibleButton("###GIZMOREGION", edit_rect.GetSize());
    bool in_canvas = click_finished || ImGui::IsItemHovered();

    ImVec2 mouse_pos = io.MousePos - ImGui::GetCurrentWindow()->Pos;
    mouse_state_update(mouse, mouse_pos.x, mouse_pos.y, io.MouseDown[0] && io.MouseDownOwned[0]);

    // restore the ImGui state
    ImGui::SetCursorPos(imgui_cursor_pos);
    return in_canvas;
}

/*-----------------------------------------------------------------------------
    Miscellaneous graphics helpers
 */

static void start_gl_rendering()
{
    static std::once_flag once;
    std::call_once(once, []()
    {
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
    });

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

    lab::camera::m44f view = gApp.camera.mount.gl_view_transform();
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
    gApp.navigator_panel = create_navigator_panel();

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

void shutdown_graphics() 
{
    simgui_shutdown();
    sg_imgui_discard(&gApp.sg_imgui);
    sgl_shutdown();
    sg_shutdown();

    release_navigator_panel(gApp.navigator_panel);
    gApp.navigator_panel = nullptr;
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
    std::vector<float>    vertices;
};
static GizmoTriangles gizmo_triangles;

// return true if the gizmo was interacted
bool run_gizmo(MouseState* ms, float width, float height)
{
    auto& cmt = gApp.camera.mount.transform();
    lab::camera::v3f camera_pos = cmt.position;
    lab::camera::Ray ray = gApp.camera.get_ray_from_pixel({ ms->mousex, ms->mousey }, { 0, 0 }, { width, height });
    lab::camera::quatf camera_orientation = cmt.orientation;

    gApp.gizmo_state.mouse_left = ms->dragging;
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
    Application logic
 */

void run_application_logic()
{
    const int window_width = sapp_width();
    const int window_height = sapp_height();
    const float w = (float)sapp_width();
    const float h = (float)sapp_height();
    const double delta_time = std::max(stm_sec(stm_laptime(&gApp.last_time)), 1./60.);

    lab::camera::PanTiltController& ptc = gApp.main_pan_tilt;

    float fovy = lab::camera::degrees_from_radians(gApp.camera.vertical_FOV());
    gApp.camera.optics.focal_length = gApp.camera.sensor.focal_length_from_vertical_FOV(lab::camera::radians_from_degrees(60));
    gApp.camera.optics.squeeze = w / h;
    lab::camera::m44f proj = gApp.camera.perspective();
    lab::camera::m44f view = gApp.camera.mount.gl_view_transform();
    lab::camera::m44f view_t = gApp.camera.mount.gl_view_transform_inv();
    lab::camera::m44f view_proj = gApp.camera.view_projection(1.f);

    auto& cmt = gApp.camera.mount.transform();
    lab::camera::v3f pos = cmt.position;

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
            lab::camera::v3f lookat = gApp.navigator_panel->pan_tilt.orbit_center_constraint();
            m.w = { lookat.x, lookat.y, lookat.z, 1.f };
            draw_jack(1, m);
        }

        m.w = { gApp.initial_hit_point.x, gApp.initial_hit_point.y, gApp.initial_hit_point.z, 1.f };
        draw_jack(0.25, m);

        // hit point on manipulator plane
        if (gApp.show_manip_plane_intersect)
        {
            lab::camera::HitResult hit = gApp.camera.hit_test(
                { gApp.mouse.mousex, gApp.mouse.mousey },
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
            lab::camera::v3f cam_pos = cmt.position;
            lab::camera::v3f cam_nrm = cmt.forward();
            cam_nrm.x *= -1.f;
            cam_nrm.y *= -1.f;
            cam_nrm.z *= -1.f;
            cam_pos.x += cam_nrm.x;
            cam_pos.y += cam_nrm.y;
            cam_pos.z += cam_nrm.z;

            lab::camera::HitResult hit = gApp.camera.hit_test(
                { gApp.mouse.mousex, gApp.mouse.mousey },
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
        lab::camera::v3f cam_pos = cmt.position;
        lab::camera::v3f cam_nrm = cmt.forward();
        cam_nrm.x *= -1.f;
        cam_nrm.y *= -1.f;
        cam_nrm.z *= -1.f;
        cam_pos.x += cam_nrm.x;
        cam_pos.y += cam_nrm.y;
        cam_pos.z += cam_nrm.z;

        // project onto a plane one unit in front of the camera
        lab::camera::HitResult hit = gApp.camera.hit_test(
            { gApp.mouse.mousex, gApp.mouse.mousey },
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
    bool mouse_in_viewport = update_mouseStatus_in_current_imgui_window(&gApp.mouse);

    if (gApp.show_state)
    {
        ImGui::Begin("App State");
        ImGui::Text("state: %s", name_state(gApp.ui_state));
        lab::camera::v3f ypr = gApp.camera.mount.ypr();
        ImGui::Text("ypr: (%f, %f, %f)", ypr.x, ypr.y, ypr.z);
        lab::camera::v3f pos = cmt.position;
        ImGui::Text("pos: (%f, %f, %f)", pos.x, pos.y, pos.z);
        ImGui::Separator();
        ImGui::Text("imouse: %f, %f", gApp.mouse.initial_mousex, gApp.mouse.initial_mousey);
        ImGui::Text(" mouse: %f, %f", gApp.mouse.mousex, gApp.mouse.mousey);
        ImGui::Text(" click: %s", gApp.mouse.dragging ? "X" : "-");
        ImGui::End();
    }

    LabCameraNavigatorPanelInteraction in = LCNav_None;
    if (gApp.show_navigator)
    {
        ptc.sync_constraints(gApp.navigator_panel->pan_tilt);
        in = run_navigator_panel(gApp.navigator_panel, gApp.camera);
    }

    const lab::camera::rigid_transform rt = gApp.camera.mount.transform();
    camera_minimap(320, 240, &rt, gApp.main_pan_tilt.orbit_center_constraint());


    if (in > LCNav_None)
    {
        gApp.ui_state = UIStateMachine::None;
        run_gizmo(&gApp.mouse, (float)window_width, (float)window_height);
    }
    else if (mouse_in_viewport)
    {
        if (gApp.ui_state == UIStateMachine::Gizmo)
        {
            if (!run_gizmo(&gApp.mouse, (float)window_width, (float)window_height))
            {
                gApp.ui_state = UIStateMachine::None;
                sapp_lock_mouse(false);
            }
        }
        else if (gApp.ui_state <= UIStateMachine::UI)
        {
            if (run_gizmo(&gApp.mouse, (float)window_width, (float)window_height))
            {
                gApp.ui_state = UIStateMachine::Gizmo;
            }
            else if (gApp.mouse.click_initiated)
            {
                // hit test versus the gizmo's plane
                lab::camera::HitResult hit = gApp.camera.hit_test(
                    { gApp.mouse.mousex, gApp.mouse.mousey },
                    { (float)window_width, (float)window_height },
                    *(lab::camera::v3f*)(&gApp.gizmo_transform.w),
                    *(lab::camera::v3f*)(&gApp.gizmo_transform.y));

                if (gApp.mouse.click_initiated && gApp.ui_state <= UIStateMachine::UI)
                {
                    if (hit.hit)
                    {
                        gApp.initial_hit_point = hit.point;
                        gApp.ui_state = UIStateMachine::TTLCamera;
                    }
                    else
                    {
                        lab::camera::v3f cam_pos = cmt.position;
                        lab::camera::v3f cam_nrm = cmt.forward();
                        cam_nrm.x *= -1.f;
                        cam_nrm.y *= -1.f;
                        cam_nrm.z *= -1.f;
                        cam_pos.x += cam_nrm.x;
                        cam_pos.y += cam_nrm.y;
                        cam_pos.z += cam_nrm.z;

                        hit = gApp.camera.hit_test(
                            { gApp.mouse.mousex, gApp.mouse.mousey },
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
            if (gApp.mouse.click_initiated)
                phase = lab::camera::InteractionPhase::Start;
            else if (gApp.mouse.click_ended)
                phase = lab::camera::InteractionPhase::Finish;

            ImGui::CaptureMouseFromApp(true);

            if (gApp.ui_state == UIStateMachine::TTLCamera)
            {
                ImGui::SetCursorScreenPos(ImVec2{ mouse_pos.x, mouse_pos.y });
                ImGui::TextUnformatted("TTL+");

                // through the lens mode
                lab::camera::InteractionToken tok = ptc.begin_interaction(viewport);
                ptc.constrained_ttl_interaction(
                    gApp.camera,
                    tok, phase, gApp.navigator_panel->camera_interaction_mode,
                    { mouse_pos.x, mouse_pos.y },
                    gApp.initial_hit_point);
                ptc.end_interaction(tok);
            }
            else
            {
                ImGui::SetCursorScreenPos(ImVec2{ mouse_pos.x, mouse_pos.y });
                ImGui::TextUnformatted("TTL");

                // virtual joystick mode
                lab::camera::InteractionToken tok = ptc.begin_interaction(viewport);
                ptc.ttl_interaction(
                    gApp.camera,
                    tok, phase, gApp.navigator_panel->camera_interaction_mode,
                    { mouse_pos.x, mouse_pos.y });
                ptc.end_interaction(tok);
            }

            if (gApp.mouse.click_ended)
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
