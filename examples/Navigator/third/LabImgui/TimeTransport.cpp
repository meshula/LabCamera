
#include "LabImGui/TimeTransport.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "LabImGui/FontManager.h"

namespace lab
{
    using namespace ImGui;

    void DrawTriangle(ImGuiWindow* w, float x, float y, float sz, float flip)
    {
        ImVec2 play[3] = { { x - flip * sz * 0.5f, y },
                           { x - flip * sz * 0.5f, y + sz },
                           { x + flip * sz * 0.5f, y + sz * 0.5f } };
        w->DrawList->AddConvexPolyFilled(play, 3, ::ImGui::GetColorU32(ImGuiCol_Text));
    }

	void TimeTransport::ui(lab::ImGui::FontManager& fm)
	{
        constexpr float bump_size = 30;
        const float half_w = size[0] * 0.5f;
        ImGuiWindow* window = ::ImGui::GetCurrentWindow();
        ImVec2 ul { position[0], position[1] };
        ImVec2 lr { ul.x + size[0], ul.y + size[1] };
        ::ImGui::PushClipRect({ ul.x, ul.y - bump_size }, { lr.x, lr.y + 40 }, false);
        ::ImGui::SetCursorPos(ul);

        const float rounding = 2.0f;
        const ImU32 fill_col = ::ImGui::GetColorU32(ImGuiCol_ButtonActive);
        
        // badge
		window->DrawList->AddRectFilled(ul, lr, fill_col, rounding);
        window->DrawList->AddCircleFilled({ ul.x + half_w, ul.y }, bump_size, fill_col, 16);
        window->DrawList->AddRectFilled({ ul.x + 20, lr.y }, { lr.x - 20, lr.y + 40 }, fill_col, rounding);

        // times
        SetFont icon(fm.GetFont(FontName::MonoSmall));
        const char* start_time = "0:00:00.0";
        const ImVec2 label_size = ::ImGui::CalcTextSize(start_time, start_time + 9, false);
        window->DrawList->AddText({ ul.x + 22, lr.y + 5 }, ::ImGui::GetColorU32(ImGuiCol_Text), start_time, start_time + 9);
        const char* end_time = "0:03:14.1";
        window->DrawList->AddText({ lr.x - 22 - label_size.x, lr.y + 5 }, ::ImGui::GetColorU32(ImGuiCol_Text), end_time, end_time + 9);
        const char* curr_time = "0:00:05.2";
        window->DrawList->AddText({ ul.x + (size[0] - label_size.x) * 0.5f, lr.y + 5 }, ::ImGui::GetColorU32(ImGuiCol_Text), curr_time, curr_time + 9);

        // controls
        DrawTriangle(window, ul.x + half_w, ul.y + 8, 45, 1);
        DrawTriangle(window, ul.x + half_w + 60, ul.y + 18, 28, 1);
        DrawTriangle(window, ul.x + half_w + 100, ul.y + 22, 22, 1);
        DrawTriangle(window, ul.x + half_w - 60, ul.y + 18, 28, -1);
        DrawTriangle(window, ul.x + half_w - 100, ul.y + 22, 22, -1);

        ::ImGui::PopClipRect();
	}
	
}
