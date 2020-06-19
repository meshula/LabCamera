
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "imgui_internal.h"
#include <cmath>

void Header(char const* label, ImU32 color, ImVec2 padding)
{
    ImVec2 label_size = ImGui::CalcTextSize(label);
    ImVec2 region_size = ImGui::GetWindowSize() + ImVec2(-padding.x, padding.y * 2);
    ImVec2 p0 = ImGui::GetCursorPos();
    ImVec2 pos = p0 + ImGui::GetWindowPos();
    ImDrawList* pDrawList = ImGui::GetWindowDrawList();
    pDrawList->AddRectFilled(pos, pos + region_size, color);
    ImGui::SetCursorPos(p0 + padding);
    ImGui::TextUnformatted(label);
    ImGui::SetCursorPosY(std::floor(ImGui::GetCursorPosY() + padding.y / 2));
}
