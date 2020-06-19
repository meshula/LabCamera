

#include "LabImGui/Timeline.h"
#include "LabImGui/FontManager.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>

#include <functional>

// original source of timeline code is
// https://github.com/nem0/LumixEngine
// licensed under the MIT license.
//
// modified further from imgui issue 76
// https://github.com/ocornut/imgui/issues/76
//

static float s_max_timeline_value = 100.f;

static double s_time_offset = 0;
static double s_time_scale = 1;
static double * s_time_in = nullptr;
static double * s_time_out = nullptr;

namespace ImGui {

	bool BeginTimeline(const char* str_id, float pixel_offset, float max_value, double * time_in, double * time_out,
					   lab::FontManager * fontManager)
	{
		// reset global variables
		s_time_in = time_in;
		s_time_out = time_out;

		s_time_scale = 1.0 / (*s_time_out - *s_time_in);
		s_time_offset = *s_time_in * s_time_scale;

		ImGuiWindow * win = GetCurrentWindow();

		float height = win->ContentRegionRect.Max.y - win->ContentRegionRect.Min.y
                         - ImGui::GetTextLineHeightWithSpacing()   // space for the time bar
                         - ImGui::GetTextLineHeightWithSpacing();  // space for horizontal scroller

		bool rv = BeginChild(str_id, ImVec2(0, height), false);

		//ImGui::PushStyleColor(ImGuiCol_Column, GImGui->Style.Colors[ImGuiCol_Border]);
		ImGui::Columns(2, str_id); /// @TODO Columns should take a pointer to an array of pixel_offsets so resizing columns is persistent

		static float _pixel_offset = 0;
		if (pixel_offset != _pixel_offset) {
			_pixel_offset = pixel_offset;
			ImGui::SetColumnOffset(1, pixel_offset);
		}
		s_max_timeline_value = max_value >= 0 ? max_value : (ImGui::GetWindowContentRegionWidth() * 0.85f);
		return rv;
	}


	static const float TIMELINE_RADIUS = 12;


	bool TimelineEvent(const char* str_id, double & val1, double & val2)
	{
		double values[2] = { val1, val2 };
		ImGuiWindow* win = GetCurrentWindow();
		const ImU32 inactive_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Button]);
		const ImU32 active_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_ButtonHovered]);
		const ImU32 line_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Text]);
		bool changed = false;
		bool hovered = false;
		bool active = false;

		ImGui::Text("%s", str_id);
		ImGui::NextColumn();

		const float columnOffset = ImGui::GetColumnOffset(1);
		const float columnWidth = ImGui::GetColumnWidth(1) - GImGui->Style.ScrollbarSize;

		ImVec2 cursor_pos(GetWindowContentRegionMin().x + win->Pos.x + columnOffset - TIMELINE_RADIUS, win->DC.CursorPos.y);

		float posx[2] = { 0,0 };

		for (int i = 0; i < 2; ++i)
		{
			ImVec2 pos = cursor_pos;
			pos.x += float(s_time_scale) * columnWidth * float(values[i]) / s_max_timeline_value - columnWidth * float(s_time_offset) + TIMELINE_RADIUS;
			pos.y += TIMELINE_RADIUS;
			posx[i] = pos.x;

			SetCursorScreenPos(pos - ImVec2(TIMELINE_RADIUS, TIMELINE_RADIUS));
			PushID(i);
			InvisibleButton(str_id, ImVec2(2 * TIMELINE_RADIUS, 2 * TIMELINE_RADIUS));
			active = IsItemActive();
			if (active || IsItemHovered())
			{
				ImGui::SetTooltip("%f", float(values[i]));
				ImVec2 a(pos.x, GetWindowContentRegionMin().y + win->Pos.y + win->Scroll.y);

				ImGuiWindow * parent_win = win->ParentWindow;
				float endy = parent_win->ContentRegionRect.Max.y + win->Pos.y; // draw all the way to the bottom of the parent window

				ImVec2 b(pos.x, endy);
				win->DrawList->AddLine(a, b, line_color);
				hovered = true;
			}
			if (IsItemActive() && IsMouseDragging(0))
			{
				values[i] += GetIO().MouseDelta.x / win->Size.x * s_max_timeline_value;
				changed = hovered = true;
			}
			PopID();
			win->DrawList->AddCircleFilled(
				pos, TIMELINE_RADIUS, IsItemActive() || IsItemHovered() ? active_color : inactive_color);
		}

		ImVec2 start = cursor_pos;
		start.x = posx[0];
		start.y += TIMELINE_RADIUS * 0.5f;
		ImVec2 end = start;
		end.x = posx[1];
		end.y += TIMELINE_RADIUS;

		PushID(-1);
		SetCursorScreenPos(start);
		InvisibleButton(str_id, end - start);
		if (IsItemActive() && IsMouseDragging(0))
		{
			values[0] += GetIO().MouseDelta.x / win->Size.x * s_max_timeline_value;
			values[1] += GetIO().MouseDelta.x / win->Size.x * s_max_timeline_value;
			changed = hovered = true;
		}
		PopID();

		SetCursorScreenPos(cursor_pos + ImVec2(0, GetTextLineHeightWithSpacing()));

		start.x += TIMELINE_RADIUS * 0.5f;
		end.x -= TIMELINE_RADIUS * 0.5f;
		win->DrawList->AddRectFilled(start, end, IsItemActive() || IsItemHovered() ? active_color : inactive_color);

		if (values[0] > values[1])
		{
			std::swap(values[0], values[1]);
		}

		if (hovered)
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

		ImGui::NextColumn();

		val1 = values[0];
		val2 = values[1];

		return changed;
	}


	void EndTimeline(int num_vertical_grid_lines, float current_time, ImU32 timeline_indicator_color)
	{
		ImGui::NextColumn();

		ImGuiWindow* win = GetCurrentWindow();

		const float columnOffset = ImGui::GetColumnOffset(1);
		const float columnWidth = ImGui::GetColumnWidth(1) - GImGui->Style.ScrollbarSize;
		const float horizontal_interval = columnWidth / num_vertical_grid_lines;

		const ImU32 pz_inactive_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Button]);
		const ImU32 pz_active_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_ButtonHovered]);
		const ImU32 pz_line_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Text]);

		const ImU32 color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Button]);
		const ImU32 line_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Border]);
		const ImU32 text_color = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Text]);
		const float rounding = GImGui->Style.ScrollbarRounding;
		const float startY = ImGui::GetWindowHeight() + win->Pos.y;

		// vertical lines
		for (int i = 0; i <= num_vertical_grid_lines; ++i)
		{
			ImVec2 a = GetWindowContentRegionMin() + win->Pos;
			a.x += s_time_scale * i * horizontal_interval + columnOffset - columnWidth * s_time_offset;
			win->DrawList->AddLine(a, ImVec2(a.x, startY), line_color);
		}

		// hairline indicating current time
		{
			ImVec2 a = GetWindowContentRegionMin() + win->Pos;
			a.x += s_time_scale * current_time * columnWidth / s_max_timeline_value + columnOffset - columnWidth * s_time_offset;
			ImVec2 b = { a.x, startY };
			win->DrawList->AddLine(a, b, timeline_indicator_color);

			ImVec2 v0 = { a.x - TIMELINE_RADIUS, a.y + win->Scroll.y };
			ImVec2 v1 = { a.x + TIMELINE_RADIUS, a.y + win->Scroll.y };
			ImVec2 v2 = { a.x, a.y + TIMELINE_RADIUS + win->Scroll.y };
			win->DrawList->AddTriangleFilled(v0, v1, v2, timeline_indicator_color);

			a.x -= TIMELINE_RADIUS;
			SetCursorScreenPos(a);
			PushID(-1);
			InvisibleButton("currentTime", ImVec2(2 * TIMELINE_RADIUS, b.y - a.y));
			if (IsItemHovered())
				ImGui::SetTooltip("%f", float(current_time));
			PopID();
		}

		ImGui::Columns(1);
		//ImGui::PopStyleColor();

		EndChild();

		// draw bottom axis ribbon outside scrolling region
		win = GetCurrentWindow();

		float startx = ImGui::GetCursorScreenPos().x + columnOffset;
		float endy = GetWindowContentRegionMax().y + win->Pos.y;
		ImVec2 start(startx, endy - 2 * ImGui::GetTextLineHeightWithSpacing());
		ImVec2 end(startx + columnWidth, endy - ImGui::GetTextLineHeightWithSpacing());
		win->DrawList->AddRectFilled(start, end, color, rounding);

		char tmp[256] = "";
		for (int i = 0; i < num_vertical_grid_lines; ++i)
		{
			ImVec2 a = start;
			a.x = start.x + (float) s_time_scale * i * horizontal_interval - columnWidth * s_time_offset;

			if (a.x < startx)
				continue;

			a.y = start.y;
			ImFormatString(tmp, sizeof(tmp), "%.2f", i * s_max_timeline_value / num_vertical_grid_lines);
			win->DrawList->AddText(a, text_color, tmp);
		}


		// draw time panzoomer

		float posx[2] = { 0,0 };
		double values[2] = { *s_time_in, *s_time_out };

		double constrained_time_length = values[1] - values[0];

		bool active = false;
		bool hovered = false;
		bool changed = false;
		ImVec2 cursor_pos = { start.x, end.y };

		for (int i = 0; i < 2; ++i)
		{
			ImVec2 pos = cursor_pos;
			pos.x += columnWidth * float(values[i]) + TIMELINE_RADIUS;
			pos.y += TIMELINE_RADIUS;
			posx[i] = pos.x;

			SetCursorScreenPos(pos - ImVec2(TIMELINE_RADIUS, TIMELINE_RADIUS));
			PushID(i);
			InvisibleButton("zoompanner", ImVec2(2 * TIMELINE_RADIUS, 2 * TIMELINE_RADIUS));
			active = IsItemActive();
			if (active || IsItemHovered())
			{
				hovered = true;
			}
			if (IsItemActive() && IsMouseDragging(0))
			{
				values[i] += GetIO().MouseDelta.x / columnWidth;
				changed = hovered = true;
			}
			PopID();

			win->DrawList->AddCircleFilled(
				pos, TIMELINE_RADIUS, IsItemActive() || IsItemHovered() ? pz_active_color : pz_inactive_color);
		}

		if (values[0] > values[1])
			std::swap(values[0], values[1]);

		start.x = posx[0];
		start.y += TIMELINE_RADIUS * 0.5f + ImGui::GetTextLineHeightWithSpacing();
		end.x = posx[1];
		end.y = start.y + TIMELINE_RADIUS;

		PushID(-1);
		SetCursorScreenPos(start);
		InvisibleButton("zoompanner", end - start);
		if (IsItemActive() && IsMouseDragging(0))
		{
			values[0] += GetIO().MouseDelta.x / columnWidth;
			values[1] += GetIO().MouseDelta.x / columnWidth;
			changed = hovered = true;
		}
		PopID();

		start.x += TIMELINE_RADIUS * 0.5f;
		start.y -= TIMELINE_RADIUS * 0.5f;
		end.x -= TIMELINE_RADIUS * 0.5f;
		end.y += TIMELINE_RADIUS * 0.5f;
		win->DrawList->AddRectFilled(start, end, IsItemActive() || IsItemHovered() ? pz_active_color : pz_inactive_color);

		// and write the time in and out values back
		if (values[0] < 0)
		{
			values[0] = 0;
			values[1] = constrained_time_length;
		}
		if (values[1] > 1)
		{
			values[1] = 1;
			values[0] = 1.0 - constrained_time_length;
		}

		*s_time_in = values[0];
		*s_time_out = values[1];

		ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2 * ImGui::GetTextLineHeightWithSpacing());
	}


} // ImGui
