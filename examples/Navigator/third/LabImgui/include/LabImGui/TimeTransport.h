#pragma once

#include <array>

namespace lab
{
	namespace ImGui {
		class FontManager;
	}

	struct TimeTransport
	{
		std::array<float, 2> position = { 300, 200 };
		std::array<float, 2> size = { 400, 100 };
		void ui(lab::ImGui::FontManager&);
	};
	void time_transport(lab::ImGui::FontManager&);
}