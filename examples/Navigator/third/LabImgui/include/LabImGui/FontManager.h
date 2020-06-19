
#pragma once

#include <imgui.h>

namespace lab {
    namespace ImGui {

        enum class FontName : int
        {
            Default = 0, Regular, Mono, MonoSmall, Icon, COUNT
        };

        class FontManager
        {
            char* _resource_path;
            ImFont* _fonts[(int)FontName::COUNT];
            FontManager() = delete;
        public:
            FontManager(char const* const resource_path);
            ImFont* GetFont(FontName);
        };

        class SetFont
        {
        public:
            SetFont(ImFont* f) { ::ImGui::PushFont(f); }
            ~SetFont() { ::ImGui::PopFont(); }
        };

    }
} // lab::ImGui
