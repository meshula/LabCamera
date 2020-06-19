
#include "LabImGui/FontManager.h"

#include <stdint.h>
#include "LabImGui/roboto_regular.ttf.h"
#include "LabImGui/robotomono_regular.ttf.h"
#include "LabImGui/meshula-icons.h"
#include <stdlib.h>
#include <string>

namespace lab {
    namespace ImGui {

        FontManager::FontManager(char const* const resource_dir)
        {
            size_t i = strlen(resource_dir);
            _resource_path = (char*)malloc(i + 1);
            strcpy(_resource_path, resource_dir);
            for (int i = 0; i < (int) FontName::COUNT; ++i)
                _fonts[i] = nullptr;

            GetFont(FontName::Default); // initialize memory right away
        }

        ImFont* FontManager::GetFont(FontName font_name)
        {
            int index = (int)font_name;
            if (index >= (int)FontName::COUNT)
                index = (int)FontName::Default;

            if (_fonts[index])
                return _fonts[index];

            ImFontConfig config;
            config.FontDataOwnedByAtlas = false;
            config.MergeMode = false;

            ::ImGuiIO& io = ::ImGui::GetIO();
            _fonts[(int)FontName::Default] = io.Fonts->AddFontDefault(&config);
            _fonts[(int)FontName::Regular] = io.Fonts->AddFontFromMemoryTTF((void*)s_robotoRegularTtf, sizeof(s_robotoRegularTtf), 20, &config);
            _fonts[(int)FontName::Mono] = io.Fonts->AddFontFromMemoryTTF((void*)s_robotoMonoRegularTtf, sizeof(s_robotoMonoRegularTtf), 20.0f, &config);
            _fonts[(int)FontName::MonoSmall] = io.Fonts->AddFontFromMemoryTTF((void*)s_robotoMonoRegularTtf, sizeof(s_robotoMonoRegularTtf), 16.f, &config);

            ImFontConfig iconFontCfg;
            iconFontCfg.PixelSnapH = true;

            //static const ImWchar iconFontRanges[] = { (ImWchar) MI_ICON_MIN, (ImWchar) MI_ICON_MAX, 0 };
            //iconFontCfg.MergeMode = true;
            //fonts[(int) FontName::Icon] = io.Fonts->AddFontFromFileTTF(
            //    (resource_path / "fonts" / "meshula-icons" / "meshula-icons.ttf").string().c_str(), 32, &iconFontCfg, iconFontRanges);

            std::string path(_resource_path);
            path += "/fonts/meshula-icons.ttf";
            float pixel_size = 32;
            _fonts[(int)FontName::Icon] = io.Fonts->AddFontFromFileTTF(path.c_str(), pixel_size, &iconFontCfg);

            //ImGui_ImplOpenGL3_CreateFontsTexture();
            return _fonts[index];
        }


    }
} // lab::ImGui
