
#pragma once

#include <imgui.h>

namespace lab {
    class FontManager;

    namespace imgui
    {
        bool BeginTimeline(const char* str_id, float column_pixel_offset, float max_time_value,
                           double* time_in, double* time_out,
                           lab::FontManager*);
        bool TimelineEvent(const char* str_id, double& val1, double& val2);
        void EndTimeline(int num_vertical_lines, float current_time, ImU32 timeline_indicator_color);
    }

}
