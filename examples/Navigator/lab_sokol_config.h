#pragma once
#ifndef LAB_SOKOL_CONFIG_H
#define LAB_SOKOL_CONFIG_H

void lab_sokol_log(const char* s);
#define SOKOL_LOG(s) { lab_sokol_log(s); }

#endif
