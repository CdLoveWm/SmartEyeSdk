#pragma once
#include <string>

// ϵͳ·���ָ���
#ifdef _WIN32
const std::string os_pathsep("\\");
#else
const std::string os_pathsep("/");
#endif