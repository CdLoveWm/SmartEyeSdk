#pragma once
#include <string>

// 系统路径分隔符
#ifdef _WIN32
const std::string os_pathsep("\\");
#else
const std::string os_pathsep("/");
#endif