#pragma once

#ifdef _DEBUG
# define DebugLogPrint(fmtstr, ...) printf(fmtstr, ##__VA_ARGS__)
#else
# define DebugLogPrint(fmtstr, ...)
#endif

#define Stringize(x) #x
