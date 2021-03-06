#pragma once
#include <windows.h>
#include <stdio.h>
#include <fvec.h>
#include <iostream>

using namespace std;

#ifdef _DEBUG
# define Debug(fmtstr, ...) printf(fmtstr, ##__VA_ARGS__)
#else
# define Debug(fmtstr, ...)
#endif

typedef F32vec4 vec4;