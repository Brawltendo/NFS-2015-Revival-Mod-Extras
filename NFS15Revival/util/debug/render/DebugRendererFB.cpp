#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include "DebugRendererFB.h"
#include <cstdarg>
#include <d3d11.h>
#include <assert.h>
#include <time.h>
#include <iomanip>
#include <sstream>
#include "util/utils.h"
#include "util/memoryutils.h"
#include "debug/debug.h"


#pragma region RevivalDebug

std::stringstream debug_controlledDriftStr;
std::stringstream debug_driftEntryReasonStr;
__m128 debug_carPos = Vec4::kZero.simdValue;
__m128 debug_sideForceWorldPos = Vec4::kZero.simdValue;
__m128 debug_fwdForceWorldPos = Vec4::kZero.simdValue;

#pragma endregion RevivalDebug

void __fastcall fb__PerfOverlay__drawFcat(void* perfOverlay, struct IRenderContext* renderContext)
{
	fb::g_debugRender->Draw();
}

void __fastcall fb__PerfOverlay__drawFps(void* perfOverlay)
{
	fb::g_debugRender->Draw();
}

void fb::DebugRenderer::Init()
{
	fb::g_debugRender = new fb::DebugRenderer();
	debug_controlledDriftStr.str(std::string());
	debug_driftEntryReasonStr.str(std::string());
	InjectHook(0x143B6ABF0, fb__PerfOverlay__drawFcat);
	//InjectHook(0x143B6AE10, fb__PerfOverlay__drawFps);
}

void* fb::DebugRenderer::GetNativeDebugRender()
{
	static void* debugRenderer = nullptr;

	if (debugRenderer == nullptr)
	{
		void*(__fastcall* native)() = reinterpret_cast<void* (__fastcall*)()>(DebugRenderManager_getThreadContext);
		debugRenderer = native();
		return debugRenderer;
	}

	return debugRenderer;
}

void fb::DebugRenderer::Draw()
{
	static bool isNumAddAlreadyDown = false;

	void* debugRender = GetNativeDebugRender();
	if (debugRender)
	{
		// show time
		{
			time_t t;
			time(&t);
			std::stringstream str;
			// you're supposed to be able to use [[color:]] tags to color parts of the text but that code never gets hit at least from what I was able to check
			//str << "[[color:Green]]";
			str << std::put_time(localtime(&t), "%x %X");
			const char* blazeTitleId = (const char*)0x1421F2558;
			str << " Br:revival-v4-test1"; str << " Blz:Test:"; str << blazeTitleId; //str << "\nDebugRenderAddress:"; str << std::hex; str << debugRender;

			//int arr[2] = { 0 };
			//fb::g_debugRender->getDebugFontSize(debugRender, arr);
			//int textSize = 4 * arr[0] * (1.f + 0.5f);
			//fb::g_debugRender->getScreenRes(debugRender, arr);
			fb::g_debugRender->drawText(0, 0, str.str().c_str(), fb::Color32(255u, 255u, 255u, 255u), 1.f);
		}

		// do Revival stuff
		{
			fb::g_debugRender->drawText(-0.5f, 0.15f, debug_controlledDriftStr.str().c_str(), fb::Color32(0u, 255u, 0u, 255u), 1.f);

			uint8_t alpha = /*showTextTimer / 3.f * */255u;
			fb::g_debugRender->drawText(-0.5f, 0.f, debug_driftEntryReasonStr.str().c_str(), fb::Color32(0u, 255u, 0u, alpha), 1.f);
			//showTextTimer = fmaxf(showTextTimer - nfsVehicle.m_currentUpdateDt, 0.f);

			// draw side force
			fb::g_debugRender->drawLine3d(debug_carPos.m128_f32, debug_sideForceWorldPos.m128_f32, fb::Color32(255u, 0u, 0u, 255u));
			// draw forward force
			fb::g_debugRender->drawLine3d(debug_carPos.m128_f32, debug_fwdForceWorldPos.m128_f32, fb::Color32(0u, 255u, 0u, 255u));


			// numpad add
			/*SHORT keyState = GetKeyState(VK_ADD);
			bool numAddDown = keyState & 0x8000;

			if (numAddDown && !isNumAddAlreadyDown)
			{
				struct ConsoleRegistry
				{
					char buf[256];
				} console{};

				void(__fastcall* executeConsoleCommand)(ConsoleRegistry*, const char*, bool) = reinterpret_cast<void(__fastcall*)(ConsoleRegistry*, const char*, bool)>(0x1433FD330);
				executeConsoleCommand(&console, "AddCash", true);
				DebugLogPrint("You got 100k added to your bank.\n");
			}
			isNumAddAlreadyDown = numAddDown;*/
		}
	}
}

int* fb::DebugRenderer::getDebugFontSize(void* nativeDbgRender, int* outSize)
{
	int*(__fastcall* native)(void*, int*) = reinterpret_cast<int*(__fastcall*)(void*, int*)>(DebugRenderer_getDebugFontSize);

	return native(nativeDbgRender, outSize);
}

int* fb::DebugRenderer::getScreenRes(void* nativeDbgRender, int* outSize)
{
	int*(__fastcall* native)(void*, int*) = reinterpret_cast<int*(__fastcall*)(void*, int*)>(DebugRenderer_getScreenRes);

	return native(nativeDbgRender, outSize);
}

fb::DebugRenderVertex* fb::DebugRenderer::writeCachedVertices(fb::DebugGeometryType type, uint32_t vertexCount)
{
	DebugRenderVertex*(__fastcall* native)(void*, fb::DebugGeometryType, uint32_t) = reinterpret_cast<DebugRenderVertex*(__fastcall*)(void*, fb::DebugGeometryType, uint32_t)>(DebugRenderer_writeCachedVertices);

	void* debugRender = GetNativeDebugRender();
	if (debugRender != nullptr)
		return native(debugRender, type, vertexCount);
	else
		return nullptr;
}

void fb::DebugRenderer::drawLine2d(Vec2* start, Vec2* end, fb::Color32 color)
{
	void(__fastcall* native)(void*, Vec2*, Vec2*, Color32) = reinterpret_cast<void(__fastcall*)(void*, Vec2*, Vec2*, Color32)>(DebugRenderer_drawLine2d);

	void* debugRender = GetNativeDebugRender();
	if (debugRender != nullptr)
		native(debugRender, start, end, color);

}

void fb::DebugRenderer::drawLine3d(float start[], float end[], fb::Color32 color)
{
	DebugRenderVertex* vtx = writeCachedVertices(DebugGeometryType_Line3d, 2);
	
	vtx->pos[0] = start[0]; vtx->pos[1] = start[1]; vtx->pos[2] = start[2];
	vtx->color = color;
	++vtx;

	vtx->pos[0] = end[0]; vtx->pos[1] = end[1]; vtx->pos[2] = end[2];
	vtx->color = color;
	++vtx;
}

void fb::DebugRenderer::drawLineRect2d(Vec2* min, Vec2* max, fb::Color32 color)
{
	void(__fastcall* native)(void*, Vec2*, Vec2*, Color32) = reinterpret_cast<void(__fastcall*)(void*, Vec2*, Vec2*, Color32)>(DebugRenderer_drawLineRect2d);

	void* debugRender = GetNativeDebugRender();
	if (debugRender != nullptr)
		native(debugRender, min, max, color);
}

void fb::DebugRenderer::drawText(int x, int y, const char* text, fb::Color32 color, float scale)
{
	void(__fastcall* native)(void*, int, int, const char*, Color32, float) = reinterpret_cast<void(__fastcall*)(void*, int, int, const char*, Color32, float)>(DebugRenderer_drawText_0);

	void* debugRender = GetNativeDebugRender();
	if (debugRender != nullptr)
		native(debugRender, x, y, text, color, scale);
}

void fb::DebugRenderer::drawText(float x, float y, const char* text, fb::Color32 color, float scale)
{
	void(__fastcall * native)(void*, int, int, const char*, Color32, float) = reinterpret_cast<void(__fastcall*)(void*, int, int, const char*, Color32, float)>(DebugRenderer_drawText_0);

	void* debugRender = GetNativeDebugRender();
	if (debugRender != nullptr)
	{
		int res[2] = { 0 };
		getScreenRes(debugRender, res);
		Vec2 pos((x + 1.f) * res[0] * 0.5f, (y + 1.f) * res[1] * 0.5f);

		native(debugRender, (int)pos.x, (int)pos.y, text, color, scale);
	}
}

void fb::DebugRenderer::drawText(int x, int y, Color32 color, const char* format, ...)
{
	void(__fastcall* native)(void*, int, int, Color32, const char*, va_list) = reinterpret_cast<void(__fastcall*)(void*, int, int, Color32, const char*, va_list)>(DebugRenderer_drawText_1);

	void* debugRender = GetNativeDebugRender();
	if (debugRender != nullptr)
	{
		va_list args;
		// set up variadic args here to pass into the native function
		va_start(args, format);
		native(debugRender, x, y, color, format, args);
		va_end(args);
	}
}
