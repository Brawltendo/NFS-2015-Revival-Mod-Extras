#pragma once
#include <stdint.h>
#include <vector>
#include "math/vectormath.h"
#include <sstream>


// Native function offsets
#define DebugRenderManager_getThreadContext 0x1434171D0
#define DebugRenderer_getDebugFontSize 0x143419D20
#define DebugRenderer_getScreenRes 0x14341A800
#define DebugRenderer_writeCachedVertices 0x14341D640
#define DebugRenderer_drawLine2d 0x1434193C0
#define DebugRenderer_drawLineRect2d 0x143419440
#define DebugRenderer_drawText_0 0x1434196C0
#define DebugRenderer_drawText_1 0x143419820

#pragma region RevivalDebug

extern std::stringstream debug_controlledDriftStr;
extern std::stringstream debug_driftEntryReasonStr;
extern __m128 debug_carPos;
extern __m128 debug_sideForceWorldPos;
extern __m128 debug_fwdForceWorldPos;

#pragma endregion RevivalDebug


namespace fb
{

	struct Color32
	{
		union
		{
			struct
			{
				uint8_t r;
				uint8_t g;
				uint8_t b;
				uint8_t a;
			};
			uint32_t data;
		};

		// Initializes color to white
		Color32() { r = g = b = a = 255; }

		Color32(uint8_t inR, uint8_t inG, uint8_t inB, uint8_t inA)
		{
			r = inR;
			g = inG;
			b = inB;
			a = inA;
		}

		Color32(uint8_t inR, uint8_t inG, uint8_t inB)
		{
			r = inR;
			g = inG;
			b = inB;
			a = 255;
		}

		Color32(uint32_t color)
		{
			data = color;
		}

		//// Initializes color with normalized RGBA values
		//Color32(float inR, float inG, float inB, float inA)
		//{
		//	r = inR * 255;
		//	g = inG * 255;
		//	b = inB * 255;
		//	a = inA * 255;
		//}

		//// Initializes color with normalized RGB values
		//Color32(float inR, float inG, float inB)
		//{
		//	r = inR * 255;
		//	g = inG * 255;
		//	b = inB * 255;
		//	a = 255;
		//}

	};

	struct DebugRenderVertex
	{
		float pos[3];
		Color32 color;
		float normal[3];
		uint32_t pad;
	};

	enum DebugGeometryType
	{
		DebugGeometryType_Triangle3d,
		DebugGeometryType_Line3d,
		DebugGeometryType_Triangle2d,
		DebugGeometryType_Line2d,
		DebugGeometryType_Font2d,
		DebugGeometryType_Count
	};

	class DebugRenderer
	{
	public:

		// Initializes the debug renderer with a hook
		static void Init();

		// Retrieves the native debug renderer from the thread context
		void* GetNativeDebugRender();

		// Draws all debug data to the screen every frame
		void Draw();

		// Gets the base size of the debug renderer's font
		int* getDebugFontSize(void* nativeDbgRender, int* outSize);

		// Gets the current screen resolution
		int* getScreenRes(void* nativeDbgRender, int* arr);

		DebugRenderVertex* writeCachedVertices(DebugGeometryType type, uint32_t vertexCount);

		// Draws a line to the screen in 2D space
		void drawLine2d(Vec2* start, Vec2* end, Color32 color);

		// Draws a line to the screen in 3D space
		void drawLine3d(float start[], float end[], Color32 color);

		// Draws a line box to the screen in 2D space
		void drawLineRect2d(Vec2* start, Vec2* end, Color32 color);
		
		// Draws text to the screen
		void drawText(int x, int y, const char* text, Color32 color, float scale);

		// Draws text to the screen using normalized screen coords
		void drawText(float x, float y, const char* text, Color32 color, float scale);

		// Draws formatted text to the screen
		void drawText(int x, int y, Color32 color, const char* format, ...);

	};

	static DebugRenderer* g_debugRender;
}
