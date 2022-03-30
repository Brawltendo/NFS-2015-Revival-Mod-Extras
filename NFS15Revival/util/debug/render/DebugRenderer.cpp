#pragma once
#include "debugrenderer.h"
#include <Windows.h>
#include "kiero/kiero.h"


#if KIERO_INCLUDE_D3D11

#include <d3d11.h>
#include <assert.h>

#include "win32_impl.h"

#include "../imgui/imgui.h"
#include "../imgui/backends/imgui_impl_win32.h"
#include "../imgui/backends/imgui_impl_dx11.h"

typedef long(__stdcall* Present)(IDXGISwapChain*, UINT, UINT);
static Present oPresent = NULL;

typedef HRESULT(__stdcall* Present) (IDXGISwapChain* pSwapChain, UINT SyncInterval, UINT Flags);
typedef LRESULT(CALLBACK* WNDPROC)(HWND, UINT, WPARAM, LPARAM);
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace DebugRender
{

	Present oPresent;
	HWND window = NULL;
	WNDPROC oWndProc;
	ID3D11Device* pDevice = NULL;
	ID3D11DeviceContext* pContext = NULL;
	ID3D11RenderTargetView* mainRenderTargetView;

	LRESULT __stdcall WndProc(const HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {

		if (true && ImGui_ImplWin32_WndProcHandler(hWnd, uMsg, wParam, lParam))
			return true;

		return CallWindowProc(oWndProc, hWnd, uMsg, wParam, lParam);
	}

}

long __stdcall hkPresent11(IDXGISwapChain* pSwapChain, UINT SyncInterval, UINT Flags)
{
	static bool init = false;

	if (!init)
	{
		DXGI_SWAP_CHAIN_DESC desc;
		pSwapChain->GetDesc(&desc);

		pSwapChain->GetDevice(__uuidof(ID3D11Device), (void**)&DebugRender::pDevice);
		DebugRender::pDevice->GetImmediateContext(&DebugRender::pContext);

		impl::win32::init(desc.OutputWindow);

		ImGui::CreateContext();
		ImGui_ImplWin32_Init(desc.OutputWindow);
		ImGui_ImplDX11_Init(DebugRender::pDevice, DebugRender::pContext);

		init = true;
	}

	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	impl::showExampleWindow("D3D11");

	ImGui::EndFrame();
	ImGui::Render();
	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

	return oPresent(pSwapChain, SyncInterval, Flags);
}

typedef HRESULT(__stdcall* ResizeBuffers)(IDXGISwapChain* pThis, UINT BufferCount, UINT Width, UINT Height, DXGI_FORMAT NewFormat, UINT SwapChainFlags);
ResizeBuffers oResizeBuffers;
HRESULT hkResizeBuffers(IDXGISwapChain* pThis, UINT BufferCount, UINT Width, UINT Height, DXGI_FORMAT NewFormat, UINT SwapChainFlags) {
	if (DebugRender::mainRenderTargetView) {
		DebugRender::pContext->OMSetRenderTargets(0, 0, 0);
		DebugRender::mainRenderTargetView->Release();
	}

	HRESULT hr = oResizeBuffers(pThis, BufferCount, Width, Height, NewFormat, SwapChainFlags);

	ID3D11Texture2D* pBuffer;
	pThis->GetBuffer(0, __uuidof(ID3D11Texture2D),
		(void**)&pBuffer);
	// Perform error handling here!

	DebugRender::pDevice->CreateRenderTargetView(pBuffer, NULL,
		&DebugRender::mainRenderTargetView);
	// Perform error handling here!
	pBuffer->Release();

	DebugRender::pContext->OMSetRenderTargets(1, &DebugRender::mainRenderTargetView, NULL);

	// Set up the viewport.
	D3D11_VIEWPORT vp;
	vp.Width = Width;
	vp.Height = Height;
	vp.MinDepth = 0.0f;
	vp.MaxDepth = 1.0f;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	DebugRender::pContext->RSSetViewports(1, &vp);
	return hr;
}

void impl::d3d11::init()
{
	assert(kiero::bind(8, (void**)&oPresent, hkPresent11) == kiero::Status::Success);
	//assert(kiero::bind(11, (void**)&oResizeBuffers, hkResizeBuffers) == kiero::Status::Success);
}

#endif // KIERO_INCLUDE_D3D11

int DebugRenderer::Init()
{
    if (kiero::init(kiero::RenderType::Auto) == kiero::Status::Success)
    {
        if (kiero::getRenderType() == kiero::RenderType::D3D11)
        {
            impl::d3d11::init();
            return 1;
        }
    }
    return 0;
}
