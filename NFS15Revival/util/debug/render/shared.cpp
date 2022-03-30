#define _CRT_SECURE_NO_WARNINGS

#include "shared.h"
#include <stdio.h>
#include "imgui/imgui.h"

void impl::showExampleWindow(const char* comment)
{
	char buffer[128];
	::memset(buffer, 0, 128);
	::sprintf_s(buffer, "Kiero Dear ImGui Example (%s)", comment);

	ImGui::Begin(buffer);

	ImGuiIO& io = ImGui::GetIO();
	ImGui::Text("Hello");
	ImGui::Button("World!");

	if (ImGui::IsAnyItemHovered())
		io.WantCaptureMouse = true;
	else
		io.WantCaptureMouse = false;

	ImGui::End();
}