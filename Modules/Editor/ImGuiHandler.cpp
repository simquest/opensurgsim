#include "ImGuiHandler.h"

#include "imgui.h"

#include <iostream>

using namespace sf;

// This is the main rendering function that you have to implement and provide to ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by (0.5f,0.5f) or (0.375f,0.375f)
void ImGui_RenderDrawLists(ImDrawData* draw_data)
{
	// Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
	ImGuiIO& io = ImGui::GetIO();
	int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (fb_width == 0 || fb_height == 0)
	{
		return;
	}
	draw_data->ScaleClipRects(io.DisplayFramebufferScale);

	// We are using the OpenGL fixed pipeline to make the example code simpler to read!
	// Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, vertex/texcoord/color pointers.
	GLint last_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	GLint last_viewport[4];
	glGetIntegerv(GL_VIEWPORT, last_viewport);
	GLint last_scissor_box[4];
	glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_SCISSOR_TEST);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnable(GL_TEXTURE_2D);
	//glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

	// Setup viewport, orthographic projection matrix
	glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// Render command lists
#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
	for (int n = 0; n < draw_data->CmdListsCount; n++)
	{
		const ImDrawList* cmd_list = draw_data->CmdLists[n];
		const ImDrawVert* vtx_buffer = cmd_list->VtxBuffer.Data;
		const ImDrawIdx* idx_buffer = cmd_list->IdxBuffer.Data;
		glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, pos)));
		glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, uv)));
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (void*)((char*)vtx_buffer + OFFSETOF(ImDrawVert, col)));

		for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
		{
			const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
			if (pcmd->UserCallback)
			{
				pcmd->UserCallback(cmd_list, pcmd);
			}
			else
			{
				glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
				glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x),
						  (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
				glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT,
							   idx_buffer);
			}
			idx_buffer += pcmd->ElemCount;
		}
	}
#undef OFFSETOF

	// Restore modified state
// 	glDisableClientState(GL_COLOR_ARRAY);
// 	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
// 	glDisableClientState(GL_VERTEX_ARRAY);
	glBindTexture(GL_TEXTURE_2D, (GLuint)last_texture);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
	glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3]);

}

struct ImGuiNewFrameCallback : public osg::Camera::DrawCallback
{

	ImGuiNewFrameCallback(ImGuiHandler& theHandler)
		: m_handler(theHandler)
	{}

	virtual void operator()(osg::RenderInfo& theRenderInfo) const
	{
		m_handler.newFrame(theRenderInfo);
	}

private:

	ImGuiHandler& m_handler;

};

struct ImGuiDrawCallback : public osg::Camera::DrawCallback
{

	ImGuiDrawCallback(ImGuiHandler& theHandler)
		: m_handler(theHandler)
	{}

	virtual void operator()(osg::RenderInfo& theRenderInfo) const
	{
		m_handler.render(theRenderInfo);
	}

private:

	ImGuiHandler& m_handler;

};

ImGuiHandler::ImGuiHandler(GuiCallback* theGuicallback)
	: m_callback(theGuicallback)
{

	g_Time = 0.0f;
	for (int i = 0; i < 3; ++i)
	{
		g_MousePressed[i] = false;
	}
	g_MouseWheel = 0.0f;
	g_FontTexture = 0;
}

void ImGuiHandler::init()
{

	ImGuiIO& io = ImGui::GetIO();

	// Build texture atlas
	unsigned char* pixels;
	int width, height;
	io.Fonts->GetTexDataAsAlpha8(&pixels, &width, &height);

	// Create OpenGL texture
	GLint last_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGenTextures(1, &g_FontTexture);
	glBindTexture(GL_TEXTURE_2D, g_FontTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, width, height, 0, GL_ALPHA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	io.Fonts->TexID = (void*)(intptr_t)g_FontTexture;

	// Cleanup (don't clear the input data if you want to append new fonts later)
	io.Fonts->ClearInputData();
	io.Fonts->ClearTexData();
	glBindTexture(GL_TEXTURE_2D, last_texture);

	io.RenderDrawListsFn = ImGui_RenderDrawLists;
}

void ImGuiHandler::setCameraCallbacks(osg::Camera* theCamera)
{

	ImGuiDrawCallback* aPostDrawCallback = new ImGuiDrawCallback(*this);
	theCamera->setPostDrawCallback(aPostDrawCallback);

	ImGuiNewFrameCallback* aPreDrawCallback = new ImGuiNewFrameCallback(*this);
	theCamera->setPreDrawCallback(aPreDrawCallback);
}

void ImGuiHandler::newFrame(osg::RenderInfo& theRenderInfo)
{

	if (!g_FontTexture)
	{
		init();
	}

	ImGuiIO& io = ImGui::GetIO();

	osg::Viewport* aViewport = theRenderInfo.getCurrentCamera()->getViewport();
	io.DisplaySize = ImVec2(aViewport->width(), aViewport->height());

	double aCurrentTime = theRenderInfo.getView()->getFrameStamp()->getSimulationTime();
	io.DeltaTime = g_Time > 0.0 ? (float)(aCurrentTime - g_Time) : (float)(1.0f / 60.0f);
	g_Time = aCurrentTime;

	for (int i = 0; i < 3; i++)
	{
		io.MouseDown[i] = g_MousePressed[i];
		//    g_MousePressed[i] = false;
	}

	io.MouseWheel = g_MouseWheel;
	g_MouseWheel = 0.0f;

	ImGui::NewFrame();
}

void ImGuiHandler::render(osg::RenderInfo& /*theRenderInfo*/)
{

	if (m_callback)
	{
		(*m_callback)();
	}

	ImGui::Render();
}

bool ImGuiHandler::handle(const osgGA::GUIEventAdapter& theEventAdapter,
						  osgGA::GUIActionAdapter& theActionAdapter,
						  osg::Object* theObject,
						  osg::NodeVisitor* theNodeVisitor)
{

	bool wantCapureMouse = ImGui::GetIO().WantCaptureMouse;
	bool wantCapureKeyboard = ImGui::GetIO().WantCaptureKeyboard;

	switch (theEventAdapter.getEventType())
	{

		case osgGA::GUIEventAdapter::KEYDOWN:
		{

			ImGuiIO& io = ImGui::GetIO();
			int c = theEventAdapter.getKey();
			if (c > 0 && c < 0x10000)
			{
				io.AddInputCharacter((unsigned short)c);
			}
			std::cout << c << std::endl << std::flush;
			return wantCapureKeyboard;
		}
		case (osgGA::GUIEventAdapter::PUSH):
		{
			ImGuiIO& io = ImGui::GetIO();
			io.MousePos = ImVec2(theEventAdapter.getX(), io.DisplaySize.y - theEventAdapter.getY());
			g_MousePressed[0] = true;
			return wantCapureMouse;
		}
		case (osgGA::GUIEventAdapter::DRAG):
		case (osgGA::GUIEventAdapter::MOVE):
		{
			ImGuiIO& io = ImGui::GetIO();
			io.MousePos = ImVec2(theEventAdapter.getX(), io.DisplaySize.y - theEventAdapter.getY());
			return wantCapureMouse;
		}
		case (osgGA::GUIEventAdapter::RELEASE):
		{
			g_MousePressed[0] = false;
			return wantCapureMouse;
		}
		case (osgGA::GUIEventAdapter::SCROLL):
		{
			g_MouseWheel = theEventAdapter.getScrollingDeltaY();
			return wantCapureMouse;
		}
		default:
		{
			return false;
		}
	}

	return false;
}


bool ImGui::SliderReal(const char* label, double* v, float v_min, float v_max, const char* display_format, float power)
{
	float aValue = *v;
	if (ImGui::SliderFloat(label, &aValue, v_min, v_max, display_format, power))
	{
		*v = aValue;
		return true;
	}

	return false;
}
