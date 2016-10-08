#ifndef UTILS_IMGUI_HANDLER_HEADER
#define UTILS_IMGUI_HANDLER_HEADER

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#include <osgViewer/ViewerEventHandlers>
#include <osg/Camera>

namespace sf {

  struct GuiCallback
  {
      virtual void operator () () {}
  };

  class ImGuiHandler : public osgGA::GUIEventHandler {

  public:

    ImGuiHandler (GuiCallback* theGuicallback);

    void init();

    void newFrame (osg::RenderInfo& theRenderInfo);

    void render (osg::RenderInfo& theRenderInfo);

    void setCameraCallbacks (osg::Camera* theCamera);

    virtual bool handle(const osgGA::GUIEventAdapter& theEventAdapter,
                        osgGA::GUIActionAdapter& theActionAdapter,
                        osg::Object* theObject,
                        osg::NodeVisitor* theNodeVisitor);

  private:

    GuiCallback* m_callback;
    double g_Time;
    bool g_MousePressed[3];
    float g_MouseWheel;
    GLuint g_FontTexture;

  };

} // namespace sf

namespace ImGui {

  bool SliderReal (const char* label, double* v, float v_min, float v_max, const char* display_format = "%.3f", float power = 1.0f);

}

#endif // UTILS_IMGUI_HANDLER_HEADER
