#include "main.h"
#include "../Classes/AppDelegate.h"
#define GLEW_STATIC
#pragma comment(linker, "/NODEFAULTLIB:libc.lib")
USING_NS_CC;

int APIENTRY _tWinMain(HINSTANCE hInstance,
                       HINSTANCE hPrevInstance,
                       LPTSTR    lpCmdLine,
                       int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

	AllocConsole();
	freopen("CONOUT$", "wt", stdout);

    // create the application instance
    AppDelegate app;
    return Application::getInstance()->run();
}
