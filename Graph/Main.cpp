#include "framework.h"
#include "GraphRenderer.h"
#include "UAVMissionEngine.h"
#include <iostream>

#define MAX_LOADSTRING 100

// Global Variables:
UAVCore::UAVMissionEngine g_engine;
GraphRenderer g_renderer;
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING]; 
HWND hWnd;

// Forward declarations:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_GRAPH, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    AllocConsole();
    FILE* fp;
    freopen_s(&fp, "CONOUT$", "w", stdout);
    freopen_s(&fp, "CONOUT$", "w", stderr);

    if (!InitInstance(hInstance, nCmdShow)) return FALSE;

    std::string dataPath = "D:\\VS_Prj\\Graph\\x64\\Debug\\Data";
    
    // Đăng ký cách hiển thị Log cho Win32
    g_engine.SetLogger([](const std::string& logMessage) {
        // App console vẫn dùng cout
        std::cout << ">>> " << logMessage << std::endl;
        
        // Hoặc bạn có thể dùng OutputDebugStringA(logMessage.c_str()); để in ra cửa sổ debug của VS.
    });

    if (g_engine.InitEngineFromDirectory(dataPath)) 
    // -------------------------------------------------------------
        {
        // Chạy thuật toán tìm giải pháp
        g_engine.RunOptimization();
        
        // Cấu hình dữ liệu cho bộ phận Render hiển thị lên Win32
        g_renderer.setGraph(g_engine.GetGraph());
        g_renderer.setUnitList(g_engine.GetGraph().getUnitList());
        g_renderer.setAssignment(g_engine.GetBestSolution());
        g_renderer.resetView();

        // Cập nhật lại giao diện
        InvalidateRect(hWnd, NULL, TRUE);

        const AssignmentSolution& best = g_engine.GetBestSolution();
        std::cout << "\n===== ASSIGNMENT RESULT =====\n";
        for (int i = 0; i < best.nUavTypes; i++) {
            for (int j = 0; j < best.nTargets; j++) {
                if (best.at(i, j) == 1) {
                    std::cout << "UAV type " << i
                        << " attacks Target " << j << "\n";
                }
            }
        }
        std::cout << "Fitness = " << best.fitness << "\n";
        std::cout << "=============================\n";
    }
    else
    {
        std::cerr << "\n LỖI đọc dữ liệu từ Engine!" << std::endl;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GRAPH));
    MSG msg;

    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int)msg.wParam;
}

ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_GRAPH));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_GRAPH);
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
    return RegisterClassExW(&wcex);
}

BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
    hInst = hInstance;
    hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
    if (!hWnd) return FALSE;
    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);
    return TRUE;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        RECT clientRect;
        GetClientRect(hWnd, &clientRect);
        g_renderer.draw(hdc, clientRect);
        EndPaint(hWnd, &ps);
    }
    break;

    case WM_SIZE:
        InvalidateRect(hWnd, NULL, TRUE);
        break;

    case WM_MOUSEWHEEL:
    {
        int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        if (delta > 0)
            g_renderer.zoomIn();
        else
            g_renderer.zoomOut();
        InvalidateRect(hWnd, NULL, TRUE);
    }
    break;

    case WM_LBUTTONDOWN:
        SetCapture(hWnd);
        break;

    case WM_LBUTTONUP:
        ReleaseCapture();
        break;

    case WM_MOUSEMOVE:
    {
        static int lastX = -1;
        static int lastY = -1;

        if (wParam & MK_LBUTTON)
        {
            int x = LOWORD(lParam);
            int y = HIWORD(lParam);

            if (lastX != -1 && lastY != -1)
            {
                int dx = x - lastX;
                int dy = y - lastY;
                g_renderer.pan(dx, dy);
                InvalidateRect(hWnd, NULL, TRUE);
            }

            lastX = x;
            lastY = y;
        }
        else
        {
            lastX = lastY = -1;
        }
    }
    break;


    case WM_COMMAND:
    {
        int wmId = LOWORD(wParam);
        switch (wmId)
        {
        case IDM_ABOUT:
            DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
            break;
        case IDM_EXIT:
            DestroyWindow(hWnd);
            break;
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
        }
    }
    break;

    case WM_DESTROY:
        PostQuitMessage(0);
        break;

    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG: return (INT_PTR)TRUE;
    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}