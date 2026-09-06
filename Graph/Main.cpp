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

// Chart
LRESULT CALLBACK ChartWndProc(HWND hChart, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hChart, &ps);

        RECT rect;
        GetClientRect(hChart, &rect);

        g_renderer.drawChart(hdc, rect, g_engine.GetMissionStatistics());

        EndPaint(hChart, &ps);
    }
    break;

    case WM_CLOSE:
        DestroyWindow(hChart);
        return 0; // Tránh báo quit toàn bộ chương trình

    default:
        return DefWindowProc(hChart, message, wParam, lParam);
    }
    return 0;
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_GRAPH, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    AllocConsole();
    FILE* fp;
    freopen_s(&fp, "CONOUT$", "w", stdout);
    freopen_s(&fp, "CONOUT$", "w", stderr);

    if (!InitInstance(hInstance, nCmdShow)) return FALSE;

    std::string dataPath = "D:\\VS_Prj\\Graph\\x64\\Debug\\Data";

    g_engine.SetLogger([](const std::string& logMessage) {
        std::cout << ">>> " << logMessage << std::endl;
        });

    if (g_engine.InitEngineFromDirectory(dataPath))
    {
        g_engine.RunOptimization();
        g_renderer.setGraph(g_engine.GetGraph());
        g_renderer.setUnitList(g_engine.GetGraph().getUnitList());
        g_renderer.setAssignment(g_engine.GetBestSolution());
        g_renderer.setEngine(&g_engine);
        g_renderer.resetView();
        g_engine.PrintAssignmentReport();

        InvalidateRect(hWnd, NULL, TRUE);
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

    // ĐĂNG KÝ THÊM CLASS CHO CỬA SỔ BIỂU ĐỒ
    WNDCLASSEXW wcexChart = wcex;
    wcexChart.lpfnWndProc = ChartWndProc;
    wcexChart.lpszClassName = L"ChartWindow";
    RegisterClassExW(&wcexChart);

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
    case WM_KEYDOWN:
    {
        std::string unitToToggle = "";
        if (wParam == '1') unitToToggle = "a1";
        else if (wParam == '2') unitToToggle = "a2";
        else if (wParam == '3') unitToToggle = "a3";
        else if (wParam == '4') unitToToggle = "a4";

        if (!unitToToggle.empty()) {
            bool currentState = g_engine.IsUnitVisible(unitToToggle);
            g_engine.ToggleUnitVisibility(unitToToggle, !currentState);
            InvalidateRect(hWnd, NULL, TRUE);
        }

        // BẤM PHÍM 'C' HOẶC 'c' ĐỂ MỞ BIỂU ĐỒ
        if (wParam == 'C' || wParam == 'c') {
            HWND hChart = CreateWindowW(L"ChartWindow", L"Dashboard: Bieu Do Thiet Hai Quan Dich",
                WS_OVERLAPPEDWINDOW | WS_VISIBLE, 150, 150, 700, 450,
                nullptr, nullptr, hInst, nullptr);
            UpdateWindow(hChart);
        }
    }
    break;

    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        RECT clientRect;
        GetClientRect(hWnd, &clientRect);
        g_renderer.drawDashboard(hdc, clientRect);
        EndPaint(hWnd, &ps);
    }
    break;

    case WM_SIZE:
        InvalidateRect(hWnd, NULL, TRUE);
        break;

    case WM_MOUSEWHEEL:
    {
        int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        if (delta > 0) g_renderer.zoomIn();
        else g_renderer.zoomOut();
        InvalidateRect(hWnd, NULL, TRUE);
    }
    break;

    case WM_LBUTTONDOWN:
    {
        SetCapture(hWnd);
        int x = LOWORD(lParam);
        int y = HIWORD(lParam);
        RECT clientRect;
        GetClientRect(hWnd, &clientRect);

        if (g_renderer.handleTargetClick(x, y, clientRect)) {
            InvalidateRect(hWnd, NULL, TRUE);   // Vẽ lại để hiện dropdown
            break;
        }

        if (g_renderer.handleUnitClick(x, y, clientRect)) {
            InvalidateRect(hWnd, NULL, TRUE);
            break;
        }
    }
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

