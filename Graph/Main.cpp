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

// ==================== CỬA SỔ BIỂU ĐỒ BAR CHART ====================
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
            HBRUSH bgBrush = CreateSolidBrush(RGB(40, 40, 40));
            FillRect(hdc, &rect, bgBrush);
            DeleteObject(bgBrush);

            SetBkMode(hdc, TRANSPARENT);
            SetTextColor(hdc, RGB(255, 255, 255));
            TextOutA(hdc, 20, 20, "--- BIEU DO 100%: MUC DIET (DO) VA SONG SOT (XANH) THEO TUNG TRAM ---", 70);

            // Vẽ trục X và Y
            HPEN axisPen = CreatePen(PS_SOLID, 2, RGB(200, 200, 200));
            SelectObject(hdc, axisPen);
            
            // Lùi trục X lên cao chút dể lấy nhiều đất cho chữ zíc zắc
            int startX = 60, startY = rect.bottom - 80; 
            int maxH = startY - 60; // Chiều cao tuyệt đối của một cột 100%
            if (maxH < 50) maxH = 50;

            MoveToEx(hdc, startX, startY, nullptr); LineTo(hdc, rect.right - 20, startY); // Trục X
            MoveToEx(hdc, startX, startY, nullptr); LineTo(hdc, startX, 40); // Trục Y
            DeleteObject(axisPen);

            UAVCore::MissionStatistics stats = g_engine.GetMissionStatistics();
            int barWidth = 40;
            int gap = 60; // Nới rộng khoảng cách giữa các cột ra!

            HBRUSH redBrush = CreateSolidBrush(RGB(220, 40, 40));   // ĐỎ là Tiêu diệt
            HBRUSH greenBrush = CreateSolidBrush(RGB(40, 200, 40)); // XANH là Sống sót

            for (size_t i = 0; i < stats.targetDamagePercents.size(); ++i) {
                int rectLeft = startX + 30 + i * (barWidth + gap);
                int rectRight = rectLeft + barWidth;
                
                // Mức sát thương (Màu đỏ, Vẽ từ dưới đáy đi lên)
                float dmgPercent = stats.targetDamagePercents[i];
                if (dmgPercent < 0) dmgPercent = 0;
                if (dmgPercent > 100) dmgPercent = 100;
                float survPercent = 100.0f - dmgPercent;

                int redH = (int)((dmgPercent / 100.0f) * maxH);
                int greenH = maxH - redH; // Phần còn lại là màu xanh

                int redTop = startY - redH;
                int greenTop = redTop - greenH;

                // 1. Phết màu Đỏ cho Khúc dưới (Sát thương)
                if (redH > 0) {
                    RECT barDmg = { rectLeft, redTop, rectRight, startY - 1 };
                    FillRect(hdc, &barDmg, redBrush);
                }

                // 2. Phết màu Xanh cho Khúc trên (Sống sót) chồng ngay trên Đỏ
                if (greenH > 0) {
                    RECT barSurv = { rectLeft, greenTop, rectRight, redTop };
                    FillRect(hdc, &barSurv, greenBrush);
                }

                // 3. Viết Nhãn % nằm TRONG CỘT để dễ nhìn
                int centerTextX = rectLeft + 5;
                if (redH > 15) {
                    std::string rTxt = std::to_string((int)(dmgPercent + 0.5f)) + "%";
                    SetTextColor(hdc, RGB(255, 255, 255));
                    TextOutA(hdc, centerTextX, startY - (redH/2) - 8, rTxt.c_str(), (int)rTxt.size());
                }
                if (greenH > 15) {
                    std::string gTxt = std::to_string((int)(survPercent + 0.5f)) + "%";
                    SetTextColor(hdc, RGB(30, 30, 30)); // Chữ Tối trên nền Xanh sáng
                    TextOutA(hdc, centerTextX, redTop - (greenH/2) - 8, gTxt.c_str(), (int)gTxt.size());
                }

                // Tên mục tiêu ở chân cột, VẼ ZÍC-ZẮC thò thụt để ko bị đè chữ
                SetTextColor(hdc, RGB(255, 255, 255));
                std::string tName = stats.targetNames[i];
                
                // Nếu cột lẻ (1, 3, 5..) thì chữ tụt xuống 20 pixel, cột chẵn úp sát vạch
                int textY = startY + 5;
                if (i % 2 != 0) textY += 20; 

                TextOutA(hdc, rectLeft - 10, textY, tName.c_str(), (int)tName.size());
            }

            DeleteObject(redBrush);
            DeleteObject(greenBrush);

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
// ===================================================================

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
        if (wParam == '1') unitToToggle = "SQ1";
        else if (wParam == '2') unitToToggle = "SQ2";
        else if (wParam == '3') unitToToggle = "SQ3";
        else if (wParam == '4') unitToToggle = "SQ4";

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

