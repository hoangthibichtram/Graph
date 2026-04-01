#define NOMINMAX
#include "GraphRenderer.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>

// Provide out-of-class definitions for the static const doubles
const double GraphRenderer::kZoomFactor = 1.25;
const double GraphRenderer::kMinScale = 1e-6;
const double GraphRenderer::kMaxScale = 1e6;

GraphRenderer::GraphRenderer() noexcept
    : graph_(nullptr), unitList_(nullptr), scale_(1.0), offsetX_(0), offsetY_(0),
      minX_(0.0), maxX_(0.0), minY_(0.0), maxY_(0.0), boundsValid_(false)
{
}

void GraphRenderer::setAssignment(const AssignmentSolution& sol) { assignment_ = sol; hasAssignment_ = true; }
void GraphRenderer::setGraph(const Graph& graph) noexcept { graph_ = &graph; boundsValid_ = false; }
void GraphRenderer::setUnitList(const UnitUAVList& list) noexcept { unitList_ = &list; boundsValid_ = false; }
void GraphRenderer::ensureBoundsCalculated() { if (boundsValid_) return; calculateBounds(); }

void GraphRenderer::calculateBounds()
{
    bool any = false;
    double lx = 0.0, ly = 0.0, hx = 0.0, hy = 0.0;

    if (graph_) {
        for (const auto& v : graph_->GetVertices()) {
            if (v.id == 0) continue;
            if (!any) { lx = hx = v.x; ly = hy = v.y; any = true; }
            else { lx = std::min(lx, v.x); hx = std::max(hx, v.x); ly = std::min(ly, v.y); hy = std::max(hy, v.y); }
        }
    }
    if (unitList_) {
        for (const auto& u : unitList_->getUnits()) {
            if (!any) { lx = hx = u.getX(); ly = hy = u.getY(); any = true; }
            else { lx = std::min(lx, u.getX()); hx = std::max(hx, u.getX()); ly = std::min(ly, u.getY()); hy = std::max(hy, u.getY()); }
        }
    }
    if (!any) { minX_ = -1.0; maxX_ = 1.0; minY_ = -1.0; maxY_ = 1.0; }
    else {
        double padX = (hx - lx) * 0.05; double padY = (hy - ly) * 0.05;
        if (padX == 0.0) padX = 1.0; if (padY == 0.0) padY = 1.0;
        minX_ = lx - padX; maxX_ = hx + padX; minY_ = ly - padY; maxY_ = hy + padY;
    }
    boundsValid_ = true;
}

void GraphRenderer::drawAssignment(HDC hdc, RECT clientRect)
{
    if (!hasAssignment_ || !unitList_ || !graph_) return;
    int n = assignment_.nUavTypes; int m = assignment_.nTargets;
    const auto& units = unitList_->getUnits();
    const auto& targets = graph_->GetTargets();

    for (int i = 0; i < n; i++) {
        if (i >= (int)assignment_.unitIndex.size()) continue;
        int unitIdx = assignment_.unitIndex[i];
        if (unitIdx < 0 || unitIdx >= (int)units.size()) continue;

        COLORREF win32Color;
        std::string unitName = "a" + std::to_string(unitIdx + 1);
        if (m_engine && !m_engine->IsUnitVisible(unitName))
            continue;  // Ẩn/hiện đúng như trước đây

        // Lấy code UAV (nếu có)
        std::string uavCode;
        if (unitIdx >= 0 && unitIdx < (int)units.size()) {
            const auto& uavs = units[unitIdx].getUAVs();
            if (i < (int)uavs.size()) {
                uavCode = uavs[i].getCode();
            }
        }

        // Nếu là UAV trinh sát (code bắt đầu "a_A_") thì vẽ màu tím
        if (!uavCode.empty() && uavCode.find("a_A_") == 0) {
            win32Color = RGB(160, 32, 240); // Tím
        }
        else if (unitIdx == 2) {
            win32Color = RGB(255, 140, 0); // Cam cho a3
        }
        else {
            win32Color = RGB(255, 0, 0); // Mặc định
            if (m_engine != nullptr) {
                UAVCore::RGBA coreColor = m_engine->GetUnitLineColor(unitName);
                win32Color = RGB(coreColor.r * 255, coreColor.g * 255, coreColor.b * 255);
            }
        }

        HPEN pen = CreatePen(PS_SOLID, 2, win32Color);
        HPEN oldPen = (HPEN)SelectObject(hdc, pen);

        for (int j = 0; j < m; j++) {
            if (i * m + j >= (int)assignment_.x.size()) continue;
            if (assignment_.at(i, j) <= 0) continue;
            if (j >= (int)targets.size() || i >= (int)assignment_.paths.size() || j >= (int)assignment_.paths[i].size()) continue;

            const std::vector<int>& path = assignment_.paths[i][j];
            if (path.size() < 2) continue;

            for (int v = 0; v < (int)path.size() - 1; v++) {
                const auto& v1 = graph_->GetVertexById(path[v]);
                const auto& v2 = graph_->GetVertexById(path[v + 1]);
                POINT p1 = worldToScreen(v1.x, v1.y, clientRect);
                POINT p2 = worldToScreen(v2.x, v2.y, clientRect);
                MoveToEx(hdc, p1.x, p1.y, NULL);
                LineTo(hdc, p2.x, p2.y);
            }
        }
        SelectObject(hdc, oldPen);
        DeleteObject(pen);
    }
}

void GraphRenderer::draw(HDC hdc, RECT clientRect)
{
    ensureBoundsCalculated();
    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;
    if (w <= 0 || h <= 0) return;

    if (boundsValid_ && scale_ == 1.0 && offsetX_ == 0 && offsetY_ == 0) {
        double worldW = maxX_ - minX_; double worldH = maxY_ - minY_;
        if (worldW <= 0.0) worldW = 1.0; if (worldH <= 0.0) worldH = 1.0;
        double sx = static_cast<double>(w - 20) / worldW;
        double sy = static_cast<double>(h - 20) / worldH;
        scale_ = std::max(kMinScale, std::min(kMaxScale, std::min(sx, sy)));
    }

    drawGraph(hdc, clientRect);
    drawUnits(hdc, clientRect);
    drawUAVs(hdc, clientRect);
    drawTargets(hdc, clientRect);
    drawAssignment(hdc, clientRect);

	// VẼ BÁO CÁO PHÂN TÍCH CHIẾN DỊCH 
    if (m_engine != nullptr) {
        UAVCore::MissionStatistics stats = m_engine->GetMissionStatistics();
        std::wstring dashboardText = L"--- BAO CAO PHAN TICH CHIEN DICH ---\n";

        dashboardText += L"[1] Tong gia tri muc tieu quan dich (Diem Max): " + std::to_wstring((int)stats.totalTargetValue) + L" $\n";
        dashboardText += L"[2] Uoc tinh sat thuong gay ra (Ky vong Pij): " + std::to_wstring((int)stats.expectedDestroyedValue) + L" $\n";
        dashboardText += L"[3] Chi phi trien khai luc luong (Gia UAV): " + std::to_wstring((int)stats.ourLossCost) + L" $\n";

        int netProfit = (int)(stats.expectedDestroyedValue - stats.ourLossCost);
        std::wstring profitStatus = (netProfit > 0) ? L" (LAI)" : L" (LO)";
        dashboardText += L"[4] Loi nhuan/Thiet hai: " + std::to_wstring(netProfit) + L" $" + profitStatus + L"\n";
        dashboardText += L"[5] Muc do tieu diet Can cu Dich: " + std::to_wstring((int)stats.successRate) + L" %\n";
        dashboardText += L"[6] So diem muc tieu da danh: " + std::to_wstring(stats.totalTargetsHit) + L"\n";
        dashboardText += L"[7] Tong UAV tham chien: " + std::to_wstring(stats.totalUAVDeployed) + L" UAV.\n";

        SetBkMode(hdc, TRANSPARENT);
        SetTextColor(hdc, RGB(255, 255, 255));

        RECT bgRect = { 10, 10, 420, 170 };
        HBRUSH bgBrush = CreateSolidBrush(RGB(30, 30, 30));
        FillRect(hdc, &bgRect, bgBrush);
        DeleteObject(bgBrush);

        RECT textRect = { 15, 15, 415, 165 };
        DrawTextW(hdc, dashboardText.c_str(), -1, &textRect, DT_LEFT | DT_TOP);
    }

    // VẼ BẢNG ĐIỀU KHIỂN (UNIT CHECKLIST) GÓC PHẢI MÀN HÌNH
    if (m_engine != nullptr && unitList_ != nullptr) {
        int panelX = w - 180; 
        int panelY = 10;
        int lineHeight = 30;  
        int unitCount = unitList_->getUnitCount();

        RECT uiRect = { panelX, panelY, w - 10, panelY + 15 + unitCount * lineHeight };
        HBRUSH uiBrush = CreateSolidBrush(RGB(40, 40, 50));
        FillRect(hdc, &uiRect, uiBrush);
        DeleteObject(uiBrush);

        SetBkMode(hdc, TRANSPARENT);

        for (int i = 0; i < unitCount; ++i) {
            std::string unitName = "a" + std::to_string(i + 1);
            
            bool isVisible = m_engine->IsUnitVisible(unitName);
            COLORREF c;

            if (unitName == "a3") {
                c = RGB(255, 130, 0); // Cam
            }
            else {
                UAVCore::RGBA coreColor = m_engine->GetUnitLineColor(unitName);
                c = RGB(coreColor.r * 255, coreColor.g * 255, coreColor.b * 255);
            }

            /*UAVCore::RGBA coreColor = m_engine->GetUnitLineColor(unitName);
            COLORREF c = RGB(coreColor.r * 255, coreColor.g * 255, coreColor.b * 255);*/

            int yOffset = panelY + 10 + i * lineHeight;

            RECT boxRect = { panelX + 15, yOffset, panelX + 30, yOffset + 15 };
            HBRUSH boxBrush = isVisible ? CreateSolidBrush(c) : CreateSolidBrush(RGB(100, 100, 100)); 
            FillRect(hdc, &boxRect, boxBrush);
            DeleteObject(boxBrush);

            std::wstring label = std::wstring(unitName.begin(), unitName.end());
            label += isVisible ? L" (Hien)" : L" (An)";
            SetTextColor(hdc, isVisible ? RGB(255, 255, 255) : RGB(150, 150, 150));
            
            RECT textRect = { panelX + 40, yOffset - 2, w - 10, yOffset + lineHeight };
            DrawTextW(hdc, label.c_str(), -1, &textRect, DT_LEFT | DT_TOP);
        }
    }
}

POINT GraphRenderer::worldToScreen(double x, double y, RECT clientRect) const noexcept {
    int w = clientRect.right - clientRect.left; int h = clientRect.bottom - clientRect.top;
    return worldToScreenInternal(x, y, w, h);
}

POINT GraphRenderer::worldToScreenInternal(double x, double y, int w, int h) const noexcept {
    double cx = 0.5 * (minX_ + maxX_); double cy = 0.5 * (minY_ + maxY_);
    double sx = scale_; if (sx <= 0.0) sx = 1.0;
    int px = static_cast<int>((x - cx) * sx) + w / 2 + offsetX_;
    int py = h / 2 - static_cast<int>((y - cy) * sx) + offsetY_;
    return { px, py };
}

void GraphRenderer::drawGraph(HDC hdc, RECT clientRect) {
    if (!graph_) return;
    int w = clientRect.right - clientRect.left; int h = clientRect.bottom - clientRect.top;
    
    HPEN hPenEdge = CreatePen(PS_SOLID, 1, RGB(0, 120, 0));
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPenEdge);
    for (const auto& e : graph_->GetEdges()) {
        POINT p1 = worldToScreenInternal(e.start.x, e.start.y, w, h);
        POINT p2 = worldToScreenInternal(e.end.x, e.end.y, w, h);
        MoveToEx(hdc, p1.x, p1.y, nullptr); LineTo(hdc, p2.x, p2.y);
    }
    SelectObject(hdc, hOldPen); DeleteObject(hPenEdge);

    HPEN hPenV = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
    HBRUSH hBrushV = CreateSolidBrush(RGB(0, 0, 0));
    hOldPen = (HPEN)SelectObject(hdc, hPenV);
    HGDIOBJ hOldBrush = SelectObject(hdc, hBrushV);
    for (const auto& v : graph_->GetVertices()) {
       
        if (v.id == 0 || v.id >= 10000) continue; 
        
        POINT p = worldToScreenInternal(v.x, v.y, w, h);
        Ellipse(hdc, p.x - 4, p.y - 4, p.x + 4, p.y + 4);
        
        std::string s = std::to_string(v.id);
        TextOutA(hdc, p.x + 6, p.y - 6, s.c_str(), (int)s.size());
    }
    
    SelectObject(hdc, hOldBrush); SelectObject(hdc, hOldPen);
    DeleteObject(hBrushV); DeleteObject(hPenV);
}

void GraphRenderer::drawUnits(HDC hdc, RECT clientRect) {
    if (!unitList_) return;
    int w = clientRect.right - clientRect.left; int h = clientRect.bottom - clientRect.top;
    
    HPEN hPen = CreatePen(PS_SOLID, 1, RGB(180, 30, 30));
    HBRUSH hBrush = CreateSolidBrush(RGB(240, 128, 128));
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);
    HGDIOBJ hOldBrush = SelectObject(hdc, hBrush);
    
    const auto& units = unitList_->getUnits();
    
    for (size_t i = 0; i < units.size(); ++i) {
        const auto& u = units[i];
        POINT p = worldToScreenInternal(u.getX(), u.getY(), w, h);
        
        POINT pts[3] = { {p.x, p.y - 8}, {p.x - 8, p.y + 8}, {p.x + 8, p.y + 8} };
        Polygon(hdc, pts, 3);
       
        std::string label = u.getUnitId();
            
        SetBkMode(hdc, TRANSPARENT);
        SetTextColor(hdc, RGB(30,30,30));
        TextOutA(hdc, p.x + 10, p.y - 10, label.c_str(), (int)label.size());

        // NẾU ĐƠN VỊ ĐANG ĐƯỢC CLICK (XỔ MENU DROPDOWN)
        if (static_cast<int>(i) == selectedUnitIndex_) {
            int uavCount = u.getUAVCount();
            int lineH = 20;
            int boxWidth = 130; // Kéo rộng thêm 1 chút cho chữ đẹp
            int boxHeight = uavCount * lineH + 5;
            
            RECT dropRect = { p.x + 15, p.y + 10, p.x + 15 + boxWidth, p.y + 10 + boxHeight };
            HBRUSH dropBrush = CreateSolidBrush(RGB(50, 50, 60));
            FillRect(hdc, &dropRect, dropBrush);
            
            HPEN borderPen = CreatePen(PS_SOLID, 1, RGB(200, 200, 200));
            SelectObject(hdc, borderPen);
            MoveToEx(hdc, dropRect.left, dropRect.top, NULL); LineTo(hdc, dropRect.right, dropRect.top);
            LineTo(hdc, dropRect.right, dropRect.bottom); LineTo(hdc, dropRect.left, dropRect.bottom);
            LineTo(hdc, dropRect.left, dropRect.top);
            DeleteObject(borderPen);
            DeleteObject(dropBrush);
            
            SetTextColor(hdc, RGB(240, 240, 200));
            
            // 2. CẬP NHẬT DANH SÁCH UAV BẰNG MÃ CODE (A1, B1...)
            const auto& myUAVs = u.getUAVs();
            for (int k = 0; k < uavCount; ++k) {
                // Rút trích Code thật từ list UAV
                std::string uavItem = "  -> UAV_" + myUAVs[k].getCode();
                TextOutA(hdc, p.x + 20, p.y + 12 + k * lineH, uavItem.c_str(), (int)uavItem.size());
            }
        }
    }
    
    SelectObject(hdc, hOldBrush); SelectObject(hdc, hOldPen); 
    DeleteObject(hBrush); DeleteObject(hPen);
}

void GraphRenderer::drawUAVs(HDC hdc, RECT clientRect) {
    if (!unitList_) return;
    int w = clientRect.right - clientRect.left; int h = clientRect.bottom - clientRect.top;
    HPEN hPen = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
    HBRUSH hBrush = CreateSolidBrush(RGB(255, 255, 0)); 
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, hBrush);
    for (const auto& unit : unitList_->getUnits()) {
        POINT p = worldToScreenInternal(unit.getX(), unit.getY(), w, h);
        for (int i = 0; i < unit.getUAVCount(); ++i) Ellipse(hdc, p.x - 3, p.y - 3, p.x + 3, p.y + 3);
    }
    SelectObject(hdc, hOldBrush); SelectObject(hdc, hOldPen); DeleteObject(hBrush); DeleteObject(hPen);
}

void GraphRenderer::drawTargets(HDC hdc, RECT clientRect) {
    if (!graph_) return;
    int w = clientRect.right - clientRect.left; int h = clientRect.bottom - clientRect.top;
    HPEN hPen = CreatePen(PS_SOLID, 2, RGB(0, 0, 255));
    HBRUSH hBrush = CreateSolidBrush(RGB(100, 149, 237));
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, hBrush);
    for (const auto& t : graph_->GetTargets()) {
        POINT p = worldToScreenInternal(t.x, t.y, w, h);
        Ellipse(hdc, p.x - 6, p.y - 6, p.x + 6, p.y + 6);
        SetTextColor(hdc, RGB(0, 0, 0));
        // Code mới để hiển thị Tên mục tiêu (Cột name trong file CSV)
        std::string label = t.name;
        TextOutA(hdc, p.x + 8, p.y - 8, label.c_str(), (int)label.size());
    }
    // Hiển thị dropdown UAV tấn công mục tiêu khi mục tiêu được chọn
    if (m_engine != nullptr && selectedTargetIndex_ >= 0) {
        auto uavList = m_engine->GetUAVsAttackingTarget(selectedTargetIndex_);
        if (!uavList.empty()) {
            int lineH = 20;
            int boxWidth = 130;
            int boxHeight = (int)uavList.size() * lineH + 5;
            POINT p = selectedTargetScreenPos_;
            RECT dropRect = { p.x + 15, p.y + 10, p.x + 15 + boxWidth, p.y + 10 + boxHeight };
            HBRUSH dropBrush = CreateSolidBrush(RGB(50, 60, 80));
            FillRect(hdc, &dropRect, dropBrush);
            HPEN borderPen = CreatePen(PS_SOLID, 1, RGB(200, 200, 200));
            SelectObject(hdc, borderPen);
            MoveToEx(hdc, dropRect.left, dropRect.top, NULL); LineTo(hdc, dropRect.right, dropRect.top);
            LineTo(hdc, dropRect.right, dropRect.bottom); LineTo(hdc, dropRect.left, dropRect.bottom);
            LineTo(hdc, dropRect.left, dropRect.top);
            DeleteObject(borderPen);
            DeleteObject(dropBrush);
            SetTextColor(hdc, RGB(240, 240, 200));
            for (int k = 0; k < (int)uavList.size(); ++k) {
                std::string uavItem = "  -> UAV_" + uavList[k];
                TextOutA(hdc, p.x + 20, p.y + 12 + k * lineH, uavItem.c_str(), (int)uavItem.size());
            }
        }
    }
    SelectObject(hdc, hOldBrush); SelectObject(hdc, hOldPen); DeleteObject(hBrush); DeleteObject(hPen);
}

void GraphRenderer::zoomIn() { scale_ = std::min(kMaxScale, scale_ * kZoomFactor); }
void GraphRenderer::zoomOut() { scale_ = std::max(kMinScale, scale_ / kZoomFactor); }
void GraphRenderer::pan(int dx, int dy) { offsetX_ += dx; offsetY_ += dy; }
void GraphRenderer::resetView() { scale_ = 1.0; offsetX_ = 0; offsetY_ = 0; boundsValid_ = false; }

void GraphRenderer::drawPath(HDC hdc, const std::vector<int>& path, RECT clientRect) {
    if (!graph_ || path.size() < 2) return;
    HPEN hPen = CreatePen(PS_SOLID, 3, RGB(25, 0, 0)); 
    HGDIOBJ oldPen = SelectObject(hdc, hPen);
    for (size_t k = 1; k < path.size(); ++k) {
        POINT pa = worldToScreen(graph_->GetVertexById(path[k - 1]).x, graph_->GetVertexById(path[k - 1]).y, clientRect);
        POINT pb = worldToScreen(graph_->GetVertexById(path[k]).x, graph_->GetVertexById(path[k]).y, clientRect);
        MoveToEx(hdc, pa.x, pa.y, nullptr); LineTo(hdc, pb.x, pb.y);
    }
    SelectObject(hdc, oldPen); DeleteObject(hPen);
}

bool GraphRenderer::handleUnitClick(int mouseX, int mouseY, RECT clientRect)
{
    if (!unitList_) return false;
    
    // Quét qua các căn cứ để xem click chuột có trúng tọa độ của nó trên màn hình không
    const auto& units = unitList_->getUnits();
    for (size_t i = 0; i < units.size(); ++i) {
        POINT p = worldToScreen(units[i].getX(), units[i].getY(), clientRect);
        
        // Tính khoảng cách từ chuột tới căn cứ (vùng Hitbox bán kính tầm 15 pixel)
        int dx = mouseX - p.x;
        int dy = mouseY - p.y;
        if (dx * dx + dy * dy <= 15 * 15) {
            // Nếu click trúng trạm đang mở -> Tắt nó đi. Nếu click trạm khác -> Mở trạm đó lên.
            if (selectedUnitIndex_ == static_cast<int>(i)) {
                selectedUnitIndex_ = -1; 
            } else {
                selectedUnitIndex_ = static_cast<int>(i);
            }
            return true; // Trả về true báo hiệu đã trúng
        }
    }
    
    selectedUnitIndex_ = -1; // Click ra ngoài không gian thì đóng danh sách
    return false;
}
bool GraphRenderer::handleTargetClick(int mouseX, int mouseY, RECT clientRect)
{
    if (!graph_) return false;
    const auto& targets = graph_->GetTargets();
    for (size_t i = 0; i < targets.size(); ++i) {
        POINT p = worldToScreen(targets[i].x, targets[i].y, clientRect);
        int dx = mouseX - p.x;
        int dy = mouseY - p.y;
        if (dx * dx + dy * dy <= 15 * 15) {
            if (selectedTargetIndex_ == static_cast<int>(i)) {
                selectedTargetIndex_ = -1;
            }
            else {
                selectedTargetIndex_ = static_cast<int>(i);
                selectedTargetScreenPos_ = p;
            }
            return true;
        }
    }
    selectedTargetIndex_ = -1;
    return false;
}
//std::vector<std::string> GraphRenderer::getUAVsForTarget(int targetIdx) const
//{
//    std::vector<std::string> result;
//    if (!hasAssignment_ || !unitList_ || !graph_) return result;
//    int n = assignment_.nUavTypes;
//    int m = assignment_.nTargets;
//    if (targetIdx < 0 || targetIdx >= m) return result;
//
//    const auto& units = unitList_->getUnits();
//
//    for (int i = 0; i < n; ++i) {
//        if (assignment_.at(i, targetIdx) > 0) {
//            std::string uavName = "UAV_" + std::to_string(i + 1);
//
//            // Lấy đúng đơn vị chứa UAV loại i
//            if (assignment_.unitIndex.size() > i) {
//                int unitIdx = assignment_.unitIndex[i];
//                if (unitIdx >= 0 && unitIdx < (int)units.size()) {
//                    const auto& uavs = units[unitIdx].getUAVs();
//                    // Tìm UAV có code trùng với loại i (nếu có)
//                    // Nếu mỗi UAV có id duy nhất, bạn có thể duyệt qua uavs để tìm đúng code
//                    // Ở đây, nếu mỗi unit chỉ có 1 UAV, lấy luôn
//                    if (!uavs.empty()) {
//                        // Nếu số lượng UAV của đơn vị == nUavTypes, lấy theo i
//                        if ((int)uavs.size() == n) {
//                            uavName = uavs[i].getCode();
//                        }
//                        else {
//                            // Nếu không, lấy UAV đầu tiên của đơn vị
//                            uavName = uavs[0].getCode();
//                        }
//                    }
//                    else {
//                        uavName = "UAV_" + std::to_string(i + 1);
//                    }
//                    // Gắn thêm tên đơn vị nếu muốn
//                    uavName = units[unitIdx].getUnitId() + " - " + uavName;
//                }
//            }
//            result.push_back(uavName);
//        }
//    }
//    return result;
//}





