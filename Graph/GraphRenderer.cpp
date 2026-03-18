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
    : graph_(nullptr)
    , unitList_(nullptr)
    , scale_(1.0)
    , offsetX_(0)
    , offsetY_(0)
    , minX_(0.0)
    , maxX_(0.0)
    , minY_(0.0)
    , maxY_(0.0)
    , boundsValid_(false)
{
}
void GraphRenderer::setAssignment(const AssignmentSolution& sol)
{
    assignment_ = sol;
    hasAssignment_ = true;
}


void GraphRenderer::drawAssignment(HDC hdc, RECT clientRect)
{
    if (!hasAssignment_ || !unitList_ || !graph_) return;

    int n = assignment_.nUavTypes;
    int m = assignment_.nTargets;

    const auto& units = unitList_->getUnits();
    const auto& targets = graph_->GetTargets();

    // DEBUG: in ra để xem dữ liệu có hợp lý không
    std::cout << "ASSIGN n=" << n
        << " m=" << m
        << " x.size=" << assignment_.x.size()
        << " unitIndex.size=" << assignment_.unitIndex.size()
        << " paths.size=" << assignment_.paths.size() << "\n";

    HPEN pen = CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
    HPEN oldPen = (HPEN)SelectObject(hdc, pen);

    for (int i = 0; i < n; i++)
    {
        if (i >= (int)assignment_.unitIndex.size()) continue;
        int unitIdx = assignment_.unitIndex[i];
        if (unitIdx < 0 || unitIdx >= (int)units.size()) continue;

        const auto& unit = units[unitIdx];

        double ux = unit.getX();
        double uy = unit.getY();

        for (int j = 0; j < m; j++)
        {
            // bảo vệ x
            if (i * m + j >= (int)assignment_.x.size()) continue;
            int count = assignment_.at(i, j);
            if (count <= 0) continue;

            if (j >= (int)targets.size()) continue;

            double tx = targets[j].x;
            double ty = targets[j].y;

            POINT pU = worldToScreen(ux, uy, clientRect);
            POINT pT = worldToScreen(tx, ty, clientRect);

            if (i >= (int)assignment_.paths.size()) continue;
            if (j >= (int)assignment_.paths[i].size()) continue;

            const std::vector<int>& path = assignment_.paths[i][j];
            if (path.size() < 2) continue;

            for (int v = 0; v < (int)path.size() - 1; v++)
            {
                const auto& v1 = graph_->GetVertexById(path[v]);
                const auto& v2 = graph_->GetVertexById(path[v + 1]);

                POINT p1 = worldToScreen(v1.x, v1.y, clientRect);
                POINT p2 = worldToScreen(v2.x, v2.y, clientRect);

                MoveToEx(hdc, p1.x, p1.y, NULL);
                LineTo(hdc, p2.x, p2.y);
            }
        }
    }

    SelectObject(hdc, oldPen);
    DeleteObject(pen);
}




void GraphRenderer::setGraph(const Graph& graph) noexcept
{
    graph_ = &graph;
    boundsValid_ = false;
}

void GraphRenderer::setUnitList(const UnitUAVList& list) noexcept
{
    unitList_ = &list;
    boundsValid_ = false;
}

void GraphRenderer::ensureBoundsCalculated()
{
    if (boundsValid_) return;
    calculateBounds();
}

void GraphRenderer::calculateBounds()
{
    // initialize with extreme values
    bool any = false;
    double lx = 0.0, ly = 0.0, hx = 0.0, hy = 0.0;

    // lấy bounds từ các vertex trong graph
    if (graph_)
    {
        const auto& verts = graph_->GetVertices();
        for (const auto& v : verts)
        {
            if (v.id == 0)
                continue;

            if (!any) {
                lx = hx = v.x;
                ly = hy = v.y;
                any = true;
            }
            else
            {
                lx = std::min(lx, v.x);
                hx = std::max(hx, v.x);
                ly = std::min(ly, v.y);
                hy = std::max(hy, v.y);
            }
        }
    }

    // lấy thêm bounds từ các đơn vị (Unit)
    if (unitList_)
    {
        const auto& units = unitList_->getUnits();
        for (const auto& u : units)
        {
            if (!any) {
                lx = hx = u.getX();
                ly = hy = u.getY();
                any = true;
            }
            else
            {
                lx = std::min(lx, u.getX());
                hx = std::max(hx, u.getX());
                ly = std::min(ly, u.getY());
                hy = std::max(hy, u.getY());
            }
        }
    }

    // nếu không có dữ liệu nào
    if (!any)
    {
        // default bounds around origin
        minX_ = -1.0; maxX_ = 1.0;
        minY_ = -1.0; maxY_ = 1.0;
    }
    else
    {
        // thêm một chút margin
        double padX = (hx - lx) * 0.05;
        double padY = (hy - ly) * 0.05;
        if (padX == 0.0) padX = 1.0;
        if (padY == 0.0) padY = 1.0;

        minX_ = lx - padX;
        maxX_ = hx + padX;
        minY_ = ly - padY;
        maxY_ = hy + padY;
    }

    boundsValid_ = true;
}


// core draw entry
void GraphRenderer::draw(HDC hdc, RECT clientRect)
{
    ensureBoundsCalculated();

    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;
    if (w <= 0 || h <= 0) return;

    // If user did not set a meaningful scale/offset, compute fit-to-screen once.
    // We detect "unset" by checking offsetX/Y == 0 and scale_ == 1.0 (initial state).
    // This avoids overriding explicit zoom/pan by the caller.
    if (boundsValid_ && scale_ == 1.0 && offsetX_ == 0 && offsetY_ == 0)
    {
        double worldW = maxX_ - minX_;
        double worldH = maxY_ - minY_;
        if (worldW <= 0.0) worldW = 1.0;
        if (worldH <= 0.0) worldH = 1.0;
        double sx = static_cast<double>(w - 20) / (worldW);
        double sy = static_cast<double>(h - 20) / (worldH);
        scale_ = std::max(kMinScale, std::min(kMaxScale, std::min(sx, sy)));
        // keep offsets at 0 (centering done in worldToScreenInternal)
    }

    // draw graph then units
    drawGraph(hdc, clientRect);
    drawUnits(hdc, clientRect);
    drawUAVs(hdc, clientRect);
    drawTargets(hdc, clientRect);
    drawAssignment(hdc, clientRect);


}

POINT GraphRenderer::worldToScreen(double x, double y, RECT clientRect) const noexcept
{
    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;
    return worldToScreenInternal(x, y, w, h);
}

POINT GraphRenderer::worldToScreenInternal(double x, double y, int w, int h) const noexcept
{
    // center of world and screen
    double cx = 0.5 * (minX_ + maxX_);
    double cy = 0.5 * (minY_ + maxY_);
    double sx = scale_;
    if (sx <= 0.0) sx = 1.0;

    int screenCenterX = w / 2;
    int screenCenterY = h / 2;

    int px = static_cast<int>((x - cx) * sx) + screenCenterX + offsetX_;
    int py = screenCenterY - static_cast<int>((y - cy) * sx) + offsetY_; // invert Y for screen

    POINT p = { px, py };
    return p;
}

void GraphRenderer::drawGraph(HDC hdc, RECT clientRect)
{
    if (!graph_) return;

    const auto& edges = graph_->GetEdges();
    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;

    // pen for edges
    HPEN hPenEdge = CreatePen(PS_SOLID, 1, RGB(0, 120, 0));
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPenEdge);

    for (const auto& e : edges)
    {
        POINT p1 = worldToScreenInternal(e.start.x, e.start.y, w, h);
        POINT p2 = worldToScreenInternal(e.end.x, e.end.y, w, h);
        MoveToEx(hdc, p1.x, p1.y, nullptr);
        LineTo(hdc, p2.x, p2.y);
    }

    SelectObject(hdc, hOldPen);
    DeleteObject(hPenEdge);

    // draw vertices
    const auto& verts = graph_->GetVertices();
    HPEN hPenV = CreatePen(PS_SOLID, 1, RGB(0, 0, 180));
    HBRUSH hBrushV = CreateSolidBrush(RGB(100, 149, 237));
    hOldPen = (HPEN)SelectObject(hdc, hPenV);
    HGDIOBJ hOldBrush = SelectObject(hdc, hBrushV);

    const int r = 4;
    for (const auto& v : verts)
    {
        if (v.id == 0)
            continue;
        POINT p = worldToScreenInternal(v.x, v.y, w, h);
        Ellipse(hdc, p.x - r, p.y - r, p.x + r, p.y + r);

        // draw id label
        std::ostringstream ss;
        ss << v.id;
        std::string s = ss.str();
        TextOutA(hdc, p.x + r + 2, p.y - r - 2, s.c_str(), static_cast<int>(s.size()));
    }

    // restore
    SelectObject(hdc, hOldBrush);
    SelectObject(hdc, hOldPen);
    DeleteObject(hBrushV);
    DeleteObject(hPenV);
}

void GraphRenderer::drawUnits(HDC hdc, RECT clientRect)
{
    if (!unitList_) return;
    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;

    const auto& units = unitList_->getUnits();

    // pen/brush for units
    HPEN hPen = CreatePen(PS_SOLID, 1, RGB(180, 30, 30));
    HBRUSH hBrush = CreateSolidBrush(RGB(240, 128, 128));
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);
    HGDIOBJ hOldBrush = SelectObject(hdc, hBrush);

    for (const auto& u : units)
    {
        POINT p = worldToScreenInternal(u.getX(), u.getY(), w, h);

        // draw a triangle-like marker for unit
        const int s = 8;
        POINT pts[3] = { {p.x, p.y - s}, {p.x - s, p.y + s}, {p.x + s, p.y + s} };
        Polygon(hdc, pts, 3);

        // label: unit id and count
        std::ostringstream ss;
        ss << u.getUnitId();
        if (!u.getUnitName().empty()) ss << " (" << u.getUnitName() << ")";
        ss << " [" << u.getUAVCount() << "]";
        std::string label = ss.str();
        TextOutA(hdc, p.x + s + 2, p.y - s - 2, label.c_str(), static_cast<int>(label.size()));
    }

    // restore
    SelectObject(hdc, hOldBrush);
    SelectObject(hdc, hOldPen);
    DeleteObject(hBrush);
    DeleteObject(hPen);
}

void GraphRenderer::drawUAVs(HDC hdc, RECT clientRect)
{
    if (!unitList_) return;

    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;

    // Bút và brush cho UAV
    HPEN hPen = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
    HBRUSH hBrush = CreateSolidBrush(RGB(255, 255, 0)); // UAV màu vàng
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, hBrush);

    const auto& units = unitList_->getUnits();
    const int r = 3; // bán kính UAV

    for (const auto& unit : units)
    {
        POINT p = worldToScreenInternal(unit.getX(), unit.getY(), w, h);

        // Vẽ 1 chấm cho mỗi UAV thuộc đơn vị
        int count = unit.getUAVCount();
        for (int i = 0; i < count; ++i)
        {
            Ellipse(hdc, p.x - r, p.y - r, p.x + r, p.y + r);
        }
    }

    SelectObject(hdc, hOldBrush);
    SelectObject(hdc, hOldPen);
    DeleteObject(hBrush);
    DeleteObject(hPen);
}

void GraphRenderer::drawTargets(HDC hdc, RECT clientRect)
{
    if (!graph_) return;

    const auto& targets = graph_->GetTargets();
    int w = clientRect.right - clientRect.left;
    int h = clientRect.bottom - clientRect.top;

    HPEN hPen = CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
    HBRUSH hBrush = CreateSolidBrush(RGB(255, 200, 200));
    HPEN hOldPen = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hOldBrush = (HBRUSH)SelectObject(hdc, hBrush);

    const int r = 6;

    for (const auto& t : targets)
    {
        POINT p = worldToScreenInternal(t.x, t.y, w, h);

        Ellipse(hdc, p.x - r, p.y - r, p.x + r, p.y + r);

        std::string label = t.code + " (" + std::to_string(t.target_id) + ")";
        TextOutA(hdc, p.x + r + 2, p.y - r - 2, label.c_str(), (int)label.size());
    }

    SelectObject(hdc, hOldBrush);
    SelectObject(hdc, hOldPen);
    DeleteObject(hBrush);
    DeleteObject(hPen);
}


void GraphRenderer::zoomIn()
{
    scale_ = std::min(kMaxScale, scale_ * kZoomFactor);
}

void GraphRenderer::zoomOut()
{
    scale_ = std::max(kMinScale, scale_ / kZoomFactor);
}

void GraphRenderer::pan(int dx, int dy)
{
    offsetX_ += dx;
    offsetY_ += dy;
}

void GraphRenderer::resetView()
{
    // reset to defaults and recalculate bounds on next draw
    scale_ = 1.0;
    offsetX_ = 0;
    offsetY_ = 0;
    boundsValid_ = false;
}
void GraphRenderer::drawPath(HDC hdc, const std::vector<int>& path, RECT clientRect)
{
    if (!graph_ || path.size() < 2) return;

    HPEN hPen = CreatePen(PS_SOLID, 3, RGB(255, 0, 0)); // màu đỏ
    HGDIOBJ oldPen = SelectObject(hdc, hPen);

    for (size_t k = 1; k < path.size(); ++k)
    {
        Vertex a = graph_->GetVertexById(path[k - 1]);
        Vertex b = graph_->GetVertexById(path[k]);

        POINT pa = worldToScreen(a.x, a.y, clientRect);
        POINT pb = worldToScreen(b.x, b.y, clientRect);

        MoveToEx(hdc, pa.x, pa.y, nullptr);
        LineTo(hdc, pb.x, pb.y);
    }

    SelectObject(hdc, oldPen);
    DeleteObject(hPen);
}


