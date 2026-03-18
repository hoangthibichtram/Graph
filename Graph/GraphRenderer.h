#pragma once

#include <windows.h>
#include "Graph.h"
#include "unitUAVList.h"
#include "UAVOptimization.h"


class GraphRenderer
{
public:
    GraphRenderer() noexcept;

    // Thiết lập dữ liệu nguồn
    void setGraph(const Graph& graph) noexcept;
    void setUnitList(const UnitUAVList& list) noexcept;

    // Vẽ chính
    void draw(HDC hdc, RECT clientRect);

    // Vẽ từng lớp
    void drawGraph(HDC hdc, RECT clientRect);
    void drawUnits(HDC hdc, RECT clientRect);
    void drawUAVs(HDC hdc, RECT clientRect);
    void drawTargets(HDC hdc, RECT clientRect);


    // Điều khiển view
    void zoomIn();
    void zoomOut();
    void pan(int dx, int dy);
    void resetView();

    // Chuyển tọa độ thế giới -> màn hình
    POINT worldToScreen(double x, double y, RECT clientRect) const noexcept;
    

    // Tính lại giới hạn (bounds) dựa trên dữ liệu đồ thị
    void calculateBounds();
    void setAssignment(const AssignmentSolution& sol);
    void drawAssignment(HDC hdc, RECT clientRect);

private:
    const Graph* graph_;              // nguồn đồ thị (không sở hữu)
    const UnitUAVList* unitList_;     // danh sách đơn vị (không sở hữu)
    // Vẽ đường đi UAV → Target
    void drawPath(HDC hdc, const std::vector<int>& path, RECT clientRect);

    double scale_;                    // tỷ lệ zoom (world units -> pixels)
    int offsetX_;                     // dịch ngang (pixel)
    int offsetY_;                     // dịch dọc (pixel)

    double minX_, maxX_, minY_, maxY_; // giới hạn tọa độ thế giới
    bool boundsValid_;

    // Hằng số điều chỉnh zoom / giới hạn
    // Use 'static const' and define values in .cpp to avoid C++14 ODR/link errors
    static const double kZoomFactor;
    static const double kMinScale;
    static const double kMaxScale;
     
    // Helpers nội bộ (cài đặt trong .cpp)
    void ensureBoundsCalculated();
   

    POINT worldToScreenInternal(double x, double y, int w, int h) const noexcept;

    AssignmentSolution assignment_;
    bool hasAssignment_ = false;


};
