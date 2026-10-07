@echo off
chcp 65001 >nul
setlocal

set SRC=D:\VS_Prj\Graph\x64\Debug\Data
set DST=D:\UE_Project\UAVMission\Content\Data

echo ============================================================
echo   DONG BO KET QUA 2D  -^>  DU AN 3D
echo ============================================================
echo   Nguon : %SRC%
echo   Dich  : %DST%
echo.

if not exist "%SRC%\MissionPlan.csv" (
    echo [LOI] Khong tim thay MissionPlan.csv trong thu muc nguon.
    echo       Hay chay Graph.exe truoc, va kiem tra console co dong [XUAT].
    goto :ket
)
if not exist "%DST%\" (
    echo [LOI] Khong tim thay thu muc dich. Kiem tra lai duong dan du an 3D.
    goto :ket
)

echo --- Ke hoach tien cong ---
copy /Y "%SRC%\MissionPlan.csv"    "%DST%\MissionPlan.csv"     >nul && echo   OK  MissionPlan.csv    || echo   LOI MissionPlan.csv    ^(dong Unreal Editor roi thu lai^)
copy /Y "%SRC%\MissionSummary.csv" "%DST%\MissionSummary.csv"  >nul && echo   OK  MissionSummary.csv || echo   LOI MissionSummary.csv ^(dong Unreal Editor roi thu lai^)

echo.
echo --- Du lieu goc ^(doi ten them so 1^) ---
copy /Y "%SRC%\Vertex.csv"      "%DST%\Vertex1.csv"      >nul && echo   OK  Vertex1.csv      || echo   LOI Vertex1.csv
copy /Y "%SRC%\Edge.csv"        "%DST%\Edge1.csv"        >nul && echo   OK  Edge1.csv        || echo   LOI Edge1.csv
copy /Y "%SRC%\UnitUAV.csv"     "%DST%\UnitUAV1.csv"     >nul && echo   OK  UnitUAV1.csv     || echo   LOI UnitUAV1.csv
copy /Y "%SRC%\Data_target.csv" "%DST%\Data_target1.csv" >nul && echo   OK  Data_target1.csv || echo   LOI Data_target1.csv
copy /Y "%SRC%\Data_uav.csv"    "%DST%\Data_uav1.csv"    >nul && echo   OK  Data_uav1.csv    || echo   LOI Data_uav1.csv
copy /Y "%SRC%\Probability.csv" "%DST%\Probability1.csv" >nul && echo   OK  Probability1.csv || echo   LOI Probability1.csv

echo.
echo ============================================================
echo   NOI DUNG KE HOACH VUA DONG BO
echo ============================================================
type "%DST%\MissionSummary.csv"
echo ============================================================
echo.
echo Buoc tiep theo: dong Unreal Editor, rebuild neu co sua ma nguon,
echo roi mo lai va nhan Play. Doi chieu dong "Ke hoach: F = ..." trong Output Log.

:ket
echo.
pause
