# Corner Detect

参考论文：《Automatic Camera and Range Sensor Calibration using a single Shot》。

基于参考论文设计了下图所示的 marker，并实现了角点识别任务。

![marker 图标](/Marker.png)

|步骤(*)|函数|功能|
|---|---|---|
|Step1|secondDeriveCornerMetric()|论文中用模板卷积提取角点，此处用二阶图像梯度来提取角点|
|Step2|nonMaximumSuppression()|角点邻域中非极大值抑制|
|Step3|detectCornersOnMarker()|找到真正的棋盘格角点，剔除噪声角点|
|Substep3.1|findFirstSecondCorners()|读取一个角点，根据这个角点找到一个相邻角点，同时获取搜索方向|
|Substep3.1.1*|subPixelLocation()|亚像素精度角点|
|Substep3.1.2|findEdgeAngles()|用高斯曲线生成统计直方图，找到棋盘方格的两个主方向|
|Substep3.1.3|calcBolicCorrelation()|用双曲正切函数生成棋盘格模板，并计算相关度，用于剔除噪声角点|
|Substep3.1.4|findNextCorner()|根据当前 corner 的 width 参数搜索水平方向，找到一个符合要求的相邻角点|
|Substep3.2|predictNextCorner()|在 1、2 角点的线方向上循环搜索角点|

> subPixelLocation() 的准确性有待验证。