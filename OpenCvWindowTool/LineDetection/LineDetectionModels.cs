using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;

namespace OpenCvWindowTool
{
    /// <summary>
    /// 表示直线检测时允许通过的边缘极性。
    /// </summary>
    public enum LineEdgePolarity
    {
        Positive,
        Negative,
        Any
    }

    /// <summary>
    /// 表示每条卡尺线上候选边缘点的选择方式。
    /// </summary>
    public enum LineSelectionMode
    {
        First,
        Strongest,
        Last
    }

    /// <summary>
    /// 表示直线检测在ROI内部的扫描方向。
    /// </summary>
    public enum LineScanDirection
    {
        LeftToRight,
        TopToBottom,
        BottomToTop,
        RightToLeft
    }

    /// <summary>
    /// 表示检测点拟合为直线时使用的方式。
    /// </summary>
    public enum LineFitMode
    {
        Robust,
        LeastSquares
    }

    /// <summary>
    /// 保存直线检测可调参数。
    /// </summary>
    public sealed class LineDetectionParams
    {
        /// <summary>
        /// 初始化直线检测参数默认值。
        /// </summary>
        public LineDetectionParams()
        {
            EdgeThreshold = 20f;
            SampleCount = 40;
            SampleStep = 1f;
            SmoothSize = 3;
            EdgePolarity = LineEdgePolarity.Any;
            SelectionMode = LineSelectionMode.Strongest;
            ScanDirection = LineScanDirection.LeftToRight;
            FitMode = LineFitMode.Robust;
        }

        /// <summary>
        /// 获取或设置边缘强度阈值。
        /// </summary>
        public float EdgeThreshold { get; set; }

        /// <summary>
        /// 获取或设置检测点数量，也就是卡尺线数量。
        /// </summary>
        public int SampleCount { get; set; }

        /// <summary>
        /// 获取或设置沿扫描方向采样的像素间隔。
        /// </summary>
        public float SampleStep { get; set; }

        /// <summary>
        /// 获取或设置一维灰度曲线平滑窗口，偶数会自动修正为奇数。
        /// </summary>
        public int SmoothSize { get; set; }

        /// <summary>
        /// 获取或设置边缘极性。
        /// </summary>
        public LineEdgePolarity EdgePolarity { get; set; }

        /// <summary>
        /// 获取或设置候选边缘选择方式。
        /// </summary>
        public LineSelectionMode SelectionMode { get; set; }

        /// <summary>
        /// 获取或设置ROI内部扫描方向。
        /// </summary>
        public LineScanDirection ScanDirection { get; set; }

        /// <summary>
        /// 获取或设置直线拟合方式。
        /// </summary>
        public LineFitMode FitMode { get; set; }
    }

    /// <summary>
    /// 表示直线检测使用的矩形测量框。
    /// </summary>
    public struct LineDetectionFrame
    {
        public readonly PointF Center;
        public readonly float Width;
        public readonly float Height;
        public readonly float Angle;

        /// <summary>
        /// 初始化直线检测测量框。
        /// </summary>
        /// <param name="center">测量框中心点。</param>
        /// <param name="width">测量框宽度。</param>
        /// <param name="height">测量框高度。</param>
        /// <param name="angle">测量框旋转角度。</param>
        public LineDetectionFrame(PointF center, float width, float height, float angle)
        {
            Center = center;
            Width = Math.Max(1f, width);
            Height = Math.Max(1f, height);
            Angle = angle;
        }

        /// <summary>
        /// 获取ROI局部X方向单位向量。
        /// </summary>
        public PointF XDirection => UnitVector(Angle);

        /// <summary>
        /// 获取ROI局部Y方向单位向量。
        /// </summary>
        public PointF YDirection
        {
            get
            {
                PointF x = XDirection;
                return new PointF(-x.Y, x.X);
            }
        }

        /// <summary>
        /// 获取测量框是否有效。
        /// </summary>
        public bool IsValid => Width > 0f && Height > 0f;

        /// <summary>
        /// 根据扫描方向获取扫描向量。
        /// </summary>
        /// <param name="direction">扫描方向。</param>
        /// <returns>扫描方向单位向量。</returns>
        public PointF GetScanDirection(LineScanDirection direction)
        {
            PointF x = XDirection;
            PointF y = YDirection;
            switch (direction)
            {
                case LineScanDirection.RightToLeft:
                    return new PointF(-x.X, -x.Y);
                case LineScanDirection.TopToBottom:
                    return y;
                case LineScanDirection.BottomToTop:
                    return new PointF(-y.X, -y.Y);
                default:
                    return x;
            }
        }

        /// <summary>
        /// 根据扫描方向获取卡尺排列方向。
        /// </summary>
        /// <param name="direction">扫描方向。</param>
        /// <returns>垂直于扫描方向的单位向量。</returns>
        public PointF GetLineArrangeDirection(LineScanDirection direction)
        {
            PointF scan = GetScanDirection(direction);
            return new PointF(-scan.Y, scan.X);
        }

        /// <summary>
        /// 根据扫描方向获取扫描长度。
        /// </summary>
        /// <param name="direction">扫描方向。</param>
        /// <returns>扫描方向上的ROI长度。</returns>
        public float GetScanLength(LineScanDirection direction)
        {
            return direction == LineScanDirection.LeftToRight || direction == LineScanDirection.RightToLeft ? Width : Height;
        }

        /// <summary>
        /// 根据扫描方向获取卡尺排列长度。
        /// </summary>
        /// <param name="direction">扫描方向。</param>
        /// <returns>卡尺排列方向上的ROI长度。</returns>
        public float GetArrangeLength(LineScanDirection direction)
        {
            return direction == LineScanDirection.LeftToRight || direction == LineScanDirection.RightToLeft ? Height : Width;
        }

        /// <summary>
        /// 获取测量框四个角点。
        /// </summary>
        /// <returns>四个角点坐标。</returns>
        public PointF[] GetCorners()
        {
            PointF x = XDirection;
            PointF y = YDirection;
            float halfWidth = Width / 2f;
            float halfHeight = Height / 2f;
            return new[]
            {
                Offset(Center, x, y, -halfWidth, -halfHeight),
                Offset(Center, x, y, halfWidth, -halfHeight),
                Offset(Center, x, y, halfWidth, halfHeight),
                Offset(Center, x, y, -halfWidth, halfHeight)
            };
        }

        /// <summary>
        /// 根据角度生成单位向量。
        /// </summary>
        /// <param name="angle">角度值。</param>
        /// <returns>单位向量。</returns>
        private static PointF UnitVector(float angle)
        {
            double radians = angle * Math.PI / 180d;
            return new PointF((float)Math.Cos(radians), (float)Math.Sin(radians));
        }

        /// <summary>
        /// 按局部坐标偏移中心点。
        /// </summary>
        /// <param name="center">中心点。</param>
        /// <param name="x">局部X方向。</param>
        /// <param name="y">局部Y方向。</param>
        /// <param name="xDistance">X方向距离。</param>
        /// <param name="yDistance">Y方向距离。</param>
        /// <returns>偏移后的点。</returns>
        private static PointF Offset(PointF center, PointF x, PointF y, float xDistance, float yDistance)
        {
            return new PointF(center.X + x.X * xDistance + y.X * yDistance, center.Y + x.Y * xDistance + y.Y * yDistance);
        }
    }

    /// <summary>
    /// 表示单个直线检测点。
    /// </summary>
    public sealed class LineEdgePoint
    {
        /// <summary>
        /// 初始化直线检测点。
        /// </summary>
        /// <param name="point">检测点图像坐标。</param>
        /// <param name="strength">检测点边缘强度。</param>
        public LineEdgePoint(PointF point, float strength)
        {
            Point = point;
            Strength = strength;
        }

        /// <summary>
        /// 获取检测点图像坐标。
        /// </summary>
        public PointF Point { get; private set; }

        /// <summary>
        /// 获取检测点边缘强度。
        /// </summary>
        public float Strength { get; private set; }
    }

    /// <summary>
    /// 表示一次直线检测结果。
    /// </summary>
    public sealed class LineDetectionResult
    {
        /// <summary>
        /// 初始化直线检测结果。
        /// </summary>
        /// <param name="success">是否检测成功。</param>
        /// <param name="message">结果消息。</param>
        /// <param name="frame">检测测量框。</param>
        /// <param name="scanDirection">扫描方向。</param>
        /// <param name="lineStart">结果线段起点。</param>
        /// <param name="lineEnd">结果线段终点。</param>
        /// <param name="edgePoints">检测点集合。</param>
        /// <param name="elapsed">检测耗时。</param>
        private LineDetectionResult(bool success, string message, LineDetectionFrame frame, LineScanDirection scanDirection, PointF lineStart, PointF lineEnd, List<LineEdgePoint> edgePoints, TimeSpan elapsed)
        {
            Success = success;
            Message = message;
            Frame = frame;
            ScanDirection = scanDirection;
            LineStart = lineStart;
            LineEnd = lineEnd;
            EdgePoints = (edgePoints ?? new List<LineEdgePoint>()).AsReadOnly();
            Elapsed = elapsed;
            Angle = success ? CalculateAngle(lineStart, lineEnd) : 0f;
            AverageStrength = EdgePoints.Count == 0 ? 0f : EdgePoints.Average(x => x.Strength);
            MaxStrength = EdgePoints.Count == 0 ? 0f : EdgePoints.Max(x => x.Strength);
        }

        /// <summary>
        /// 获取检测是否成功。
        /// </summary>
        public bool Success { get; private set; }

        /// <summary>
        /// 获取检测结果消息。
        /// </summary>
        public string Message { get; private set; }

        /// <summary>
        /// 获取检测测量框。
        /// </summary>
        public LineDetectionFrame Frame { get; private set; }

        /// <summary>
        /// 获取扫描方向。
        /// </summary>
        public LineScanDirection ScanDirection { get; private set; }

        /// <summary>
        /// 获取检测线段起点。
        /// </summary>
        public PointF LineStart { get; private set; }

        /// <summary>
        /// 获取检测线段终点。
        /// </summary>
        public PointF LineEnd { get; private set; }

        /// <summary>
        /// 获取检测线段角度。
        /// </summary>
        public float Angle { get; private set; }

        /// <summary>
        /// 获取平均边缘强度。
        /// </summary>
        public float AverageStrength { get; private set; }

        /// <summary>
        /// 获取最大边缘强度。
        /// </summary>
        public float MaxStrength { get; private set; }

        /// <summary>
        /// 获取检测耗时。
        /// </summary>
        public TimeSpan Elapsed { get; private set; }

        /// <summary>
        /// 获取检测点集合。
        /// </summary>
        public IReadOnlyList<LineEdgePoint> EdgePoints { get; private set; }

        /// <summary>
        /// 创建成功的检测结果。
        /// </summary>
        /// <param name="frame">检测测量框。</param>
        /// <param name="scanDirection">扫描方向。</param>
        /// <param name="lineStart">结果线段起点。</param>
        /// <param name="lineEnd">结果线段终点。</param>
        /// <param name="edgePoints">检测点集合。</param>
        /// <param name="elapsed">检测耗时。</param>
        /// <returns>成功结果。</returns>
        public static LineDetectionResult CreateSuccess(LineDetectionFrame frame, LineScanDirection scanDirection, PointF lineStart, PointF lineEnd, List<LineEdgePoint> edgePoints, TimeSpan elapsed)
        {
            return new LineDetectionResult(true, "检测成功", frame, scanDirection, lineStart, lineEnd, edgePoints, elapsed);
        }

        /// <summary>
        /// 创建失败的检测结果。
        /// </summary>
        /// <param name="message">失败消息。</param>
        /// <param name="frame">检测测量框。</param>
        /// <param name="scanDirection">扫描方向。</param>
        /// <param name="edgePoints">已检测到的点集合。</param>
        /// <param name="elapsed">检测耗时。</param>
        /// <returns>失败结果。</returns>
        public static LineDetectionResult CreateFailure(string message, LineDetectionFrame frame, LineScanDirection scanDirection, List<LineEdgePoint> edgePoints = null, TimeSpan elapsed = default(TimeSpan))
        {
            return new LineDetectionResult(false, message, frame, scanDirection, PointF.Empty, PointF.Empty, edgePoints ?? new List<LineEdgePoint>(), elapsed);
        }

        /// <summary>
        /// 计算线段角度。
        /// </summary>
        /// <param name="start">线段起点。</param>
        /// <param name="end">线段终点。</param>
        /// <returns>0到180度之间的角度。</returns>
        private static float CalculateAngle(PointF start, PointF end)
        {
            float angle = (float)(Math.Atan2(end.Y - start.Y, end.X - start.X) * 180d / Math.PI);
            if (angle < 0f) angle += 180f;
            if (angle >= 180f) angle -= 180f;
            return angle;
        }
    }
}
