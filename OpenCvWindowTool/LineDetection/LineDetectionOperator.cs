using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace OpenCvWindowTool
{
    /// <summary>
    /// 执行基于卡尺灰度投影的直线检测。
    /// </summary>
    public sealed class LineDetectionOperator
    {
        /// <summary>
        /// 使用源图像执行直线检测，调用方没有缓存上下文时使用。
        /// </summary>
        /// <param name="image">源图像。</param>
        /// <param name="roi">检测 ROI。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>直线检测结果。</returns>
        public LineDetectionResult Detect(Mat image, RoiItem roi, LineDetectionParams parameters)
        {
            using (LineDetectionImageContext context = LineDetectionImageContext.FromImage(image))
            {
                return Detect(context, roi, parameters);
            }
        }

        /// <summary>
        /// 使用预处理图像上下文执行直线检测。
        /// </summary>
        /// <param name="context">直线检测图像上下文。</param>
        /// <param name="roi">检测 ROI。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>直线检测结果。</returns>
        public LineDetectionResult Detect(LineDetectionImageContext context, RoiItem roi, LineDetectionParams parameters)
        {
            LineDetectionParams actualParams = NormalizeParams(parameters);
            if (context == null || context.GrayImage == null || context.GrayImage.Empty() || context.Width <= 0 || context.Height <= 0)
            {
                return LineDetectionResult.CreateFailure("当前没有可检测的图像。", default(LineDetectionFrame), actualParams.ScanDirection);
            }

            if (roi == null)
            {
                return LineDetectionResult.CreateFailure("请先选择普通矩形ROI或带角度矩形ROI。", default(LineDetectionFrame), actualParams.ScanDirection);
            }

            if (!roi.CanDetectLine())
            {
                return LineDetectionResult.CreateFailure("直线检测只支持普通矩形ROI和带角度矩形ROI。", default(LineDetectionFrame), actualParams.ScanDirection);
            }

            Stopwatch stopwatch = Stopwatch.StartNew();
            LineDetectionFrame frame = roi.ToLineDetectionFrame();
            List<LineEdgePoint> edgePoints = CollectEdgePoints(context, frame, actualParams);
            if (edgePoints.Count < 2)
            {
                stopwatch.Stop();
                return LineDetectionResult.CreateFailure("有效检测点不足，无法拟合直线。", frame, actualParams.ScanDirection, edgePoints, stopwatch.Elapsed);
            }

            PointF[] line = FitLine(edgePoints);
            stopwatch.Stop();
            return LineDetectionResult.CreateSuccess(frame, actualParams.ScanDirection, line[0], line[1], edgePoints, stopwatch.Elapsed);
        }

        /// <summary>
        /// 归一化检测参数，避免无效参数进入热路径。
        /// </summary>
        /// <param name="parameters">原始检测参数。</param>
        /// <returns>归一化后的检测参数。</returns>
        private static LineDetectionParams NormalizeParams(LineDetectionParams parameters)
        {
            LineDetectionParams source = parameters ?? new LineDetectionParams();
            LineDetectionParams result = new LineDetectionParams
            {
                EdgeThreshold = Math.Max(0f, source.EdgeThreshold),
                SampleCount = Math.Max(2, source.SampleCount),
                SampleStep = Math.Max(0.5f, source.SampleStep),
                SmoothSize = Math.Max(1, source.SmoothSize),
                EdgePolarity = source.EdgePolarity,
                SelectionMode = source.SelectionMode,
                ScanDirection = source.ScanDirection,
                FitMode = source.FitMode
            };
            if (result.SmoothSize % 2 == 0) result.SmoothSize++;
            return result;
        }

        /// <summary>
        /// 在 ROI 内按卡尺排列方向采集检测点。
        /// </summary>
        /// <param name="context">直线检测图像上下文。</param>
        /// <param name="frame">检测测量框。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>检测点集合。</returns>
        private static List<LineEdgePoint> CollectEdgePoints(LineDetectionImageContext context, LineDetectionFrame frame, LineDetectionParams parameters)
        {
            List<LineEdgePoint> result = new List<LineEdgePoint>(parameters.SampleCount);
            List<CaliperInfo> calipers = GenerateCalipers(frame, parameters);

            foreach (CaliperInfo caliper in calipers)
            {
                List<EdgePoint> edges = FindEdgesInCaliper(context.GrayImage, caliper, parameters);
                EdgePoint selected = SelectPointByTransition(edges, parameters.SelectionMode);
                if (selected.IsValid)
                {
                    result.Add(new LineEdgePoint(selected.Position, selected.Strength));
                }
            }

            return result;
        }

        /// <summary>
        /// 按旧项目卡尺模型在 ROI 中生成等间距卡尺。
        /// </summary>
        /// <param name="frame">检测测量框。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>卡尺集合。</returns>
        private static List<CaliperInfo> GenerateCalipers(LineDetectionFrame frame, LineDetectionParams parameters)
        {
            PointF arrangeDir = frame.GetLineArrangeDirection(parameters.ScanDirection);
            PointF scanDir = frame.GetScanDirection(parameters.ScanDirection);
            float arrangeLength = frame.GetArrangeLength(parameters.ScanDirection);
            float scanLength = frame.GetScanLength(parameters.ScanDirection);
            float halfArrange = arrangeLength / 2f;
            PointF segmentStart = new PointF(frame.Center.X - arrangeDir.X * halfArrange, frame.Center.Y - arrangeDir.Y * halfArrange);
            float spacing = arrangeLength / (parameters.SampleCount + 1f);
            float caliperWidth = Math.Max(1f, arrangeLength / parameters.SampleCount);

            List<CaliperInfo> calipers = new List<CaliperInfo>(parameters.SampleCount);
            for (int i = 1; i <= parameters.SampleCount; i++)
            {
                float offset = i * spacing;
                PointF center = new PointF(segmentStart.X + arrangeDir.X * offset, segmentStart.Y + arrangeDir.Y * offset);
                calipers.Add(new CaliperInfo(center, scanDir, arrangeDir, scanLength, caliperWidth));
            }

            return calipers;
        }

        /// <summary>
        /// 在单条卡尺中查找满足阈值、极性和局部峰值条件的边缘点。
        /// </summary>
        /// <param name="grayImage">灰度图像。</param>
        /// <param name="caliper">卡尺信息。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>候选边缘点集合。</returns>
        private static List<EdgePoint> FindEdgesInCaliper(Mat grayImage, CaliperInfo caliper, LineDetectionParams parameters)
        {
            List<EdgePoint> edges = new List<EdgePoint>();
            double[] profile = ExtractCaliperProfile(grayImage, caliper);
            if (profile == null || profile.Length < 2)
            {
                return edges;
            }

            double[] gradient = new double[profile.Length];
            for (int i = 1; i < profile.Length - 1; i++)
            {
                gradient[i] = (profile[i + 1] - profile[i - 1]) / 2.0;
            }

            for (int i = 1; i < gradient.Length - 1; i++)
            {
                double strength = Math.Abs(gradient[i]);
                if (strength < parameters.EdgeThreshold)
                {
                    continue;
                }

                if (!MatchPolarity(gradient[i], parameters.EdgePolarity))
                {
                    continue;
                }

                if (Math.Abs(gradient[i]) > Math.Abs(gradient[i - 1]) &&
                    Math.Abs(gradient[i]) > Math.Abs(gradient[i + 1]))
                {
                    double offset = (i / (double)profile.Length - 0.5) * caliper.Length;
                    PointF position = new PointF(
                        (float)(caliper.Center.X + offset * caliper.Direction.X),
                        (float)(caliper.Center.Y + offset * caliper.Direction.Y));

                    edges.Add(new EdgePoint(position, (float)strength));
                }
            }

            return edges;
        }

        /// <summary>
        /// 将卡尺区域仿射拉正后，沿宽度方向平均得到一维灰度曲线。
        /// </summary>
        /// <param name="image">灰度图像。</param>
        /// <param name="caliper">卡尺信息。</param>
        /// <returns>一维灰度曲线；卡尺尺寸无效时返回 null。</returns>
        private static double[] ExtractCaliperProfile(Mat image, CaliperInfo caliper)
        {
            int length = (int)caliper.Length;
            int width = (int)caliper.Width;
            if (length <= 0 || width <= 0)
            {
                return null;
            }

            using (Mat transformMatrix = GetCaliperTransformMatrix(caliper, length, width))
            using (Mat croppedRegion = new Mat())
            {
                Cv2.WarpAffine(
                    image,
                    croppedRegion,
                    transformMatrix,
                    new OpenCvSharp.Size(length, width),
                    InterpolationFlags.Linear,
                    BorderTypes.Constant,
                    Scalar.All(0));

                double[] profile = new double[length];
                for (int x = 0; x < length; x++)
                {
                    double sum = 0d;
                    int count = 0;
                    for (int y = 0; y < width; y++)
                    {
                        sum += croppedRegion.At<byte>(y, x);
                        count++;
                    }

                    profile[x] = count > 0 ? sum / count : 0d;
                }

                return profile;
            }
        }

        /// <summary>
        /// 计算从原图坐标到卡尺矩形坐标的仿射矩阵。
        /// </summary>
        /// <param name="caliper">卡尺信息。</param>
        /// <param name="length">拉正后卡尺图像长度。</param>
        /// <param name="width">拉正后卡尺图像宽度。</param>
        /// <returns>仿射变换矩阵。</returns>
        private static Mat GetCaliperTransformMatrix(CaliperInfo caliper, int length, int width)
        {
            Point2f[] srcPoints = new Point2f[3];
            srcPoints[0] = new Point2f(0, 0);
            srcPoints[1] = new Point2f(length - 1, 0);
            srcPoints[2] = new Point2f(0, width - 1);

            double halfLength = caliper.Length / 2.0;
            double halfWidth = caliper.Width / 2.0;
            Point2f[] dstPoints = new Point2f[3];
            dstPoints[0] = new Point2f(
                (float)(caliper.Center.X - halfLength * caliper.Direction.X - halfWidth * caliper.Perpendicular.X),
                (float)(caliper.Center.Y - halfLength * caliper.Direction.Y - halfWidth * caliper.Perpendicular.Y));
            dstPoints[1] = new Point2f(
                (float)(caliper.Center.X + halfLength * caliper.Direction.X - halfWidth * caliper.Perpendicular.X),
                (float)(caliper.Center.Y + halfLength * caliper.Direction.Y - halfWidth * caliper.Perpendicular.Y));
            dstPoints[2] = new Point2f(
                (float)(caliper.Center.X - halfLength * caliper.Direction.X + halfWidth * caliper.Perpendicular.X),
                (float)(caliper.Center.Y - halfLength * caliper.Direction.Y + halfWidth * caliper.Perpendicular.Y));

            return Cv2.GetAffineTransform(dstPoints, srcPoints);
        }

        /// <summary>
        /// 按边缘选择模式从单卡尺候选点中选出一个检测点。
        /// </summary>
        /// <param name="edges">单卡尺候选边缘点。</param>
        /// <param name="mode">边缘选择模式。</param>
        /// <returns>选中的边缘点；没有候选点时返回无效点。</returns>
        private static EdgePoint SelectPointByTransition(List<EdgePoint> edges, LineSelectionMode mode)
        {
            if (edges.Count == 0)
            {
                return EdgePoint.Invalid;
            }

            switch (mode)
            {
                case LineSelectionMode.First:
                    return edges[0];
                case LineSelectionMode.Last:
                    return edges[edges.Count - 1];
                default:
                    EdgePoint strongest = edges[0];
                    for (int i = 1; i < edges.Count; i++)
                    {
                        if (edges[i].Strength > strongest.Strength)
                        {
                            strongest = edges[i];
                        }
                    }
                    return strongest;
            }
        }

        /// <summary>
        /// 判断梯度方向是否符合边缘极性设置。
        /// </summary>
        /// <param name="gradient">灰度梯度。</param>
        /// <param name="polarity">边缘极性。</param>
        /// <returns>符合极性时返回 true。</returns>
        private static bool MatchPolarity(double gradient, LineEdgePolarity polarity)
        {
            switch (polarity)
            {
                case LineEdgePolarity.Positive:
                    return gradient > 0d;
                case LineEdgePolarity.Negative:
                    return gradient < 0d;
                default:
                    return true;
            }
        }

        /// <summary>
        /// 使用旧项目最小二乘公式将检测点拟合成直线段。
        /// </summary>
        /// <param name="edgePoints">检测点集合。</param>
        /// <returns>拟合线段起点和终点。</returns>
        private static PointF[] FitLine(List<LineEdgePoint> edgePoints)
        {
            int n = edgePoints.Count;
            double sumX = 0d;
            double sumY = 0d;
            double sumXY = 0d;
            double sumX2 = 0d;
            double sumY2 = 0d;
            double minX = double.MaxValue;
            double maxX = double.MinValue;
            double minY = double.MaxValue;
            double maxY = double.MinValue;

            foreach (LineEdgePoint edgePoint in edgePoints)
            {
                double x = edgePoint.Point.X;
                double y = edgePoint.Point.Y;
                sumX += x;
                sumY += y;
                sumXY += x * y;
                sumX2 += x * x;
                sumY2 += y * y;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }

            double denom = n * sumX2 - sumX * sumX;
            if (Math.Abs(denom) < 0.0001d)
            {
                double verticalDenom = n * sumY2 - sumY * sumY;
                if (Math.Abs(verticalDenom) < 0.0001d)
                {
                    PointF point = edgePoints[0].Point;
                    return new[] { point, point };
                }

                double m = (n * sumXY - sumX * sumY) / verticalDenom;
                double b = (sumX - m * sumY) / n;
                return new[]
                {
                    new PointF((float)(m * minY + b), (float)minY),
                    new PointF((float)(m * maxY + b), (float)maxY)
                };
            }

            double slope = (n * sumXY - sumX * sumY) / denom;
            double intercept = (sumY - slope * sumX) / n;
            return new[]
            {
                new PointF((float)minX, (float)(slope * minX + intercept)),
                new PointF((float)maxX, (float)(slope * maxX + intercept))
            };
        }

        /// <summary>
        /// 保存单条卡尺的几何参数。
        /// </summary>
        private struct CaliperInfo
        {
            public readonly PointF Center;
            public readonly PointF Direction;
            public readonly PointF Perpendicular;
            public readonly float Length;
            public readonly float Width;

            /// <summary>
            /// 初始化单条卡尺的几何参数。
            /// </summary>
            /// <param name="center">卡尺中心点。</param>
            /// <param name="direction">卡尺搜索方向。</param>
            /// <param name="perpendicular">卡尺宽度方向。</param>
            /// <param name="length">卡尺搜索长度。</param>
            /// <param name="width">卡尺宽度。</param>
            public CaliperInfo(PointF center, PointF direction, PointF perpendicular, float length, float width)
            {
                Center = center;
                Direction = direction;
                Perpendicular = perpendicular;
                Length = length;
                Width = width;
            }
        }

        /// <summary>
        /// 保存单条卡尺中的候选边缘点。
        /// </summary>
        private struct EdgePoint
        {
            public static readonly EdgePoint Invalid = new EdgePoint(PointF.Empty, 0f, false);

            public readonly PointF Position;
            public readonly float Strength;
            public readonly bool IsValid;

            /// <summary>
            /// 初始化有效候选边缘点。
            /// </summary>
            /// <param name="position">候选点坐标。</param>
            /// <param name="strength">边缘强度。</param>
            public EdgePoint(PointF position, float strength)
                : this(position, strength, true)
            {
            }

            /// <summary>
            /// 初始化候选边缘点。
            /// </summary>
            /// <param name="position">候选点坐标。</param>
            /// <param name="strength">边缘强度。</param>
            /// <param name="isValid">候选点是否有效。</param>
            private EdgePoint(PointF position, float strength, bool isValid)
            {
                Position = position;
                Strength = strength;
                IsValid = isValid;
            }
        }
    }
}
