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
        /// <param name="roi">检测ROI。</param>
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
        /// <param name="roi">检测ROI。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>直线检测结果。</returns>
        public LineDetectionResult Detect(LineDetectionImageContext context, RoiItem roi, LineDetectionParams parameters)
        {
            LineDetectionParams actualParams = NormalizeParams(parameters);
            if (context == null || context.GrayPixels == null || context.Width <= 0 || context.Height <= 0)
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

            PointF[] line = FitLine(frame, actualParams, edgePoints);
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
        /// 在ROI内按卡尺排列方向采集检测点。
        /// </summary>
        /// <param name="context">直线检测图像上下文。</param>
        /// <param name="frame">检测测量框。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>检测点集合。</returns>
        private static List<LineEdgePoint> CollectEdgePoints(LineDetectionImageContext context, LineDetectionFrame frame, LineDetectionParams parameters)
        {
            PointF scanDir = frame.GetScanDirection(parameters.ScanDirection);
            PointF arrangeDir = frame.GetLineArrangeDirection(parameters.ScanDirection);
            float arrangeLength = frame.GetArrangeLength(parameters.ScanDirection);
            float scanLength = frame.GetScanLength(parameters.ScanDirection);
            float caliperHalfWidth = Math.Max(0.5f, arrangeLength / parameters.SampleCount * 0.5f);
            float arrangeStart = -arrangeLength / 2f + caliperHalfWidth;
            float arrangeEnd = arrangeLength / 2f - caliperHalfWidth;
            float arrangeStep = parameters.SampleCount == 1 ? 0f : (arrangeEnd - arrangeStart) / (parameters.SampleCount - 1);
            int scanCount = Math.Max(3, (int)Math.Ceiling(scanLength / parameters.SampleStep) + 1);
            float[] profile = new float[scanCount];
            float[] smoothed = new float[scanCount];
            int[] counts = new int[scanCount];
            List<LineEdgePoint> result = new List<LineEdgePoint>(parameters.SampleCount);

            for (int i = 0; i < parameters.SampleCount; i++)
            {
                float along = arrangeStart + arrangeStep * i;
                PointF center = new PointF(frame.Center.X + arrangeDir.X * along, frame.Center.Y + arrangeDir.Y * along);
                CaliperCandidate candidate = DetectOneCaliper(context, center, scanDir, arrangeDir, scanLength, caliperHalfWidth, parameters, profile, smoothed, counts);
                if (candidate.IsValid)
                {
                    result.Add(new LineEdgePoint(candidate.Point, candidate.Strength));
                }
            }

            return result;
        }

        /// <summary>
        /// 在单条卡尺线上检测候选边缘并按参数选择一个检测点。
        /// </summary>
        /// <param name="context">直线检测图像上下文。</param>
        /// <param name="center">卡尺中心点。</param>
        /// <param name="scanDir">扫描方向。</param>
        /// <param name="widthDir">卡尺宽度方向。</param>
        /// <param name="scanLength">扫描长度。</param>
        /// <param name="halfWidth">卡尺半宽。</param>
        /// <param name="parameters">检测参数。</param>
        /// <param name="profile">灰度曲线缓存。</param>
        /// <param name="smoothed">平滑曲线缓存。</param>
        /// <param name="counts">采样计数缓存。</param>
        /// <returns>选中的卡尺候选点。</returns>
        private static CaliperCandidate DetectOneCaliper(LineDetectionImageContext context, PointF center, PointF scanDir, PointF widthDir, float scanLength, float halfWidth, LineDetectionParams parameters, float[] profile, float[] smoothed, int[] counts)
        {
            int scanCount = profile.Length;
            int widthCount = Math.Max(1, (int)Math.Ceiling((halfWidth * 2f) / parameters.SampleStep));
            float scanStep = scanLength / Math.Max(1, scanCount - 1);
            float widthStep = widthCount == 1 ? 0f : (halfWidth * 2f) / (widthCount - 1);
            float scanStart = -scanLength / 2f;
            float widthStart = -halfWidth;

            for (int i = 0; i < scanCount; i++)
            {
                profile[i] = 0f;
                smoothed[i] = 0f;
                counts[i] = 0;
            }

            for (int scanIndex = 0; scanIndex < scanCount; scanIndex++)
            {
                float scanOffset = scanStart + scanIndex * scanStep;
                float baseX = center.X + scanDir.X * scanOffset;
                float baseY = center.Y + scanDir.Y * scanOffset;
                int sum = 0;
                int count = 0;

                for (int widthIndex = 0; widthIndex < widthCount; widthIndex++)
                {
                    float widthOffset = widthStart + widthIndex * widthStep;
                    int x = (int)Math.Round(baseX + widthDir.X * widthOffset);
                    int y = (int)Math.Round(baseY + widthDir.Y * widthOffset);
                    if (x < 0 || y < 0 || x >= context.Width || y >= context.Height) continue;

                    sum += context.GrayPixels[y * context.Width + x];
                    count++;
                }

                if (count > 0)
                {
                    profile[scanIndex] = (float)sum / count;
                    counts[scanIndex] = count;
                }
            }

            SmoothProfile(profile, smoothed, counts, parameters.SmoothSize);
            return SelectCandidate(smoothed, counts, center, scanDir, scanStart, scanStep, parameters);
        }

        /// <summary>
        /// 对一维灰度曲线做滑动平均平滑。
        /// </summary>
        /// <param name="source">原始灰度曲线。</param>
        /// <param name="target">平滑后的灰度曲线。</param>
        /// <param name="counts">有效采样计数。</param>
        /// <param name="size">平滑窗口大小。</param>
        private static void SmoothProfile(float[] source, float[] target, int[] counts, int size)
        {
            int length = source.Length;
            if (size <= 1)
            {
                for (int i = 0; i < length; i++) target[i] = source[i];
                return;
            }

            int half = size / 2;
            for (int i = 0; i < length; i++)
            {
                float sum = 0f;
                int count = 0;
                int start = Math.Max(0, i - half);
                int end = Math.Min(length - 1, i + half);
                for (int j = start; j <= end; j++)
                {
                    if (counts[j] <= 0) continue;
                    sum += source[j];
                    count++;
                }
                target[i] = count == 0 ? source[i] : sum / count;
            }
        }

        /// <summary>
        /// 从一维灰度曲线中按极性和选择方式获取候选点。
        /// </summary>
        /// <param name="profile">平滑后的灰度曲线。</param>
        /// <param name="counts">有效采样计数。</param>
        /// <param name="center">卡尺中心点。</param>
        /// <param name="scanDir">扫描方向。</param>
        /// <param name="scanStart">扫描起始偏移。</param>
        /// <param name="scanStep">扫描步长。</param>
        /// <param name="parameters">检测参数。</param>
        /// <returns>选中的候选点。</returns>
        private static CaliperCandidate SelectCandidate(float[] profile, int[] counts, PointF center, PointF scanDir, float scanStart, float scanStep, LineDetectionParams parameters)
        {
            CaliperCandidate selected = CaliperCandidate.Invalid;
            int length = profile.Length;
            for (int i = 1; i < length - 1; i++)
            {
                if (counts[i] <= 0 || counts[i - 1] <= 0 || counts[i + 1] <= 0) continue;

                float gradient = (profile[i + 1] - profile[i - 1]) / Math.Max(0.0001f, scanStep * 2f);
                float strength = Math.Abs(gradient);
                if (strength < parameters.EdgeThreshold) continue;
                if (!MatchPolarity(gradient, parameters.EdgePolarity)) continue;

                float prevStrength = Math.Abs((profile[i] - profile[i - 1]) / Math.Max(0.0001f, scanStep));
                float nextStrength = Math.Abs((profile[i + 1] - profile[i]) / Math.Max(0.0001f, scanStep));
                if (strength < prevStrength || strength < nextStrength) continue;

                float offset = scanStart + i * scanStep;
                CaliperCandidate candidate = new CaliperCandidate(
                    new PointF(center.X + scanDir.X * offset, center.Y + scanDir.Y * offset),
                    offset,
                    strength);

                if (!selected.IsValid || IsBetterCandidate(candidate, selected, parameters.SelectionMode))
                {
                    selected = candidate;
                }
            }

            return selected;
        }

        /// <summary>
        /// 判断候选点是否优于当前已选点。
        /// </summary>
        /// <param name="candidate">新的候选点。</param>
        /// <param name="selected">当前已选点。</param>
        /// <param name="mode">边缘选择方式。</param>
        /// <returns>新候选点更优时返回true。</returns>
        private static bool IsBetterCandidate(CaliperCandidate candidate, CaliperCandidate selected, LineSelectionMode mode)
        {
            switch (mode)
            {
                case LineSelectionMode.First:
                    return candidate.Offset < selected.Offset;
                case LineSelectionMode.Last:
                    return candidate.Offset > selected.Offset;
                default:
                    return candidate.Strength > selected.Strength;
            }
        }

        /// <summary>
        /// 判断梯度方向是否符合边缘极性。
        /// </summary>
        /// <param name="gradient">灰度梯度。</param>
        /// <param name="polarity">边缘极性。</param>
        /// <returns>符合极性时返回true。</returns>
        private static bool MatchPolarity(float gradient, LineEdgePolarity polarity)
        {
            switch (polarity)
            {
                case LineEdgePolarity.Positive:
                    return gradient > 0f;
                case LineEdgePolarity.Negative:
                    return gradient < 0f;
                default:
                    return true;
            }
        }

        /// <summary>
        /// 将检测点拟合为ROI内线段。
        /// </summary>
        /// <param name="frame">检测测量框。</param>
        /// <param name="parameters">检测参数。</param>
        /// <param name="edgePoints">检测点集合。</param>
        /// <returns>线段起点和终点。</returns>
        private static PointF[] FitLine(LineDetectionFrame frame, LineDetectionParams parameters, List<LineEdgePoint> edgePoints)
        {
            Point2f[] points = new Point2f[edgePoints.Count];
            for (int i = 0; i < edgePoints.Count; i++)
            {
                points[i] = new Point2f(edgePoints[i].Point.X, edgePoints[i].Point.Y);
            }

            Line2D line = parameters.FitMode == LineFitMode.LeastSquares
                ? Cv2.FitLine(points, DistanceTypes.L2, 0, 0.01, 0.01)
                : Cv2.FitLine(points, DistanceTypes.Welsch, 0, 0.01, 0.01);

            PointF direction = new PointF((float)line.Vx, (float)line.Vy);
            PointF point = new PointF((float)line.X1, (float)line.Y1);
            PointF arrangeDir = frame.GetLineArrangeDirection(parameters.ScanDirection);
            if (direction.X * arrangeDir.X + direction.Y * arrangeDir.Y < 0f)
            {
                direction = new PointF(-direction.X, -direction.Y);
            }

            float halfLength = frame.GetArrangeLength(parameters.ScanDirection) / 2f;
            PointF start = new PointF(point.X - direction.X * halfLength, point.Y - direction.Y * halfLength);
            PointF end = new PointF(point.X + direction.X * halfLength, point.Y + direction.Y * halfLength);
            return ClipLineToFrame(point, direction, frame, out PointF clippedStart, out PointF clippedEnd)
                ? new[] { clippedStart, clippedEnd }
                : new[] { start, end };
        }

        /// <summary>
        /// 将无限直线裁剪到检测测量框内部。
        /// </summary>
        /// <param name="point">直线上的点。</param>
        /// <param name="direction">直线方向。</param>
        /// <param name="frame">检测测量框。</param>
        /// <param name="start">裁剪后的起点。</param>
        /// <param name="end">裁剪后的终点。</param>
        /// <returns>裁剪成功时返回true。</returns>
        private static bool ClipLineToFrame(PointF point, PointF direction, LineDetectionFrame frame, out PointF start, out PointF end)
        {
            start = PointF.Empty;
            end = PointF.Empty;
            PointF[] corners = frame.GetCorners();
            List<PointF> intersections = new List<PointF>(4);

            for (int i = 0; i < corners.Length; i++)
            {
                PointF a = corners[i];
                PointF b = corners[(i + 1) % corners.Length];
                if (TryIntersectInfiniteLineWithSegment(point, direction, a, b, out PointF intersection))
                {
                    AddUniquePoint(intersections, intersection);
                }
            }

            if (intersections.Count < 2) return false;

            float maxDistance = float.NegativeInfinity;
            for (int i = 0; i < intersections.Count - 1; i++)
            {
                for (int j = i + 1; j < intersections.Count; j++)
                {
                    float dx = intersections[i].X - intersections[j].X;
                    float dy = intersections[i].Y - intersections[j].Y;
                    float distance = dx * dx + dy * dy;
                    if (distance > maxDistance)
                    {
                        maxDistance = distance;
                        start = intersections[i];
                        end = intersections[j];
                    }
                }
            }

            return maxDistance > 0.0001f;
        }

        /// <summary>
        /// 计算无限直线和线段的交点。
        /// </summary>
        /// <param name="linePoint">直线上的点。</param>
        /// <param name="lineDirection">直线方向。</param>
        /// <param name="segStart">线段起点。</param>
        /// <param name="segEnd">线段终点。</param>
        /// <param name="intersection">交点。</param>
        /// <returns>存在有效交点时返回true。</returns>
        private static bool TryIntersectInfiniteLineWithSegment(PointF linePoint, PointF lineDirection, PointF segStart, PointF segEnd, out PointF intersection)
        {
            intersection = PointF.Empty;
            PointF segmentDirection = new PointF(segEnd.X - segStart.X, segEnd.Y - segStart.Y);
            float denominator = lineDirection.X * segmentDirection.Y - lineDirection.Y * segmentDirection.X;
            if (Math.Abs(denominator) < 0.0001f) return false;

            PointF delta = new PointF(segStart.X - linePoint.X, segStart.Y - linePoint.Y);
            float u = (delta.X * lineDirection.Y - delta.Y * lineDirection.X) / denominator;
            if (u < -0.0001f || u > 1.0001f) return false;

            intersection = new PointF(segStart.X + segmentDirection.X * u, segStart.Y + segmentDirection.Y * u);
            return true;
        }

        /// <summary>
        /// 向点集合中添加非重复点。
        /// </summary>
        /// <param name="points">点集合。</param>
        /// <param name="point">待添加点。</param>
        private static void AddUniquePoint(List<PointF> points, PointF point)
        {
            foreach (PointF existing in points)
            {
                float dx = existing.X - point.X;
                float dy = existing.Y - point.Y;
                if (dx * dx + dy * dy < 0.01f) return;
            }
            points.Add(point);
        }

        /// <summary>
        /// 表示卡尺线上的候选边缘点。
        /// </summary>
        private struct CaliperCandidate
        {
            public static readonly CaliperCandidate Invalid = new CaliperCandidate(PointF.Empty, 0f, 0f, false);

            public readonly PointF Point;
            public readonly float Offset;
            public readonly float Strength;
            public readonly bool IsValid;

            /// <summary>
            /// 初始化候选边缘点。
            /// </summary>
            /// <param name="point">候选点坐标。</param>
            /// <param name="offset">扫描方向偏移。</param>
            /// <param name="strength">边缘强度。</param>
            public CaliperCandidate(PointF point, float offset, float strength)
                : this(point, offset, strength, true)
            {
            }

            /// <summary>
            /// 初始化候选边缘点。
            /// </summary>
            /// <param name="point">候选点坐标。</param>
            /// <param name="offset">扫描方向偏移。</param>
            /// <param name="strength">边缘强度。</param>
            /// <param name="isValid">候选点是否有效。</param>
            private CaliperCandidate(PointF point, float offset, float strength, bool isValid)
            {
                Point = point;
                Offset = offset;
                Strength = strength;
                IsValid = isValid;
            }
        }
    }
}
