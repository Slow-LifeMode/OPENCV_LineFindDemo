using OpenCvWindowTool;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace OpenCvWindowToolDemo
{
    /// <summary>
    /// 提供WinForms示例窗口，用于验证图像控件基础能力。
    /// </summary>
    public sealed class MainForm : Form
    {
        private readonly OpenCvImageViewer viewer;
        private readonly NumericUpDown thresholdBox;
        private readonly NumericUpDown sampleCountBox;
        private readonly ComboBox polarityBox;
        private readonly ComboBox selectionModeBox;
        private readonly ComboBox fitModeBox;
        private readonly ToolStripLabel resultLabel;

        /// <summary>
        /// 初始化WinForms示例窗口。
        /// </summary>
        public MainForm()
        {
            Text = "OpenCV 图像控件测试";
            Width = 1280;
            Height = 820;
            StartPosition = FormStartPosition.CenterScreen;

            ToolStrip toolStrip = new ToolStrip { GripStyle = ToolStripGripStyle.Hidden };
            toolStrip.Items.Add("打开图像", null, (s, e) => OpenImage());
            toolStrip.Items.Add("添加叠加", null, (s, e) => AddDemoOverlay());
            toolStrip.Items.Add("清除叠加", null, (s, e) => viewer.ClearOverlays());
            toolStrip.Items.Add(new ToolStripSeparator());
            toolStrip.Items.Add("创建矩形检测ROI", null, (s, e) => viewer.StartCreateRoi(RoiShape.Rectangle));
            toolStrip.Items.Add("创建带角度检测ROI", null, (s, e) => viewer.StartCreateRoi(RoiShape.RotatedRectangle));
            toolStrip.Items.Add("创建圆环ROI", null, (s, e) => viewer.StartCreateRoi(RoiShape.Ring));
            toolStrip.Items.Add("清除ROI", null, (s, e) => viewer.ClearRois());
            toolStrip.Items.Add(new ToolStripSeparator());

            thresholdBox = CreateNumberBox(20, 0, 1000, 5);
            sampleCountBox = CreateNumberBox(40, 2, 500, 1);
            polarityBox = CreateEnumBox(typeof(LineEdgePolarity), LineEdgePolarity.Any);
            selectionModeBox = CreateEnumBox(typeof(LineSelectionMode), LineSelectionMode.Strongest);
            fitModeBox = CreateEnumBox(typeof(LineFitMode), LineFitMode.Robust);
            resultLabel = new ToolStripLabel("直线检测: 未执行");

            toolStrip.Items.Add("边缘强度");
            toolStrip.Items.Add(new ToolStripControlHost(thresholdBox));
            toolStrip.Items.Add("检测点数");
            toolStrip.Items.Add(new ToolStripControlHost(sampleCountBox));
            toolStrip.Items.Add("方向");
            toolStrip.Items.Add(new ToolStripControlHost(polarityBox));
            toolStrip.Items.Add("选择");
            toolStrip.Items.Add(new ToolStripControlHost(selectionModeBox));
            toolStrip.Items.Add("拟合");
            toolStrip.Items.Add(new ToolStripControlHost(fitModeBox));
            toolStrip.Items.Add("执行直线检测", null, (s, e) => DetectLine());
            toolStrip.Items.Add("清除检测结果", null, (s, e) => ClearLineResult());
            toolStrip.Items.Add(resultLabel);

            viewer = new OpenCvImageViewer { Dock = DockStyle.Fill };
            Controls.Add(viewer);
            Controls.Add(toolStrip);
        }

        /// <summary>
        /// 打开图像文件并加载到图像控件。
        /// </summary>
        private void OpenImage()
        {
            using (OpenFileDialog dialog = new OpenFileDialog())
            {
                dialog.Filter = "图像文件|*.bmp;*.jpg;*.jpeg;*.png;*.tif;*.tiff|所有文件|*.*";
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                viewer.LoadImage(dialog.FileName);
                resultLabel.Text = "直线检测: 未执行";
            }
        }

        /// <summary>
        /// 使用当前参数执行直线检测。
        /// </summary>
        private void DetectLine()
        {
            LineDetectionResult result = viewer.DetectLine(viewer.SelectedRoi, CreateLineDetectionParams());
            resultLabel.Text = string.Format("直线检测: {0}, 点数 {1}", result.Message, result.EdgePoints.Count);
            if (!result.Success)
            {
                MessageBox.Show(this, result.Message, "直线检测", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
        }

        /// <summary>
        /// 清除当前直线检测结果显示。
        /// </summary>
        private void ClearLineResult()
        {
            viewer.ClearLineDetectionResult();
            resultLabel.Text = "直线检测: 已清除";
        }

        /// <summary>
        /// 根据工具栏控件创建直线检测参数。
        /// </summary>
        /// <returns>直线检测参数。</returns>
        private LineDetectionParams CreateLineDetectionParams()
        {
            return new LineDetectionParams
            {
                EdgeThreshold = (float)thresholdBox.Value,
                SampleCount = (int)sampleCountBox.Value,
                EdgePolarity = (LineEdgePolarity)polarityBox.SelectedItem,
                SelectionMode = (LineSelectionMode)selectionModeBox.SelectedItem,
                FitMode = (LineFitMode)fitModeBox.SelectedItem
            };
        }

        /// <summary>
        /// 添加示例叠加图形。
        /// </summary>
        private void AddDemoOverlay()
        {
            List<OverlayItem> items = new List<OverlayItem>
            {
                OverlayItem.CreateRectangle(new RectangleF(80, 80, 220, 160), Color.Lime, 2f),
                OverlayItem.Circle(new PointF(420, 220), 80, Color.DeepSkyBlue, 2f),
                OverlayItem.Line(new PointF(40, 40), new PointF(520, 320), Color.Yellow, 2f),
                OverlayItem.Cross(new PointF(320, 180), 45, Color.Red, 2f),
                OverlayItem.TextItem("OpenCV", new PointF(90, 60), Color.Orange, new Font("Microsoft YaHei UI", 14f, FontStyle.Bold))
            };
            viewer.AddOverlays(items);
        }

        /// <summary>
        /// 创建数值输入框。
        /// </summary>
        /// <param name="value">默认值。</param>
        /// <param name="minimum">最小值。</param>
        /// <param name="maximum">最大值。</param>
        /// <param name="increment">步进值。</param>
        /// <returns>数值输入框。</returns>
        private static NumericUpDown CreateNumberBox(decimal value, decimal minimum, decimal maximum, decimal increment)
        {
            return new NumericUpDown
            {
                Width = 64,
                Minimum = minimum,
                Maximum = maximum,
                Value = value,
                Increment = increment,
                DecimalPlaces = 0
            };
        }

        /// <summary>
        /// 创建枚举下拉框。
        /// </summary>
        /// <param name="enumType">枚举类型。</param>
        /// <param name="selected">默认选中值。</param>
        /// <returns>枚举下拉框。</returns>
        private static ComboBox CreateEnumBox(System.Type enumType, object selected)
        {
            ComboBox comboBox = new ComboBox
            {
                Width = 100,
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            foreach (object value in System.Enum.GetValues(enumType))
            {
                comboBox.Items.Add(value);
            }
            comboBox.SelectedItem = selected;
            return comboBox;
        }
    }
}
