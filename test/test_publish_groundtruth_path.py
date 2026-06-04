"""测试真值轨迹发布脚本的增量显示辅助函数。"""

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from evaluate_euroc_trajectory import PoseSample  # noqa: E402
from publish_groundtruth_path import (  # noqa: E402
    align_groundtruth_to_estimate,
    truncate_groundtruth_by_time,
)


class PublishGroundtruthPathTest(unittest.TestCase):
    """验证真值轨迹按 VIO 当前时间逐步截断。"""

    def test_truncate_groundtruth_keeps_only_visible_prefix(self):
        """截断结果应只包含不晚于当前估计时间的真值点。"""
        groundtruth = [
            PoseSample(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(2.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(3.0, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]

        visible_groundtruth = truncate_groundtruth_by_time(groundtruth, 2.1)

        self.assertEqual([sample.timestamp for sample in visible_groundtruth], [1.0, 2.0])

    def test_truncate_groundtruth_keeps_start_before_first_timestamp(self):
        """估计时间早于真值首帧时应保留起点，避免 RViz2 Path 为空。"""
        groundtruth = [
            PoseSample(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(2.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]

        visible_groundtruth = truncate_groundtruth_by_time(groundtruth, 0.5)

        self.assertEqual(len(visible_groundtruth), 1)
        self.assertAlmostEqual(visible_groundtruth[0].timestamp, 1.0)

    def test_align_groundtruth_to_estimate_uses_start_segment_direction(self):
        """真值对齐到估计坐标系时应使用起步段位置方向。"""
        groundtruth = [
            PoseSample(1.0, (10.0, 20.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(2.0, (11.0, 20.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(3.0, (12.0, 20.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
        ]
        estimate_anchor = PoseSample(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        estimate_direction = PoseSample(2.0, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))

        aligned = align_groundtruth_to_estimate(
            groundtruth,
            estimate_anchor,
            groundtruth[0],
            estimate_direction,
            groundtruth[1],
        )

        self.assertAlmostEqual(aligned[1].position[0], 0.0, places=9)
        self.assertAlmostEqual(aligned[1].position[1], 1.0, places=9)
        self.assertAlmostEqual(aligned[2].position[0], 0.0, places=9)
        self.assertAlmostEqual(aligned[2].position[1], 2.0, places=9)


if __name__ == "__main__":
    unittest.main()
