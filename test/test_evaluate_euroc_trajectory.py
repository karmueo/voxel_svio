"""测试 EuRoC 轨迹评估脚本的时间匹配、对齐和误差统计。"""

import math
import unittest
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from evaluate_euroc_trajectory import (  # noqa: E402
    PoseSample,
    align_estimates_to_groundtruth,
    compute_ate,
    match_by_timestamp,
)


class EvaluateEurocTrajectoryTest(unittest.TestCase):
    """验证轨迹评估核心函数。"""

    def test_match_by_timestamp_uses_nearest_samples_with_threshold(self):
        """时间戳匹配应选择阈值内最近的真值样本。"""
        estimates = [
            PoseSample(1.00, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(1.11, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(2.00, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        groundtruth = [
            PoseSample(1.02, (10.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(1.09, (11.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]

        pairs = match_by_timestamp(estimates, groundtruth, max_dt=0.05)

        self.assertEqual(len(pairs), 2)
        self.assertAlmostEqual(pairs[0][0].timestamp, 1.00)
        self.assertAlmostEqual(pairs[0][1].timestamp, 1.02)
        self.assertAlmostEqual(pairs[1][0].timestamp, 1.11)
        self.assertAlmostEqual(pairs[1][1].timestamp, 1.09)

    def test_pos_yaw_alignment_removes_planar_yaw_and_translation_offset(self):
        """pos+yaw 对齐应消除全局平移和 yaw 差异。"""
        yaw = math.pi / 2.0
        q_yaw_90 = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        estimates = [
            PoseSample(1.0, (0.0, 0.0, 0.0), q_yaw_90),
            PoseSample(2.0, (0.0, 1.0, 0.0), q_yaw_90),
            PoseSample(3.0, (0.0, 2.0, 0.0), q_yaw_90),
        ]
        groundtruth = [
            PoseSample(1.0, (10.0, 20.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(2.0, (11.0, 20.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
            PoseSample(3.0, (12.0, 20.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
        ]

        aligned = align_estimates_to_groundtruth(list(zip(estimates, groundtruth)))
        ate = compute_ate(aligned)

        self.assertLess(ate["rmse"], 1e-9)
        self.assertLess(ate["max"], 1e-9)


if __name__ == "__main__":
    unittest.main()
