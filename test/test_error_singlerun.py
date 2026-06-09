"""测试 OpenVINS 风格单次轨迹误差评估入口。"""

import contextlib
import io
import math
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from error_singlerun import (  # noqa: E402
    PoseSample,
    align_posyaw_umeyama,
    compute_ate,
    compute_nees,
    compute_rpe,
    format_report,
    match_by_timestamp,
    quat_multiply,
    read_pose_file,
    yaw_quaternion as evaluator_yaw_quaternion,
)


def yaw_quaternion(yaw: float):
    """构造 OpenVINS/JPL 绕 Z 轴旋转的四元数。"""
    return (0.0, 0.0, -math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class ErrorSingleRunTest(unittest.TestCase):
    """验证单次轨迹误差评估的核心数值定义。"""

    def test_yaw_quaternion_uses_openvins_jpl_sign(self):
        """yaw 四元数应复刻 OpenVINS/JPL 的 z 分量符号。"""
        quaternion = evaluator_yaw_quaternion(math.radians(90.0))

        self.assertAlmostEqual(quaternion[0], 0.0)
        self.assertAlmostEqual(quaternion[1], 0.0)
        self.assertAlmostEqual(quaternion[2], -math.sqrt(0.5))
        self.assertAlmostEqual(quaternion[3], math.sqrt(0.5))

    def test_quat_multiply_matches_openvins_jpl_yaw_composition(self):
        """JPL 四元数乘法应与 OpenVINS yaw 组合一致。"""
        first_yaw = yaw_quaternion(math.radians(30.0))
        second_yaw = yaw_quaternion(math.radians(45.0))

        composed = quat_multiply(first_yaw, second_yaw)
        expected = yaw_quaternion(math.radians(75.0))

        for actual_value, expected_value in zip(composed, expected):
            self.assertAlmostEqual(actual_value, expected_value)

    def test_read_pose_file_accepts_8_and_20_column_openvins_format(self):
        """轨迹读取应支持无协方差和 OpenVINS 20 列协方差格式。"""
        with tempfile.TemporaryDirectory() as tmp_dir:
            trajectory_path = Path(tmp_dir) / "trajectory.txt"
            trajectory_path.write_text(
                "\n".join(
                    [
                        "# timestamp tx ty tz qx qy qz qw ...",
                        "1.0 1 2 3 0 0 0 1",
                        "2.0 4 5 6 0 0 0.70710678 0.70710678 "
                        "1 0.1 0.2 2 0.3 3 4 0.4 0.5 5 0.6 6",
                    ]
                ),
                encoding="utf-8",
            )

            samples = read_pose_file(trajectory_path)

        self.assertEqual(len(samples), 2)
        self.assertIsNone(samples[0].covariance_orientation)
        self.assertIsNone(samples[0].covariance_position)
        self.assertEqual(samples[1].position, (4.0, 5.0, 6.0))
        self.assertAlmostEqual(samples[1].covariance_orientation[0][1], 0.1)
        self.assertAlmostEqual(samples[1].covariance_orientation[1][0], 0.1)
        self.assertAlmostEqual(samples[1].covariance_position[2][2], 6.0)

    def test_match_by_timestamp_uses_002_second_threshold(self):
        """时间戳关联默认阈值应为 0.02 秒。"""
        estimates = [
            PoseSample(1.000, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
            PoseSample(2.000, (1.0, 0.0, 0.0), yaw_quaternion(0.0)),
            PoseSample(3.000, (2.0, 0.0, 0.0), yaw_quaternion(0.0)),
        ]
        groundtruth = [
            PoseSample(1.019, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
            PoseSample(2.021, (1.0, 0.0, 0.0), yaw_quaternion(0.0)),
            PoseSample(2.990, (2.0, 0.0, 0.0), yaw_quaternion(0.0)),
        ]

        pairs = match_by_timestamp(estimates, groundtruth)

        self.assertEqual([pair[0].timestamp for pair in pairs], [1.0, 3.0])

    def test_posyaw_umeyama_alignment_removes_global_yaw_and_translation(self):
        """posyaw 对齐应通过整段位置估计 yaw 和平移。"""
        yaw = math.radians(90.0)
        estimates = [
            PoseSample(1.0, (0.0, 0.0, 0.0), yaw_quaternion(yaw)),
            PoseSample(2.0, (1.0, 0.0, 0.0), yaw_quaternion(yaw)),
            PoseSample(3.0, (2.0, 1.0, 0.0), yaw_quaternion(yaw)),
        ]
        groundtruth = [
            PoseSample(1.0, (10.0, 20.0, 0.5), yaw_quaternion(0.0)),
            PoseSample(2.0, (10.0, 21.0, 0.5), yaw_quaternion(0.0)),
            PoseSample(3.0, (9.0, 22.0, 0.5), yaw_quaternion(0.0)),
        ]

        aligned = align_posyaw_umeyama(list(zip(estimates, groundtruth)))
        ate = compute_ate(aligned)

        self.assertLess(ate["pos"].rmse, 1e-9)
        self.assertLess(ate["ori"].rmse, 1e-9)

    def test_posyaw_alignment_uses_jpl_orientation_update(self):
        """ATE 姿态对齐不应因 JPL/Hamilton 约定混用残留全局 yaw 误差。"""
        alignment_yaw = math.radians(14.0)
        body_yaws = [math.radians(value) for value in (5.0, 18.0, 31.0, 46.0)]
        estimate_positions = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.4, 0.0),
            (3.0, 1.0, 0.0),
        ]
        estimates = [
            PoseSample(float(index), position, yaw_quaternion(body_yaws[index]))
            for index, position in enumerate(estimate_positions)
        ]
        groundtruth = [
            PoseSample(
                float(index),
                (
                    3.0 + math.cos(alignment_yaw) * position[0] - math.sin(alignment_yaw) * position[1],
                    -2.0 + math.sin(alignment_yaw) * position[0] + math.cos(alignment_yaw) * position[1],
                    position[2] + 0.25,
                ),
                yaw_quaternion(body_yaws[index] - alignment_yaw),
            )
            for index, position in enumerate(estimate_positions)
        ]

        aligned = align_posyaw_umeyama(list(zip(estimates, groundtruth)))
        ate = compute_ate(aligned)

        self.assertLess(ate["pos"].rmse, 1e-9)
        self.assertLess(ate["ori"].rmse, 1e-9)

    def test_compute_ate_reports_orientation_degrees_and_position_meters(self):
        """ATE 应同时统计姿态角度误差和三维位置误差。"""
        aligned_pairs = [
            (
                PoseSample(1.0, (1.0, 0.0, 0.0), yaw_quaternion(math.radians(10.0))),
                PoseSample(1.0, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
            ),
            (
                PoseSample(2.0, (0.0, 2.0, 0.0), yaw_quaternion(math.radians(20.0))),
                PoseSample(2.0, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
            ),
        ]

        ate = compute_ate(aligned_pairs)

        self.assertAlmostEqual(ate["ori"].rmse, math.sqrt((10.0**2 + 20.0**2) / 2.0))
        self.assertAlmostEqual(ate["pos"].rmse, math.sqrt((1.0**2 + 2.0**2) / 2.0))
        self.assertAlmostEqual(ate["ori"].min, 10.0)
        self.assertAlmostEqual(ate["pos"].max, 2.0)

    def test_compute_rpe_uses_fixed_distance_segments_and_sample_counts(self):
        """RPE 应按 OpenVINS 默认距离片段统计中位误差和样本数。"""
        aligned_pairs = []
        for index in range(41):
            truth = PoseSample(float(index), (float(index), 0.0, 0.0), yaw_quaternion(0.0))
            estimate = PoseSample(
                float(index),
                (float(index) * 1.1, 0.0, 0.0),
                yaw_quaternion(0.0),
            )
            aligned_pairs.append((estimate, truth))

        rpe = compute_rpe(aligned_pairs)

        self.assertEqual(rpe[8.0]["pos"].count, 33)
        self.assertEqual(rpe[16.0]["pos"].count, 25)
        self.assertEqual(rpe[24.0]["pos"].count, 17)
        self.assertEqual(rpe[32.0]["pos"].count, 9)
        self.assertEqual(rpe[40.0]["pos"].count, 1)
        self.assertAlmostEqual(rpe[8.0]["pos"].median, 0.8)

        orientation_pairs = []
        for index in range(41):
            truth = PoseSample(float(index), (float(index), 0.0, 0.0), yaw_quaternion(0.0))
            estimate = PoseSample(
                float(index),
                (float(index), 0.0, 0.0),
                yaw_quaternion(math.radians(float(index))),
            )
            orientation_pairs.append((estimate, truth))

        orientation_rpe = compute_rpe(orientation_pairs)

        self.assertAlmostEqual(orientation_rpe[16.0]["ori"].median, 16.0)

    def test_compute_rpe_rotates_openvins_se3_error_into_world_frame(self):
        """RPE 平移误差应复刻 OpenVINS 的 SE(3) 矩阵流程。"""
        aligned_pairs = []
        for index in range(10):
            yaw = math.radians(index * 10.0)
            truth_position = (
                math.cos(yaw) * float(index),
                math.sin(yaw) * float(index),
                0.0,
            )
            truth = PoseSample(float(index), truth_position, yaw_quaternion(yaw))
            estimate = PoseSample(
                float(index),
                (truth_position[0] + 0.05 * float(index), truth_position[1], 0.0),
                yaw_quaternion(yaw),
            )
            aligned_pairs.append((estimate, truth))

        rpe = compute_rpe(aligned_pairs, segments=(8.0,))

        self.assertEqual(rpe[8.0]["pos"].count, 3)
        self.assertAlmostEqual(rpe[8.0]["pos"].median, 0.3)
        self.assertLess(rpe[8.0]["pos"].median, 0.5)

    def test_compute_nees_warns_and_returns_zero_without_covariance(self):
        """没有协方差时 NEES 应输出 OpenVINS 风格警告并保持 0 统计。"""
        aligned_pairs = [
            (
                PoseSample(1.0, (1.0, 0.0, 0.0), yaw_quaternion(0.0)),
                PoseSample(1.0, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
            )
        ]

        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            nees = compute_nees(aligned_pairs)

        self.assertIn(
            "[TRAJ]: Normalized Estimation Error Squared called but trajectory does not have any covariances",
            stdout.getvalue(),
        )
        self.assertEqual(nees["ori"].mean, 0.0)
        self.assertEqual(nees["pos"].mean, 0.0)

    def test_format_report_matches_openvins_section_layout(self):
        """报告格式应包含 ATE、RPE、NEES 三段 OpenVINS 风格输出。"""
        zero_stats = compute_ate(
            [
                (
                    PoseSample(1.0, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
                    PoseSample(1.0, (0.0, 0.0, 0.0), yaw_quaternion(0.0)),
                )
            ]
        )
        nees = {"ori": zero_stats["ori"], "pos": zero_stats["pos"]}
        report = format_report(zero_stats, {8.0: zero_stats}, nees)

        self.assertIn("Setting printing level to: INFO", report)
        self.assertIn("Absolute Trajectory Error", report)
        self.assertIn("min_ori  = 0.000 | min_pos  = 0.000", report)
        self.assertIn("std_ori  = 0.000 | std_pos  = 0.000", report)
        self.assertNotIn("min _ori", report)
        self.assertNotIn("std _ori", report)
        self.assertIn("seg 8 - median_ori = 0.000 | median_pos = 0.000 (1 samples)", report)
        self.assertIn("Normalized Estimation Error Squared", report)


if __name__ == "__main__":
    unittest.main()
