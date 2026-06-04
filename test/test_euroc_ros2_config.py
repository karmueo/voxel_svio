"""验证 ROS2 EuRoC 配置包含难序列所需的鲁棒视觉更新参数。"""

import re
import unittest
from pathlib import Path


class EurocRos2ConfigTest(unittest.TestCase):
    """检查 EuRoC ROS2 YAML 中的关键鲁棒更新参数。"""

    def setUp(self):
        """读取 EuRoC ROS2 配置文本。"""
        self.config_path = Path(__file__).resolve().parents[1] / "config" / "euroc_ros2.yaml"
        self.config_text = self.config_path.read_text(encoding="utf-8")

    def _read_scalar(self, key):
        """读取简单 `key: value` 形式的 YAML 标量。

        Args:
            key: 需要读取的参数名。

        Returns:
            参数文本值。
        """
        match = re.search(rf"^\s+{re.escape(key)}:\s+([^\n#]+)", self.config_text, re.MULTILINE)
        self.assertIsNotNone(match, f"缺少配置项 {key}")
        return match.group(1).strip()

    def test_visual_updates_use_robust_gating_for_difficult_euroc_sequences(self):
        """V1_03_difficult 应使用 Huber 和较宽的卡方门限避免视觉更新过早失效。"""
        self.assertEqual(self._read_scalar("use_huber"), "true")
        self.assertGreaterEqual(float(self._read_scalar("up_msckf_chi2_multipler")), 5.0)
        self.assertGreaterEqual(float(self._read_scalar("up_slam_chi2_multipler")), 5.0)


if __name__ == "__main__":
    unittest.main()
