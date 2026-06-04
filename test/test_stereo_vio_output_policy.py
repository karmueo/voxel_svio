"""测试 Voxel-SVIO 轨迹输出文件的覆盖与追加策略。"""

import re
import unittest
from pathlib import Path


class StereoVioOutputPolicyTest(unittest.TestCase):
    """验证 `pose.txt` 在节点启动和帧写入时使用正确打开模式。"""

    def setUp(self):
        """读取 `stereoVio.cpp` 源码供静态策略检查使用。"""
        self.source_path = Path(__file__).resolve().parents[1] / "src" / "stereoVio.cpp"
        self.source_text = self.source_path.read_text(encoding="utf-8")

    def test_pose_output_is_truncated_once_when_node_starts(self):
        """节点启动时应清空旧 `pose.txt`，避免多次运行结果混入。"""
        self.assertRegex(
            self.source_text,
            re.compile(
                r"resetPoseOutputFile\s*\([^)]*\).*?"
                r"std::ofstream\s+\w+\s*\([^;]*?/pose\.txt[^;]*std::ios::trunc",
                re.DOTALL,
            ),
        )

    def test_pose_output_appends_samples_during_runtime(self):
        """运行中每帧位姿仍应追加写入，保留本次运行的完整轨迹。"""
        self.assertRegex(
            self.source_text,
            r"std::ofstream\s+\w+\s*\([^;]*?/pose\.txt[^;]*std::ios::app",
        )


if __name__ == "__main__":
    unittest.main()
