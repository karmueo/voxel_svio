"""验证静态 IMU 初始化使用相机时间锚定窗口，避免回调调度影响初值。"""

import re
import unittest
from pathlib import Path


class StaticInitializerPolicyTest(unittest.TestCase):
    """检查静态初始化接口和调用点显式传递最新相机时间。"""

    def setUp(self):
        """读取初始化相关源码文本。"""
        self.repo_root = Path(__file__).resolve().parents[1]
        self.header_text = (self.repo_root / "include" / "initializer.h").read_text(
            encoding="utf-8"
        )
        self.source_text = (self.repo_root / "src" / "initializer.cpp").read_text(
            encoding="utf-8"
        )
        self.stereo_vio_text = (self.repo_root / "src" / "stereoVio.cpp").read_text(
            encoding="utf-8"
        )

    def test_static_initializer_accepts_camera_anchor_time(self):
        """静态初始化接口应接收 `newest_cam_time`，而不是只看 IMU 队列末尾。"""
        self.assertRegex(
            self.header_text,
            re.compile(
                r"class\s+staticInitializer.*?"
                r"bool\s+initialize\s*\([^;]*double\s+newest_cam_time",
                re.DOTALL,
            ),
        )

    def test_inertial_initializer_passes_camera_anchor_time(self):
        """惯性初始化调度静态初始化时应传递最新相机时间。"""
        self.assertRegex(
            self.source_text,
            re.compile(
                r"static_initializer->initialize\s*\([^;]*wait_for_jerk\s*,\s*newest_cam_time",
                re.DOTALL,
            ),
        )

    def test_static_initializer_converts_camera_anchor_to_imu_time(self):
        """静态初始化窗口终点应由相机时间和 IMU-camera 时间偏移共同确定。"""
        self.assertRegex(
            self.source_text,
            re.compile(
                r"newest_time\s*=\s*newest_cam_time\s*\+\s*"
                r"initializer_options\.calib_camimu_dt"
            ),
        )

    def test_initializer_imu_cache_is_not_pruned_by_newest_imu_time(self):
        """初始化前 IMU 缓存应保留完整数据，后续按相机锚定窗口裁剪。"""
        self.assertRegex(
            self.stereo_vio_text,
            re.compile(r"initializer_ptr->feedImu\s*\(\s*imu_data\s*\)\s*;"),
        )


if __name__ == "__main__":
    unittest.main()
