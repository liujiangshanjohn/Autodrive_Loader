#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
采集倾角仪（/tilt_*）与回转编码器（/rotary_encoder_angle）最新角度，追加写入 CSV。

默认「每帧」记录：任一订阅话题每次收到消息即写一行（--period 0）。
若需降采样，设 --period 为秒数（如 1.0 表示每秒写一行当前快照）。

用法（先 source install/setup.bash）:
  python3 $(ros2 pkg prefix cannode)/share/cannode/scripts/tilt_encoder_csv_logger.py
  python3 .../tilt_encoder_csv_logger.py -o /tmp/tilt_enc.csv --period 1.0
"""

from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime, timezone
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cannode.msg import RotaryEncoderAngle, TiltRPY


def _qos_sensor() -> QoSProfile:
    # 与 multi_tilt_imu / danfoss 发布侧常见配置一致
    q = QoSProfile(depth=20)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    return q


class TiltEncoderCsvLogger(Node):
    def __init__(
        self,
        csv_path: str,
        period_sec: float,
        topics: Dict[str, str],
    ) -> None:
        super().__init__('tilt_encoder_csv_logger')
        self._csv_path = csv_path
        self._period_sec = period_sec
        self._topics = topics

        self._tilt: Dict[str, Optional[TiltRPY]] = {
            'boom': None,
            'stick': None,
            'bucket': None,
            'body': None,
        }
        self._enc: Optional[RotaryEncoderAngle] = None

        qos = _qos_sensor()

        self.create_subscription(
            TiltRPY, topics['boom'], lambda m: self._set_tilt('boom', m), qos
        )
        self.create_subscription(
            TiltRPY, topics['stick'], lambda m: self._set_tilt('stick', m), qos
        )
        self.create_subscription(
            TiltRPY, topics['bucket'], lambda m: self._set_tilt('bucket', m), qos
        )
        self.create_subscription(
            TiltRPY, topics['body'], lambda m: self._set_tilt('body', m), qos
        )
        self.create_subscription(
            RotaryEncoderAngle,
            topics['encoder'],
            self._on_encoder,
            qos,
        )

        self._file = open(self._csv_path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._header_row())
        self._file.flush()

        if period_sec > 0.0:
            self.create_timer(float(period_sec), self._write_row)
            mode = '定时 %.3f s/行' % period_sec
        else:
            mode = '每帧（任一话题有消息即写一行）'
        self.get_logger().info(
            '写入 CSV: %s ，%s ，话题 boom=%s stick=%s bucket=%s body=%s enc=%s'
            % (
                csv_path,
                mode,
                topics['boom'],
                topics['stick'],
                topics['bucket'],
                topics['body'],
                topics['encoder'],
            )
        )

    def close_csv(self) -> None:
        """关闭 CSV（在 shutdown 前调用）"""
        try:
            if getattr(self, '_file', None) and not self._file.closed:
                self._file.flush()
                self._file.close()
        except Exception:
            pass

    def _header_row(self) -> List[str]:
        cols = [
            'utc_iso8601',
            'unix_time_sec',
        ]
        for name in ('boom', 'stick', 'bucket', 'body'):
            p = f'tilt_{name}'
            cols.extend([f'{p}_roll_deg', f'{p}_pitch_deg', f'{p}_yaw_deg'])
        cols.extend(
            [
                'rotary_encoder_angle1_deg',
                'rotary_encoder_angle2_deg',
                'rotary_encoder_error_code',
            ]
        )
        return cols

    def _set_tilt(self, key: str, msg: TiltRPY) -> None:
        self._tilt[key] = msg
        if self._period_sec <= 0.0:
            self._write_row()

    def _on_encoder(self, msg: RotaryEncoderAngle) -> None:
        self._enc = msg
        if self._period_sec <= 0.0:
            self._write_row()

    def _fmt_tilt(self, key: str) -> List[object]:
        m = self._tilt.get(key)
        if m is None:
            return ['', '', '']
        return [m.roll, m.pitch, m.yaw]

    def _fmt_enc(self) -> List[object]:
        m = self._enc
        if m is None:
            return ['', '', '']
        return [m.angle1, m.angle2, m.error_code]

    def _write_row(self) -> None:
        now = datetime.now(timezone.utc)
        iso = now.isoformat()
        unix_sec = now.timestamp()
        row: List[object] = [iso, f'{unix_sec:.6f}']
        for k in ('boom', 'stick', 'bucket', 'body'):
            row.extend(self._fmt_tilt(k))
        row.extend(self._fmt_enc())
        self._writer.writerow(row)
        self._file.flush()


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description='记录倾角仪与编码器角度到 CSV（ROS 2）；默认每帧一条'
    )
    parser.add_argument(
        '-o',
        '--output',
        default='tilt_encoder_angles.csv',
        help='输出 CSV 路径（默认当前目录 tilt_encoder_angles.csv）',
    )
    parser.add_argument(
        '--period',
        type=float,
        default=0.0,
        help='0=每帧记录（任一话题有消息即写一行）；>0 为定时采样周期（秒）',
    )
    parser.add_argument('--tilt-boom', default='/tilt_boom')
    parser.add_argument('--tilt-stick', default='/tilt_stick')
    parser.add_argument('--tilt-bucket', default='/tilt_bucket')
    parser.add_argument('--tilt-body', default='/tilt_body')
    parser.add_argument('--rotary-encoder', default='/rotary_encoder_angle')
    args = parser.parse_args(argv)

    if args.period < 0:
        print('period 必须 >= 0（0 表示每帧）', file=sys.stderr)
        return 1

    topics = {
        'boom': args.tilt_boom,
        'stick': args.tilt_stick,
        'bucket': args.tilt_bucket,
        'body': args.tilt_body,
        'encoder': args.rotary_encoder,
    }

    rclpy.init()
    node = TiltEncoderCsvLogger(args.output, args.period, topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
