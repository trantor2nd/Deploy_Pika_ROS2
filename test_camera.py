#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试 Pika 鱼眼相机和 RealSense 相机，实时预览并保存截图。"""

import cv2
from pika.gripper import Gripper
from config import (
    GRIPPER_PORT, FISHEYE_INDEX, REALSENSE_SN,
    FISHEYE_WIDTH, FISHEYE_HEIGHT, REALSENSE_FPS,
)


def main():
    print("正在连接 Pika Gripper 设备 ...")
    gripper = Gripper(GRIPPER_PORT)
    if not gripper.connect():
        print("[ERROR] 连接 Pika Gripper 失败，请检查设备连接和串口路径")
        return

    print("成功连接到 Pika Gripper 设备")
    gripper.set_camera_param(FISHEYE_WIDTH, FISHEYE_HEIGHT, REALSENSE_FPS)
    gripper.set_fisheye_camera_index(FISHEYE_INDEX)
    gripper.set_realsense_serial_number(REALSENSE_SN)

    fisheye = gripper.get_fisheye_camera()
    realsense = gripper.get_realsense_camera()

    if not fisheye and not realsense:
        print("[ERROR] 未检测到任何相机")
        return

    print("按 q 退出, 按 s 保存当前帧")
    while True:
        if fisheye:
            ok, frame = fisheye.get_frame()
            if ok and frame is not None:
                cv2.imshow("Fisheye Camera", frame)

        if realsense:
            ok, color = realsense.get_color_frame()
            if ok and color is not None:
                cv2.imshow("RealSense Color", color)

            ok, depth = realsense.get_depth_frame()
            if ok and depth is not None:
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
                )
                cv2.imshow("RealSense Depth", depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            if fisheye:
                ok, frame = fisheye.get_frame()
                if ok and frame is not None:
                    cv2.imwrite("test_fisheye.jpg", frame)
                    print("已保存 test_fisheye.jpg")
            if realsense:
                ok, color = realsense.get_color_frame()
                if ok and color is not None:
                    cv2.imwrite("test_realsense.jpg", color)
                    print("已保存 test_realsense.jpg")

    cv2.destroyAllWindows()
    print("测试结束")


if __name__ == "__main__":
    main()
