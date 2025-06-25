#!/usr/bin/env python3
"""
作者：旭指导下的deepseek
ROSbag智能图像采样器 v3.5
新增特性：
1. 彩虹渐变进度条
2. 动态批次状态显示
3. 异常信息分流输出
"""

import os
import argparse
import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg  import Image
from tqdm import tqdm  # 新增进度条库

class BagImageSampler:
    def __init__(self, bag_path, output_dir, topic, prefix):
        self.bag  = rosbag.Bag(bag_path, 'r')
        self.output_dir  = output_dir
        self.topic  = topic
        self.prefix  = prefix
        self.bridge  = CvBridge()
        os.makedirs(output_dir,  exist_ok=True)

    def _golden_sampling(self, total, sample_num):
        if sample_num >= total:
            return range(total)

        phi = (np.sqrt(5)-1)/2
        indexes = set()
        current = 0.0
        while len(indexes) < sample_num:
            idx = int(current * total)
            if idx < total:
                indexes.add(idx)
            current = (current + phi) % 1
        return sorted(indexes)

    def extract(self, sample_num):
        total = self.bag.get_message_count(self.topic)
        indexes = sorted(self._golden_sampling(total, sample_num))

        buffer = []
        pbar = tqdm(total=sample_num,
                   desc="🌈 时空采样进度",
                   bar_format="{l_bar}{bar:50}{r_bar}",
                   colour='#00FF7F')  # 荧光绿渐变

        # print(f"🛰 目标帧数: {len(indexes)}/{total}")

        for idx, (_, msg, _) in enumerate(self.bag.read_messages(topics=[self.topic])):
            if idx not in indexes:
                continue

            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg,  'bgr8')
                buffer.append(  (idx, cv_img) )

                if len(buffer) >= 50 or (buffer and idx == indexes[-1]):
                    # 生成批次范围描述
                    batch_range = f"{buffer[0][0]:06d}-{buffer[-1][0]:06d}"
                    # 更新进度条描述
                    pbar.set_postfix_str(f" 批次 {batch_range}")

                    for num, img in buffer:
                        filename = f"{self.prefix}{num:08d}.png"
                        cv2.imwrite(os.path.join(self.output_dir,  filename), img)
                    pbar.update(len(buffer))   # 批量更新进度
                    buffer.clear()

            except Exception as e:
                tqdm.write(f"⚠️  异常帧@{idx}: {str(e)}")  # 防干扰输出
        # 最终缓冲区保护
        if buffer:
            for num, img in buffer:
                filename = f"{self.prefix}{num:08d}.png"
                cv2.imwrite(os.path.join(self.output_dir, filename), img)
            pbar.update(len(buffer))

        self.bag.close()
        pbar.close()
        print(f"\n✅ 完成！保存路径: {os.path.abspath(self.output_dir)}")

def main():
    parser = argparse.ArgumentParser(prog='bag_sampler',
                                    description='ROSbag智能图像采样器 v3.5（旭）')
    parser.add_argument("bag_file",  help="ROSbag文件路径")
    parser.add_argument("output_dir",  help="输出目录路径")
    parser.add_argument("image_topic",  help="图像话题名称")

    args = parser.parse_args()

    if not os.path.exists(args.bag_file):
        print(f"❌ 文件不存在: {args.bag_file}")
        return

    with rosbag.Bag(args.bag_file,  'r') as bag:
        total = bag.get_message_count(args.image_topic)

    print(f"\n📊 文件分析:")
    print(f"├─ 总帧数: {total}")
    print(f"└─ 存储目录: {os.path.abspath(args.output_dir)}")

    while True:
        user_input = input("\n🖍 采样数量（0=全量｜q=退出）: ").strip().lower()
        if user_input == 'q':
            print("🚪 操作终止")
            return
        if user_input.isdigit():
            sample_num = int(user_input)
            if 0 <= sample_num <= total:
                sample_num = total if sample_num ==0 else sample_num
                break
            print(f"⚠️ 有效范围: 0-{total}")
        else:
            print("⚠️ 请输入数字")

    prefix = input("🖍 文件名前缀（默认img_）: ").strip() or "img_"
    prefix = prefix.translate(str.maketrans('/\\:*?"<> |', '_'*10))  # 过滤非法字符

    print("\n🔥 开始处理...")
    sampler = BagImageSampler(args.bag_file,  args.output_dir,  args.image_topic,  prefix)
    sampler.extract(sample_num)

if __name__ == '__main__':
    main()