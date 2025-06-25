import os
import shutil
from sklearn.model_selection import train_test_split

# 配置参数
DATASET_RATIOS = {
    'train': 0.7,  # 训练集比例
    'val': 0.2,  # 验证集比例
    'test': 0.1  # 测试集比例
}
assert abs(sum(DATASET_RATIOS.values())  - 1.0) < 1e-9, "数据集比例异常，当前总和为：%.2f" % sum(DATASET_RATIOS.values())

# 路径配置
SOURCE_PATHS = {
    'images': r'/home/xu/Code/uav_maze/datasets/images',
    'labels': r'/home/xu/Code/uav_maze/datasets/labels'
}

DESTINATION_PATHS = {
    'base': r'/home/xu/PycharmProjects/ultralytics-main/datasets_uav_maze',
    'image_folders': {
        'train': 'images/train',
        'val': 'images/val',
        'test': 'images/test'
    },
    'label_folders': {
        'train': 'labels/train',
        'val': 'labels/val',
        'test': 'labels/test'
    }
}

# 创建输出目录
for split in ['train', 'val', 'test']:
    img_folder = os.path.join(DESTINATION_PATHS['base'], DESTINATION_PATHS['image_folders'][split])
    lbl_folder = os.path.join(DESTINATION_PATHS['base'], DESTINATION_PATHS['label_folders'][split])
    os.makedirs(img_folder, exist_ok=True)
    os.makedirs(lbl_folder, exist_ok=True)

# 获取所有标签文件列表
label_files = [f for f in os.listdir(SOURCE_PATHS['labels']) if f.endswith('.txt')]

# 执行数据集分割
# 第一次分割：分离测试集
train_val_files, test_files = train_test_split(
    label_files,
    test_size=DATASET_RATIOS['test'],
    random_state=42,
    shuffle=True
)

# 第二次分割：分离训练集和验证集
train_files, val_files = train_test_split(
    train_val_files,
    test_size=DATASET_RATIOS['val'] / (1 - DATASET_RATIOS['test']),  # 调整验证集比例
    random_state=42,
    shuffle=True
)


# 文件复制函数
def copy_files(file_list, split_name):
    """复制文件到目标目录"""
    for label_file in file_list:
        # 源文件路径
        img_src = os.path.join(
            SOURCE_PATHS['images'],
            f"{os.path.splitext(label_file)[0]}.png"  # 修改为实际图片格式
        )
        lbl_src = os.path.join(SOURCE_PATHS['labels'], label_file)

        # 目标文件路径
        img_dst = os.path.join(
            DESTINATION_PATHS['base'],
            DESTINATION_PATHS['image_folders'][split_name],
            os.path.basename(img_src)
        )
        lbl_dst = os.path.join(
            DESTINATION_PATHS['base'],
            DESTINATION_PATHS['label_folders'][split_name],
            os.path.basename(lbl_src)
        )

        # 执行复制
        shutil.copy(img_src, img_dst)
        shutil.copy(lbl_src, lbl_dst)


# 执行文件复制
copy_files(train_files, 'train')
copy_files(val_files, 'val')
copy_files(test_files, 'test')

print("数据集分割完成！")
print(f"训练集样本数: {len(train_files)}")
print(f"验证集样本数: {len(val_files)}")
print(f"测试集样本数: {len(test_files)}")