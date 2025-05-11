import sys
import re
import csv
import argparse
from pathlib import Path

import csv
from pathlib import Path

def read_csv_file(file_path: Path, skip_header=True) -> list:
    """读取CSV文件并返回二维列表"""
    data = []
    encodings = ['utf-8-sig', 'gbk', 'gb2312', 'utf-16']  # 常见中文编码

    for encoding in encodings:
        try:
            with open(file_path, 'r', newline='', encoding=encoding) as f:
                reader = csv.reader(f)
                for row in reader:
                    if skip_header and reader.line_num == 1:
                        continue
                    # 统一转换为UTF-8编码
                    data.append([cell.encode('utf-8', 'ignore').decode('utf-8').strip()
                                for cell in row])
                break  # 成功读取则退出循环
        except UnicodeDecodeError:
            continue
        except Exception as e:
            print(f"CSV读取失败: {str(e)}")
            return []

    return data

def write_clean_csv(input_path: Path, output_path: Path) -> None:
    # 读取原始数据（包含标题行）
    data = read_csv_file(input_path, skip_header=False)  # 修改skip_header参数

    try:
        with open(output_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f,
                              delimiter=',',
                              quoting=csv.QUOTE_NONE,
                              escapechar=' ')

            # 保留原始标题行
            if data:
                writer.writerow(data[0])  # 直接写入首行

            # 仅处理后续数据行
            for row in data[1:]:
                processed_row = [
                    re.sub(r'\s+', ',', cell)
                        .replace('"', '')
                        .replace(',', ', ')  # 新增：将逗号替换为下划线
                        .replace('mm', '')
                        .replace('\\', '')
                        .strip()
                    for cell in row
                ]
                writer.writerow(processed_row)

        print(f"生成清洗后的文件：{output_path}")
    except Exception as e:
        sys.exit(f"文件写入失败：{e}")

# 修改main函数处理逻辑
def main():
    print("CSV文件清洗工具")
    parser = argparse.ArgumentParser(description='CSV文件清洗工具')
    parser.add_argument('input', type=Path, help='输入CSV文件路径')

    args = parser.parse_args()

    if not args.input.exists():
        sys.exit(f"输入文件不存在：{args.input}")

    output_path = args.input.with_name(f"{args.input.stem}_cdpnp{args.input.suffix}")
    write_clean_csv(args.input, output_path)

if __name__ == "__main__":
    main()