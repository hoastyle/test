#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys
import argparse
from collections import defaultdict

def parse_include(line):
    """解析头文件引用"""
    match = re.search(r'#include\s+["<](.*)[">]', line)
    if match:
        return match.group(1)
    return None

def analyze_file(file_path):
    """分析单个文件的依赖"""
    includes = []
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                include = parse_include(line)
                if include:
                    includes.append(include)
    except Exception as e:
        print(f"警告: 无法读取文件 {file_path}: {e}")
    return includes

def map_include_to_module(include_path):
    """映射头文件到对应的模块"""
    module_map = {
        'core/': 'mm_common_core',
        'log/': 'mm_common_log',
        'util/': 'mm_common_util',
        'pcl/': 'mm_common_pcl',
        'transform/': 'mm_common_transform',
        'adaptor/': 'mm_common_adaptor',
        'arith/': 'mm_common_arith',
        'audio/': 'mm_common_audio',
        'collision/': 'mm_common_collision',
        'filter/': 'mm_common_filter',
        'fs/': 'mm_common_fs',  # 添加文件系统库
        'lic/': 'mm_common_lic',
        'lidar/': 'mm_common_lidar',
        'mmcv/': 'mm_common_mmcv',
        'policy/': 'mm_common_policy',
        'projection/': 'mm_common_projection',
        'runtime/': 'mm_common_runtime',
        'web/': 'mm_common_web',
        'obstacle/': 'mm_common_obstacle',
    }

    # 检查MM模块
    for prefix, module in module_map.items():
        if include_path.startswith(prefix):
            return module

    # 检查外部依赖
    if include_path.startswith('boost/filesystem/') or include_path == 'boost/filesystem.hpp':
        return 'mm_common_fs'  # 直接映射到mm_common_fs
    elif include_path.startswith('opencv2/'):
        return 'OpenCV'
    elif include_path.startswith('pcl/'):
        return 'PCL'
    elif include_path.startswith('boost/'):
        return 'Boost'
    elif include_path.startswith('eigen3/') or include_path.startswith('Eigen/'):
        return 'Eigen'
    elif include_path.startswith('yaml-cpp/'):
        return 'YamlCpp'

    return None

def analyze_module(module_dir):
    """分析模块的依赖关系"""
    dependencies = defaultdict(int)

    for root, _, files in os.walk(module_dir):
        for file in files:
            if file.endswith(('.cpp', '.c', '.hpp', '.h')):
                file_path = os.path.join(root, file)
                includes = analyze_file(file_path)

                for include in includes:
                    module = map_include_to_module(include)
                    if module:
                        dependencies[module] += 1

    return dependencies

def detect_boost_filesystem_usage(module_dir):
    """检测是否使用了Boost Filesystem库"""
    for root, _, files in os.walk(module_dir):
        for file in files:
            if file.endswith(('.cpp', '.c', '.hpp', '.h')):
                file_path = os.path.join(root, file)
                try:
                    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                        # 检查是否使用boost::filesystem命名空间或相关函数
                        if 'boost::filesystem' in content or 'boost/filesystem' in content:
                            return True
                except Exception:
                    pass
    return False

def generate_module_config(module_dir, dependencies, overwrite=False):
    """生成模块配置文件"""
    module_config = os.path.join(module_dir, "module.config")

    # 如果文件已存在且不覆盖，则退出
    if os.path.exists(module_config) and not overwrite:
        print(f"配置文件已存在: {module_config}")
        print("使用 --overwrite 参数覆盖现有文件")
        return False

    # 检查是否使用了Boost Filesystem但没有被包含分析检测到
    uses_filesystem = detect_boost_filesystem_usage(module_dir)
    if uses_filesystem and 'mm_common_fs' not in dependencies:
        dependencies['mm_common_fs'] = 1

    with open(module_config, 'w') as f:
        f.write("# 自动生成的模块配置文件\n\n")
        f.write("# 添加头文件路径\n")
        f.write("target_include_directories(${PROJECT_NAME} PRIVATE\n")
        f.write("    ${MM_ROOT}/include\n")
        f.write("    ${MM_ROOT}/src\n")

        # 检查是否使用了darknet
        if any(lib in dependencies for lib in ['yolo', 'darknet']):
            f.write("    ${MM_ROOT}/thirdparty/darknet/yolotrt\n")
            f.write("    ${MM_ROOT}/thirdparty/darknet/yolotrt/extra\n")
            f.write("    ${MM_ROOT}/thirdparty/darknet/src\n")
            f.write("    ${MM_ROOT}/thirdparty/darknet/include\n")

        f.write(")\n\n")

        f.write("# 链接依赖库\n")
        f.write("target_link_libraries(${PROJECT_NAME} PRIVATE\n")

        # 添加核心库
        core_libs = ['mm_common_core', 'mm_common_log', 'mm_common_util']
        for lib in core_libs:
            f.write(f"    {lib}\n")

        # 添加其他MM库
        for module, count in sorted(dependencies.items(), key=lambda x: x[1], reverse=True):
            if module.startswith('mm_common_') and module not in core_libs:
                f.write(f"    {module}\n")

        # 添加外部库
        external_libs = {
            'OpenCV': '${OpenCV_LIBS}',
            'PCL': '${PCL_LIBRARIES}',
            'Boost': '${Boost_LIBRARIES}',
            'Eigen': '${EIGEN3_INCLUDE_DIR}',
            'YamlCpp': '${YAML_CPP_LIBRARIES}'
        }

        for ext_lib, cmake_var in external_libs.items():
            if ext_lib in dependencies:
                f.write(f"    {cmake_var}\n")

        # 添加额外的库检查
        darknet_libs = ['yolo', 'darknet']
        if any(lib in dependencies for lib in darknet_libs):
            f.write("    ${DARKNET_LIB}\n")
            f.write("    ${YOLOTRT_LIB}\n")

        f.write(")\n")

    print(f"生成模块配置: {module_config}")
    return True

def main():
    parser = argparse.ArgumentParser(description='分析模块依赖关系并生成module.config')
    parser.add_argument('module_dir', help='模块目录路径')
    parser.add_argument('--generate', '-g', action='store_true', help='生成module.config文件')
    parser.add_argument('--overwrite', '-o', action='store_true', help='覆盖已存在的module.config文件')
    parser.add_argument('--force-fs', '-f', action='store_true', help='强制添加文件系统库依赖')
    args = parser.parse_args()

    if not os.path.isdir(args.module_dir):
        print(f"错误: {args.module_dir} 不是有效目录")
        return 1

    print(f"分析模块: {args.module_dir}")
    dependencies = analyze_module(args.module_dir)

    # 如果强制添加文件系统依赖
    if args.force_fs and 'mm_common_fs' not in dependencies:
        dependencies['mm_common_fs'] = 1
        print("强制添加 mm_common_fs 依赖")

    if not dependencies:
        print("未找到依赖关系")
        return 0

    print("\n依赖关系统计:")
    print("-" * 50)
    print("模块                    引用次数")
    print("-" * 50)

    for module, count in sorted(dependencies.items(), key=lambda x: x[1], reverse=True):
        print(f"{module:<25} {count}")

    print("\n推荐的库依赖配置:")
    print("```cmake")
    print("target_link_libraries(${PROJECT_NAME} PRIVATE")

    # 首先添加核心库
    core_libs = ['mm_common_core', 'mm_common_log', 'mm_common_util']
    for lib in core_libs:
        print(f"    {lib}")

    # 添加其他库
    for module, count in sorted(dependencies.items(), key=lambda x: x[1], reverse=True):
        if module.startswith('mm_common_') and module not in core_libs:
            print(f"    {module}")

    # 添加外部库
    external_libs = {
        'OpenCV': '${OpenCV_LIBS}',
        'PCL': '${PCL_LIBRARIES}',
        'Boost': '${Boost_LIBRARIES}',
        'Eigen': '${EIGEN3_INCLUDE_DIR}',
        'YamlCpp': '${YAML_CPP_LIBRARIES}'
    }
    for ext_lib, cmake_var in external_libs.items():
        if ext_lib in dependencies:
            print(f"    {cmake_var}")

    print(")")
    print("```")

    # 生成module.config文件
    if args.generate:
        generate_module_config(args.module_dir, dependencies, args.overwrite)
    else:
        print("\n提示: 使用 --generate 参数可自动生成module.config文件")

    return 0

def batch_analyze(root_dir, output_file=None, generate_configs=False, overwrite=False):
    """批量分析多个模块并输出结果"""
    results = {}

    # 遍历src目录下的子目录
    for dir_type in ['detector', 'service', 'sensor', 'plc', 'tos']:
        type_dir = os.path.join(root_dir, 'src', dir_type)
        if not os.path.isdir(type_dir):
            continue

        # 遍历类型目录下的子目录
        for subdir in os.listdir(type_dir):
            module_dir = os.path.join(type_dir, subdir)
            if not os.path.isdir(module_dir):
                continue

            print(f"分析模块: {dir_type}/{subdir}")
            dependencies = analyze_module(module_dir)
            results[f"{dir_type}/{subdir}"] = dependencies

            # 检查是否使用了boost::filesystem但没有被检测到
            if detect_boost_filesystem_usage(module_dir) and 'mm_common_fs' not in dependencies:
                dependencies['mm_common_fs'] = 1
                print(f"  检测到使用Boost Filesystem，添加 mm_common_fs 依赖")

            if generate_configs:
                generate_module_config(module_dir, dependencies, overwrite)

    # 输出结果
    if output_file:
        with open(output_file, 'w') as f:
            for module, deps in results.items():
                f.write(f"模块: {module}\n")
                f.write("-" * 50 + "\n")

                for dep, count in sorted(deps.items(), key=lambda x: x[1], reverse=True):
                    f.write(f"{dep:<25} {count}\n")

                f.write("\n\n")

    return results

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == '--batch':
        # 批量分析模式
        root_dir = '.'
        output_file = 'module_dependencies.txt'
        generate_configs = False
        overwrite = False

        # 解析参数
        for i, arg in enumerate(sys.argv[2:]):
            if arg == '--root':
                root_dir = sys.argv[i+3]
            elif arg == '--output':
                output_file = sys.argv[i+3]
            elif arg == '--generate':
                generate_configs = True
            elif arg == '--overwrite':
                overwrite = True

        batch_analyze(root_dir, output_file, generate_configs, overwrite)
        print(f"批量分析完成，结果已保存到 {output_file}")
        if generate_configs:
            print("已为所有模块生成配置文件")
    else:
        # 单模块分析模式
        sys.exit(main())
