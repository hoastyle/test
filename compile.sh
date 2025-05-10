#!/bin/bash

# compile.sh - 统一编译脚本
# 用法: ./compile.sh [--full|--lite] [--modules|--perception-only|--vision-only] [--config config.yaml] [additional args...]

# 颜色定义，用于改善输出可读性
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 全局变量，用于模块选择
BUILD_MODULES=""
SELECTED_MODULES=""
MODULE_SPECIFIC_BUILD=false
BUILD_MODE="full"  # 默认值
BUILD_MODE_SPECIFIED=false  # 标记是否明确指定了编译模式
CONFIG_FILE=""  # 配置文件路径

# 依赖检查函数
check_dependencies() {
    local exit_on_error=$1
    local has_error=0

    # 检查Python3是否存在
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}错误: Python3 is required but not installed.${NC}"
        has_error=1
    fi

    # 检查ccache是否存在
    if ! command -v ccache &> /dev/null; then
        echo -e "${RED}错误: ccache is required but not installed.${NC}"
        echo -e "${YELLOW}请安装ccache: sudo apt-get install ccache${NC}"
        has_error=1
    fi

    # 检查ninja是否存在
    if ! command -v ninja &> /dev/null; then
        echo -e "${RED}错误: ninja is required but not installed.${NC}"
        echo -e "${YELLOW}请安装ninja: sudo apt-get install ninja-build${NC}"
        has_error=1
    fi

    # 检查关键脚本存在性
    if [ ! -f "scripts/delRedundancyLib.py" ]; then
        echo -e "${RED}错误: delRedundancyLib.py not found${NC}"
        has_error=1
    fi

    if [ ! -f "compileFull.sh" ]; then
        echo -e "${RED}错误: compileFull.sh not found${NC}"
        has_error=1
    fi

    if [ $has_error -eq 1 ] && [ "$exit_on_error" = true ]; then
        echo -e "${RED}构建环境不完整，请确保所有依赖已安装。${NC}"
        exit 1
    fi

    return $has_error
}

# 解析配置文件
parse_config_file() {
    local config_file=$1

    if [ ! -f "$config_file" ]; then
        echo -e "${RED}错误: 配置文件 $config_file 不存在${NC}"
        return 1
    fi

    # 使用Python解析YAML配置文件
    local config_values=$(python3 - <<EOF
import yaml
import sys
import os

try:
    with open("$config_file", 'r') as file:
        config = yaml.safe_load(file)

    # 默认值
    result = {
        'build_mode': 'full',
        'environment': 'develop',
        'build_type': 'Debug',
        'dev_mode': 'common',
        'modules_include': '',
        'modules_exclude': '',
        'perception_only': 'false',
        'vision_only': 'false',
        'threads': '0',
        'verbose': 'false',
        'cmake_args': ''
    }

    # 读取配置
    if config:
        result['build_mode'] = config.get('build_mode', 'full')
        result['environment'] = config.get('environment', 'develop')
        result['build_type'] = config.get('build_type', 'Debug')
        result['dev_mode'] = config.get('dev_mode', 'common')

        # 处理模块配置
        modules = config.get('modules', {})
        if 'include' in modules:
            result['modules_include'] = ','.join(modules['include'])
        if 'exclude' in modules:
            result['modules_exclude'] = ','.join(modules['exclude'])
        if modules.get('perception_only', False):
            result['perception_only'] = 'true'
        if modules.get('vision_only', False):
            result['vision_only'] = 'true'

        # 处理其他选项
        options = config.get('options', {})
        result['threads'] = str(options.get('threads', 0))
        result['verbose'] = str(options.get('verbose', False)).lower()
        if 'cmake_args' in options:
            result['cmake_args'] = ' '.join(options['cmake_args'])

    # 输出为shell可以source的格式
    for key, value in result.items():
        print(f"config_{key}='{value}'")

except Exception as e:
    print(f"# 错误: 解析配置文件失败 - {e}", file=sys.stderr)
    sys.exit(1)
EOF
    )

    if [ $? -ne 0 ]; then
        return 1
    fi

    # 导入配置值到环境变量
    eval "$config_values"

    return 0
}

# 应用配置文件设置
apply_config() {
    # 如果没有通过命令行明确指定编译模式，则使用配置文件中的值
    if [ "$BUILD_MODE_SPECIFIED" = false ] && [ -n "$config_build_mode" ]; then
        BUILD_MODE="$config_build_mode"
        echo -e "${BLUE}使用配置文件指定的编译模式: $BUILD_MODE${NC}"
    fi

    # 处理模块配置
    if [ -n "$config_modules_include" ]; then
        SELECTED_MODULES="$config_modules_include"
        MODULE_SPECIFIC_BUILD=true
        echo -e "${BLUE}使用配置文件指定的包含模块: $SELECTED_MODULES${NC}"
    fi

    if [ -n "$config_modules_exclude" ]; then
        export EXCLUDE_MODULES="$config_modules_exclude"
        MODULE_SPECIFIC_BUILD=true
        echo -e "${BLUE}使用配置文件指定的排除模块: $config_modules_exclude${NC}"
    fi

    if [ "$config_perception_only" = "true" ]; then
        BUILD_MODULES="perception"
        MODULE_SPECIFIC_BUILD=true
        echo -e "${BLUE}使用配置文件指定: 只编译perception模块${NC}"
    fi

    if [ "$config_vision_only" = "true" ]; then
        BUILD_MODULES="vision"
        MODULE_SPECIFIC_BUILD=true
        echo -e "${BLUE}使用配置文件指定: 只编译vision模块${NC}"
    fi

    # 设置其他环境变量
    if [ -n "$config_threads" ] && [ "$config_threads" != "0" ]; then
        export COMPILE_THREADS="$config_threads"
        echo -e "${BLUE}使用配置文件指定的编译线程数: $config_threads${NC}"
    fi

    if [ "$config_verbose" = "true" ]; then
        export VERBOSE_BUILD="1"
        echo -e "${BLUE}启用详细输出模式${NC}"
    fi

    if [ -n "$config_cmake_args" ]; then
        export EXTRA_CMAKE_ARGS="$config_cmake_args"
        echo -e "${BLUE}使用配置文件指定的额外CMake参数: $config_cmake_args${NC}"
    fi
}

# 通用编译函数
compile_with_flags() {
    local tests_needed=$1
    local tools_needed=$2
    shift 2  # 移除前两个参数，保留其余参数

    echo -e "${GREEN}编译配置：${NC}"
    echo -e "  - Tests: $([ $tests_needed -eq 1 ] && echo '是' || echo '否')"
    echo -e "  - Tools: $([ $tools_needed -eq 1 ] && echo '是' || echo '否')"
    echo ""

    # 设置编译环境变量
    testsNeeded=$tests_needed
    toolsNeeded=$tools_needed

    # 传递模块选择参数
    export BUILD_MODULES
    export SELECTED_MODULES

    # 执行编译
    source compileFull.sh "$@"

    # 清理冗余库
    if [ ${debugNeeded} -eq 1 ]; then
        python3 scripts/delRedundancyLib.py ${project} ${gantry} Debug
    fi

    if [ ${releaseNeeded} -eq 1 ]; then
        python3 scripts/delRedundancyLib.py ${project} ${gantry} Release
    fi
}

# 显示使用说明
show_usage() {
    echo "用法: $0 [编译模式选项] [模块选择选项] [配置文件选项] [develop|product] [Debug|Release] [common|sdk]"
    echo ""
    echo "编译模式选项:"
    echo "  --full              完整编译（包含tests和tools）[默认]"
    echo "  --lite              轻量编译（不包含tests和tools）"
    echo ""
    echo "模块选择选项:"
    echo "  --modules MODULE_LIST    指定编译的模块（逗号分隔）"
    echo "                          如: --modules perception/lidarETDetector,vision/yoloObjectDetector"
    echo "  --perception-only       只编译 perception 模块"
    echo "  --vision-only           只编译 vision 模块"
    echo "  --exclude MODULE_LIST   排除特定模块（逗号分隔）"
    echo ""
    echo "配置文件选项:"
    echo "  --config CONFIG_FILE    使用指定的YAML配置文件"
    echo "                          配置文件选项可被命令行选项覆盖"
    echo ""
    echo "构建形式选项:"
    echo "  develop                开发环境 [默认]"
    echo "  product                生产环境"
    echo ""
    echo "构建类型选项:"
    echo "  Debug                  调试构建 [默认]"
    echo "  Release                发布构建"
    echo "  Debug|Release          同时构建调试和发布版本"
    echo ""
    echo "开发模式选项:"
    echo "  common                 通用模式（编译应用程序）[默认]"
    echo "  sdk                    SDK模式（生成SDK包）"
    echo ""
    echo "注意："
    echo "  - 参数顺序：[编译模式选项] [模块选择选项] [配置文件选项] [构建形式] [构建类型] [开发模式]"
    echo "  - 所有参数都是可选的，未指定的参数将使用默认值"
    echo "  - 命令行选项优先级高于配置文件选项"
    echo "  - 当使用模块选择选项时，默认不编译 tests 和 tools"
    echo "  - 若需要在模块选择时编译 tests 和 tools，请明确使用 --full 选项"
    echo ""
    echo "配置文件示例（build.config）:"
    echo "  build_mode: full"
    echo "  modules:"
    echo "    include:"
    echo "      - perception/lidarETDetector"
    echo "      - vision/yoloObjectDetector"
    echo "  environment: develop"
    echo "  build_type: Debug"
    echo "  dev_mode: common"
    echo ""
    echo "示例:"
    echo "  $0                                                    # 默认：--full develop Debug common"
    echo "  $0 --config build.config                              # 使用配置文件"
    echo "  $0 --modules perception/lidarETDetector               # 只编译指定模块，使用默认参数"
    echo "  $0 --config build.config --full product Release       # 配置文件 + 命令行覆盖"
    echo "  $0 --perception-only --lite develop Release           # 只编译perception，轻量模式，发布构建"
    echo "  $0 --exclude perception/mapTruckLocator product Debug # 排除特定模块，生产环境，调试构建"
    echo "  $0 --full product Release sdk                         # 完整编译，生产环境，发布构建，SDK模式"
    echo "  $0 --modules perception/lidarETDetector --full        # 编译指定模块并包含 tests 和 tools"
    echo ""
    echo "模式说明："
    echo "  - develop: 开发环境，使用符号链接，支持增量开发"
    echo "  - product: 生产环境，复制文件，适合部署"
    echo "  - common: 编译完整应用程序"
    echo "  - sdk: 生成SDK包，用于外部集成"
    echo ""
}

# 主程序开始
echo -e "${GREEN}=== 开始编译流程 ===${NC}"

# 检查依赖
check_dependencies true

# 创建临时参数数组
args=()
remaining_args=()

# 解析所有参数
while [ $# -gt 0 ]; do
    case "$1" in
        --config)
            if [ -z "$2" ]; then
                echo -e "${RED}错误: --config 需要指定配置文件路径${NC}"
                show_usage
                exit 1
            fi
            CONFIG_FILE="$2"
            echo -e "${BLUE}使用配置文件: $CONFIG_FILE${NC}"
            shift 2
            ;;
        --modules)
            if [ -z "$2" ]; then
                echo -e "${RED}错误: --modules 需要指定模块列表${NC}"
                show_usage
                exit 1
            fi
            SELECTED_MODULES="$2"
            MODULE_SPECIFIC_BUILD=true
            echo -e "${BLUE}将编译指定模块: $SELECTED_MODULES${NC}"
            shift 2
            ;;
        --perception-only)
            BUILD_MODULES="perception"
            MODULE_SPECIFIC_BUILD=true
            echo -e "${BLUE}将只编译 perception 模块${NC}"
            shift
            ;;
        --vision-only)
            BUILD_MODULES="vision"
            MODULE_SPECIFIC_BUILD=true
            echo -e "${BLUE}将只编译 vision 模块${NC}"
            shift
            ;;
        --exclude)
            if [ -z "$2" ]; then
                echo -e "${RED}错误: --exclude 需要指定模块列表${NC}"
                show_usage
                exit 1
            fi
            export EXCLUDE_MODULES="$2"
            MODULE_SPECIFIC_BUILD=true
            echo -e "${BLUE}将排除模块: $2${NC}"
            shift 2
            ;;
        --full)
            BUILD_MODE="full"
            BUILD_MODE_SPECIFIED=true
            shift
            ;;
        --lite)
            BUILD_MODE="lite"
            BUILD_MODE_SPECIFIED=true
            shift
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            # 保存所有其他参数，传递给 compileFull.sh
            remaining_args+=("$1")
            shift
            ;;
    esac
done

# 如果指定了配置文件，解析并应用配置
if [ -n "$CONFIG_FILE" ]; then
    echo -e "${GREEN}正在解析配置文件...${NC}"

    # 检查pyyaml是否安装
    if ! python3 -c "import yaml" 2>/dev/null; then
        echo -e "${YELLOW}警告: pyyaml未安装，请安装: pip3 install pyyaml${NC}"
        echo -e "${YELLOW}或者使用系统包管理器安装，例如: sudo apt-get install python3-yaml${NC}"
        exit 1
    fi

    if parse_config_file "$CONFIG_FILE"; then
        echo -e "${GREEN}配置文件解析成功${NC}"
        apply_config
    else
        echo -e "${RED}错误: 配置文件解析失败${NC}"
        exit 1
    fi
fi

# 如果没有指定必要参数，使用默认值或配置文件中的值
if [ ${#remaining_args[@]} -eq 0 ]; then
    remaining_args+=("${config_environment:-develop}")
fi
if [ ${#remaining_args[@]} -eq 1 ]; then
    remaining_args+=("${config_build_type:-Debug}")
fi
if [ ${#remaining_args[@]} -eq 2 ]; then
    remaining_args+=("${config_dev_mode:-common}")
fi

# 决定最终编译模式
if [ "$MODULE_SPECIFIC_BUILD" = true ] && [ "$BUILD_MODE_SPECIFIED" = false ]; then
    # 当使用模块选择但没有明确指定编译模式时，默认使用 lite 模式
    BUILD_MODE="lite"
    echo -e "${YELLOW}模块选择模式下默认不编译 tests 和 tools${NC}"
    echo -e "${YELLOW}如需包含 tests 和 tools，请使用 --full 选项${NC}"
fi

# 执行编译
if [ "$BUILD_MODE" == "lite" ]; then
    echo -e "${GREEN}使用 Lite 模式编译${NC}"
    echo -e "${BLUE}传递参数: ${remaining_args[@]}${NC}"
    compile_with_flags 0 0 "${remaining_args[@]}"
else
    echo -e "${GREEN}使用 Full 模式编译${NC}"
    echo -e "${BLUE}传递参数: ${remaining_args[@]}${NC}"
    compile_with_flags 1 1 "${remaining_args[@]}"
fi

echo -e "${GREEN}=== 编译流程结束 ===${NC}"
