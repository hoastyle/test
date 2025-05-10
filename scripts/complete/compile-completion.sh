#!/bin/bash
# Autane编译系统通用自动补全脚本（支持bash和zsh）

# 检测shell类型
_detect_shell() {
    if [ -n "$BASH_VERSION" ]; then
        echo "bash"
    elif [ -n "$ZSH_VERSION" ]; then
        echo "zsh"
    else
        echo "unknown"
    fi
}

# 获取项目根目录
_get_project_root() {
    local dir=$(pwd)
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/CMakeLists.txt" ] && [ -d "$dir/src/detector" ]; then
            echo "$dir"
            return
        fi
        dir=$(dirname "$dir")
    done
    echo "."
}

# 提取模块列表（带类型前缀）
_get_modules_with_prefix() {
    local project_root=$(_get_project_root)
    local detector_dir="$project_root/src/detector"
    
    if [ -d "$detector_dir" ]; then
        # 获取perception模块，添加前缀
        find "$detector_dir/perception" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | xargs -n1 basename | sed 's/^/perception\//'
        # 获取vision模块，添加前缀
        find "$detector_dir/vision" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | xargs -n1 basename | sed 's/^/vision\//'
    fi
}

# 智能模块补全（支持部分匹配）
_complete_modules() {
    local cur="$1"
    local modules=($(_get_modules_with_prefix))
    
    # 如果输入包含/，则只返回匹配的完整路径
    if [[ $cur == */* ]]; then
        printf '%s\n' "${modules[@]}" | grep "^$cur"
    else
        # 否则返回所有模块，优先显示类型
        printf '%s\n' "${modules[@]}"
    fi
}

# 查找配置文件（包括.conf, .cfg）
_get_config_files() {
    local project_root=$(_get_project_root)
    find "$project_root" -maxdepth 3 -type f \( -name "*.config" -o -name "*.yaml" -o -name "*.yml" -o -name "*.conf" -o -name "*.cfg" \) | sed "s|$project_root/||" | sort
}

# bash补全函数
_compile_complete_bash() {
    local cur prev words cword
    if command -v _init_completion &> /dev/null; then
        _init_completion || return
    else
        # fallback for systems without bash-completion
        cur="${COMP_WORDS[COMP_CWORD]}"
        prev="${COMP_WORDS[COMP_CWORD-1]}"
        words=("${COMP_WORDS[@]}")
        cword=$COMP_CWORD
    fi

    # 定义选项
    local build_modes="--full --lite"
    local module_options="--modules --perception-only --vision-only --exclude"
    local config_options="--config"
    local help_options="--help -h"
    local environments="develop product"
    local build_types="Debug Release Debug|Release"
    local dev_modes="common sdk"
    
    # 检查当前位置和前一个选项
    case "$prev" in
        --config)
            # 补全配置文件，支持路径展开
            local files=($(_get_config_files))
            COMPREPLY=($(compgen -W "${files[*]}" -- "$cur"))
            return 0
            ;;
        --modules|--exclude)
            # 智能补全模块名称
            local modules=($(_complete_modules "$cur"))
            COMPREPLY=($(compgen -W "${modules[*]}" -- "$cur"))
            return 0
            ;;
    esac

    # 如果当前输入是选项
    if [[ $cur == -* ]]; then
        local all_options="$build_modes $module_options $config_options $help_options"
        COMPREPLY=($(compgen -W "$all_options" -- "$cur"))
        return 0
    fi

    # 统计非选项参数位置
    local arg_pos=0
    local i
    for ((i=1; i<$cword; i++)); do
        case "${words[i]}" in
            --config|--modules|--exclude)
                ((i++))  # 跳过选项的参数
                ;;
            --*) ;;
            *) ((arg_pos++)) ;;
        esac
    done

    # 根据位置提供补全
    case "$arg_pos" in
        0)
            # 第一个位置：环境选择
            COMPREPLY=($(compgen -W "$environments" -- "$cur"))
            ;;
        1)
            # 第二个位置：构建类型
            COMPREPLY=($(compgen -W "$build_types" -- "$cur"))
            ;;
        2)
            # 第三个位置：开发模式
            COMPREPLY=($(compgen -W "$dev_modes" -- "$cur"))
            ;;
    esac
}

# zsh补全函数
_compile_complete_zsh() {
    # zsh自动补全初始化
    autoload -U compinit && compinit

    local -a build_modes module_options config_options help_options environments build_types dev_modes
    
    build_modes=("--full" "--lite")
    module_options=("--modules" "--perception-only" "--vision-only" "--exclude")
    config_options=("--config")
    help_options=("--help" "-h")
    environments=("develop" "product")
    build_types=("Debug" "Release" "Debug|Release")
    dev_modes=("common" "sdk")

    # 组合所有选项
    local -a all_options=("${build_modes[@]}" "${module_options[@]}" "${config_options[@]}" "${help_options[@]}")

    # 获取当前位置的单词
    local cur="${words[$CURRENT]}"
    local prev="${words[$((CURRENT-1))]}"
    
    # 根据前一个单词决定补全内容
    case "$prev" in
        --config)
            # 补全配置文件
            local config_files=($(_get_config_files))
            compadd -a config_files
            return 0
            ;;
        --modules|--exclude)
            # 补全模块
            local modules=($(_complete_modules "$cur"))
            compadd -a modules
            return 0
            ;;
    esac

    # 如果输入以-开头，显示所有选项
    if [[ $cur == -* ]]; then
        compadd -a all_options
        return 0
    fi

    # 统计非选项参数位置
    local arg_pos=0
    local i
    for ((i=1; i<$CURRENT; i++)); do
        case "${words[i]}" in
            --config|--modules|--exclude)
                ((i++))  # 跳过选项的参数
                ;;
            --*) ;;
            *) ((arg_pos++)) ;;
        esac
    done

    # 根据位置提供补全
    case "$arg_pos" in
        0)
            compadd -a environments
            ;;
        1)
            compadd -a build_types
            ;;
        2)
            compadd -a dev_modes
            ;;
    esac
}

# 主补全函数，根据shell类型调用相应的补全函数
_compile_complete() {
    case $(_detect_shell) in
        bash)
            _compile_complete_bash
            ;;
        zsh)
            _compile_complete_zsh
            ;;
        *)
            echo "Unsupported shell"
            ;;
    esac
}

# 注册补全函数
if [ -n "$BASH_VERSION" ]; then
    complete -F _compile_complete_bash compile.sh
    complete -F _compile_complete_bash compileLite.sh
    complete -F _compile_complete_bash ./compile.sh
    complete -F _compile_complete_bash ./compileLite.sh
elif [ -n "$ZSH_VERSION" ]; then
    compdef _compile_complete compile.sh
    compdef _compile_complete compileLite.sh
    compdef _compile_complete ./compile.sh
    compdef _compile_complete ./compileLite.sh
fi

# 如果直接执行此脚本，显示使用说明
if [[ "${BASH_SOURCE[0]}" == "${0}" ]] || [[ "${(%):-%N}" == "$0" ]]; then
    cat << 'EOF'
Autane编译系统通用自动补全脚本（支持bash和zsh）

安装方法（根据你使用的shell）：

For Bash:
1. 临时加载：
   source scripts/complete/compile-completion.sh

2. 永久加载：
   echo "source $(pwd)/scripts/complete/compile-completion.sh" >> ~/.bashrc

For Zsh (oh-my-zsh):
1. 临时加载：
   source scripts/complete/compile-completion.sh

2. 永久加载（如果使用oh-my-zsh）：
   mkdir -p ~/.oh-my-zsh/custom/completions
   cp scripts/complete/compile-completion.sh ~/.oh-my-zsh/custom/completions/_autane
   
   或添加到 ~/.zshrc：
   source $(pwd)/scripts/complete/compile-completion.sh

使用示例：
   ./compile.sh --modules perception/<Tab>   # 显示perception模块
   ./compile.sh --config <Tab>               # 显示可用配置文件
   ./compile.sh --exclude vision/<Tab>       # 排除特定vision模块
   ./compile.sh develop <Tab>                # 补全构建类型
   ./compile.sh <Tab><Tab>                   # 显示所有选项

提示：按两次Tab键可以显示所有可用选项
EOF
fi
