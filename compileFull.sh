#!/bin/bash

# 颜色定义，改善输出可读性
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# 全局状态
makeResult=0

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_debug() { echo -e "${BLUE}[DEBUG]${NC} $*"; }

# 进度条函数
show_progress() {
    local current=$1
    local total=$2
    local message=$3
    local percent=$((current * 100 / total))
    printf "\r${GREEN}[%3d%%]${NC} %s" "$percent" "$message"
    [ $current -eq $total ] && echo ""
}

# 从配置文件读取项目列表
get_project_info() {
    local config_file="${MMRoot}/project.info"
    if [ ! -f "$config_file" ]; then
        cat > "$config_file" << EOL
# 项目配置文件
mw1:Autane system for ARTGs in WAISI
my2:Autane system for ARTGs in NINGBO
mt3:Autane system for ARTGs in TAICANG
atl:Anti-Truck-Lift system on ZPMC
sil:SIL safety certificate in WAISI
perc:Perception crane system in WAISI
EOL
    fi

    declare -A projects
    while IFS=: read -r id desc; do
        [[ $id =~ ^#.*$ ]] && continue  # 跳过注释行
        projects["$id"]=$desc
    done < "$config_file"

    printf '\n%s\n' "可用项目:"
    for id in "${!projects[@]}"; do
        printf " %-6s - %s\n" "$id" "${projects[$id]}"
    done
    echo ""
}

# 优化的项目信息检查
checkProjectInfo() {
    local CFGInfo="${MMRoot}/build/project.sh"

    if [ ! -f "${CFGInfo}" ]; then
        get_project_info
        read -p "选择项目 [mw1|my2|mt3|atl|sil|perc]: " projectID

        local machs=$(getGantries "$settingsDir/$projectID")
        if [[ " mw1 my2 mt3 atl sil perc " =~ " ${projectID} " ]]; then
            read -p "选择吊机 [$machs]: " gantryID
        else
            log_error "未识别的项目 ${projectID}"
            rm -f "${CFGInfo}"
            exit 1
        fi

        cat > "${CFGInfo}" << EOL
#!/bin/bash
project=${projectID}
gantry=${gantryID}
EOL
    fi

    source "${CFGInfo}"

    if [ -z "${project}" ] || [ -z "${gantry}" ]; then
        log_error "build/${CFGInfo} 配置不完整"
        rm -f "${CFGInfo}"
        exit 1
    fi

    local settings_path="${MMRoot}/settings/${project}/${gantry}"
    if [ -d "$settings_path" ]; then
        log_info "已选择 settings/${project}/${gantry}，详见 build/project.sh"
    else
        log_error "settings/${project}/${gantry} 不存在"
        rm -f "${CFGInfo}"
        exit 1
    fi
}

# 性能优化的库包解码
decodeLibPackage() {
    local buildType=$1
    local installDir="${MMRoot}/build/${buildType}/install"
    local libMatched=1

    # 使用关联数组提高查找效率
    declare -A perceptionModules visionModules

    # 收集模块文件夹名称
    for dir in "${MMRoot}/src/detector/perception/"*; do
        [ -d "$dir" ] && perceptionModules["$(basename "$dir")"]=""
    done
    for dir in "${MMRoot}/src/detector/vision/"*; do
        [ -d "$dir" ] && visionModules["$(basename "$dir")"]=""
    done

    # 检查源文件存在性
    local perceptionHasSource=0 visionHasSource=0

    for dir in "${MMRoot}/src/detector/perception/"*; do
        if [ -d "$dir" ] && find "$dir" -name "*.cpp" -o -name "*.hpp" | grep -q .; then
            perceptionHasSource=1
            break
        fi
    done

    for dir in "${MMRoot}/src/detector/vision/"*; do
        if [ -d "$dir" ] && find "$dir" -name "*.cpp" -o -name "*.hpp" | grep -q .; then
            visionHasSource=1
            break
        fi
    done

    if [ $perceptionHasSource -eq 1 ] && [ $visionHasSource -eq 1 ]; then
        log_info "在 perception 和 vision 目录中发现源文件，跳过库检查。"
        return
    fi

    # 处理库包
    local libPackage="lib.tar.gz"
    if [ -f "${MMRoot}/lib/${libPackage}" ]; then
        pushd "${MMRoot}/lib" > /dev/null

        local lastModifyTimestamp=$(stat -c %Y "${libPackage}")
        local lastModifyTime=$(date +%Y%m%d%H%M%S -d "@${lastModifyTimestamp}")
        local timeRecord=0
        local CACHE="${installDir}/lib/.cache"

        mkdir -p "${CACHE}"
        [ -f "${CACHE}/libModifiedTime.txt" ] && timeRecord=$(cat "${CACHE}/libModifiedTime.txt")

        if [ "${timeRecord}" != "${lastModifyTime}" ]; then
            log_info "解压 ${libPackage} (时间戳: ${lastModifyTime})"
            echo "${lastModifyTime}" > "${CACHE}/libModifiedTime.txt"

            tar zxf "${libPackage}"
            mkdir -p "${installDir}/lib"

            # 优化文件移动操作
            for file in lib/*; do
                if [ -f "$file" ]; then
                    local basename=$(basename "$file")
                    local skip=0

                    for module in "${!perceptionModules[@]}" "${!visionModules[@]}"; do
                        if [[ $basename == *"$module"* ]]; then
                            skip=1
                            break
                        fi
                    done

                    [ $skip -eq 0 ] && mv "$file" "${installDir}/lib/"
                fi
            done

            rm -rf lib
        fi
        popd > /dev/null
    else
        libMatched=0
        log_warn "lib/lib-${buildType}.tar.gz 不存在"
    fi

    # 版本检查优化
    local version1="${installDir}/lib/version.txt"
    local version2="${MMRoot}/lib/version.txt"

    if [ $libMatched -ne 0 ] && ! diff -q "${version1}" "${version2}" > /dev/null 2>&1; then
        libMatched=0
        log_warn "库版本不匹配"
        echo ""
        echo "期望版本 (${version1}):"
        [ -f "${version1}" ] && cat "${version1}"
        echo ""
        echo "当前版本 (${version2}):"
        [ -f "${version2}" ] && cat "${version2}"
    fi

    if [ $libMatched -ne 1 ]; then
        cat << EOL
        请下载匹配的 lib-${buildType}.tar.gz：
------------------------------------------------------
cd ..
git clone ssh://your.name@192.168.10.25:10005/iCrane2/libRel
cp libRel/lib-${buildType}.tar.gz iCrane2/lib
cd iCrane2
./compileFull.sh
EOL
        exit 1
    fi
}

# 优化的准备运行环境
prepareRuntimeEnv() {
    local buildType=$1
    local installDir="${MMRoot}/build/${buildType}/install"

    # 优化目录创建
    mkdir -p "${installDir}/"{bin,lib,include}

    decodeLibPackage "${buildType}"

    # 数据集和模型目录处理
    if [[ $FORM == "develop" ]]; then
        log_info "准备开发环境"
        cp -rfd "${MMRoot}/datasets" "${installDir}"
        cp -prfd "${MMRoot}/lib/"*.so "${installDir}/lib/"

        ln -sf "${MMRoot}/models" "${installDir}/models"
        # ln -sf "${MMRoot}/settings/${project}/${gantry}" "${installDir}/settings"
    else
        log_info "准备生产环境"
        rm -rf "${installDir:?}"/*
        mkdir -p "${installDir}/"{bin,lib,include}

        decodeLibPackage "${buildType}"

        cp -rfd "${MMRoot}/models" "${installDir}"
        cp -rfd "${MMRoot}/datasets" "${installDir}"
        cp -prfd "${MMRoot}/lib/"*.so "${installDir}/lib/"
        cp -rfL "${MMRoot}/settings/${project}/${gantry}" "${installDir}/settings"
    fi
}

# 映射项目特定配置
mapProjectSpecial() {
    local currInclude="${MMRoot}/include/common/core/curr"
    local currServer="${MMRoot}/server/curr"

    ln -sf "${project}" "${currInclude}"
    ln -sf "${project}" "${currServer}"
}

# 优化的编译流程
compilePackage() {
    local buildType=$1
    local installDir="${MMRoot}/build/${buildType}/install"

    log_info "开始 ${buildType} 模式编译"

    # 配置ccache
    config_ccache

    prepareRuntimeEnv "${buildType}"
    mapProjectSpecial

    # 生成项目配置
    cat > project.yaml << EOL
# Generated automatically
form: ${FORM}
mode: ${MODE}
project: ${project}
gantry: ${gantry}
EOL
    mv project.yaml "${installDir}/settings"

    # 设置CMake模式
    cd "${buildType}" || exit 1

    echo "set(DEV_MODE \"${MODE}\")" > mode.cmake

    # 运行CMake with Ninja
    log_info "运行 CMake with Ninja..."
    if [ -n "$EXTRA_CMAKE_ARGS" ]; then
        log_info "应用额外的CMake参数: $EXTRA_CMAKE_ARGS"
        cmake ../.. -GNinja -DCMAKE_BUILD_TYPE="${buildType}" $EXTRA_CMAKE_ARGS -Wno-dev
    else
        cmake ../.. -GNinja -DCMAKE_BUILD_TYPE="${buildType}" -Wno-dev
    fi

    # 更新模型目录
    cp -rfd "${MMRoot}/models" "${installDir}"
    cp -r "${MMRoot}/scripts/bin/"* "${installDir}/bin/"

    # 智能线程数计算 - 保持和原程序完全一致
    local nthreads=$(calculate_threads)
    log_info "使用 ${nthreads} 个线程编译"

    # 计时编译
    local start_time=$(date +%s)
    # 支持详细输出模式（新增）
    if [ "$VERBOSE_BUILD" = "1" ]; then
        log_info "启用详细输出模式"
        ninja -v -j"${nthreads}"
    else
        ninja -j"${nthreads}"
    fi
    makeResult=$?
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    log_info "编译耗时: ${duration} 秒"

    chmod -x "${installDir}/lib/"*
    cd ..
}

# 智能计算编译线程数
calculate_threads() {
    local total_cores=$(nproc)
    local load_avg=$(awk '{print $1}' /proc/loadavg)
    local available_cores=$(bc <<< "$total_cores - $load_avg" | cut -d'.' -f1)

    # 至少使用1个核心，最多使用总核心数的80%
    local nthreads=$((available_cores > 0 ? available_cores : 1))
    local max_threads=$((total_cores * 8 / 10))

    # 特殊IP处理（优化为配置文件）
    local config_threads=$(get_config_threads)
    [[ $config_threads -gt 0 ]] && nthreads=$config_threads

    # 从环境变量读取配置文件指定的线程数（新增）
    if [ -n "$COMPILE_THREADS" ] && [ "$COMPILE_THREADS" != "0" ]; then
        nthreads=$COMPILE_THREADS
        log_info "Using configured thread count: $nthreads" >&2
    fi

    printf "%d\n" "$((nthreads > max_threads ? max_threads : nthreads))"
}

# 从配置文件读取特定IP的线程数
get_config_threads() {
    local config_file="${MMRoot}/build/compile.config"

    if [ ! -f "$config_file" ]; then
        cat > "$config_file" << EOL
# 特定IP的编译线程配置
192.168.10.137:10
EOL
    fi

    local current_ip=$(hostname -I | awk '{print $1}')
    while IFS=: read -r ip threads; do
        [[ $ip =~ ^#.*$ ]] && continue
        if [ "$ip" == "$current_ip" ]; then
            echo "$threads"
            return
        fi
    done < "$config_file"

    echo "0"
}

# 配置测试选项
configTests() {
    local VALUE=$1
    local TFILE="${MMRoot}/build/tests.cmake"

    cat > "${TFILE}" << EOL
unset(MM_ENABLE_TESTS CACHE)
option(MM_ENABLE_TESTS "unit and function tests" ${VALUE})
EOL
}

# 配置工具选项
configTools() {
    local VALUE=$1
    local TFILE="${MMRoot}/build/tools.cmake"

    cat > "${TFILE}" << EOL
unset(MM_ENABLE_TOOLS CACHE)
option(MM_ENABLE_TOOLS "tools of Autane system" ${VALUE})
EOL
}

# 配置ccache
config_ccache() {
    export USE_CCACHE=1
    export CCACHE_EXEC=$(which ccache)

    export CCACHE_DIR="${HOME}/.ccache"
    mkdir -p "${CCACHE_DIR}"

    local max_size=$(ccache -s | grep "max cache size" | awk '{print $4" "$5}')
    local current_size=$(ccache -s | grep "cache size" | awk '{print $3" "$4}')

    log_info "ccache状态:"
    log_info "  最大容量: $max_size"
    log_info "  当前使用: $current_size"

    if ccache -s | grep -E "max cache size.*[0-9]+.*G" | awk '{exit $4 < 10 ? 0 : 1}'; then
        log_info "设置ccache最大容量为20GB"
        ccache -M 20G
    fi

    export CC="ccache gcc"
    export CXX="ccache g++"
}

# 主流程
main() {
    # 环境设置
    source "./scripts/util/listGantries.sh"
    local execuDir=$(cd "$(dirname "$0")" && pwd)
    local settingsDir="$execuDir/settings"

    # 添加ccache的环境变量提示
    log_info "启用ccache和ninja加速编译"
    log_info "ccache统计: $(ccache -s | head -1)"

    # 解析参数
    local FORM="${1:-develop}"
    local TYPE="${2:-Debug}"
    local MODE="${3:-common}"

    # 验证参数
    if [[ ! "$FORM" =~ ^(develop|product)$ ]] ||
       [[ ! "$TYPE" =~ ^(Debug|Release|Debug\|Release)$ ]] ||
       [[ ! "$MODE" =~ ^(common|sdk)$ ]]; then
        log_error "参数错误"
        cat << EOL
用法:
./compileFull.sh
./compileFull.sh [develop|product]
./compileFull.sh [develop|product] [Debug|Release]
./compileFull.sh [develop|product] [Debug|Release] [common|sdk]
EOL
        exit 1
    fi

    # 设置根目录
    MMRoot=$(cd "$(dirname "$0")" && pwd)
    mkdir -p build
    cd build

    # 配置测试和工具选项
    configTests "${testsNeeded:-ON}"
    configTools "${toolsNeeded:-ON}"

    checkProjectInfo

    # 编译流程
    local debugNeeded=0 releaseNeeded=0

    case $TYPE in
        "Debug") debugNeeded=1 ;;
        "Release") releaseNeeded=1 ;;
        *) debugNeeded=1; releaseNeeded=1 ;;
    esac

    if [ $debugNeeded -eq 1 ]; then
        compilePackage Debug
        ln -sf "${MMRoot}/build/Debug/install" "${MMRoot}/build/install"
    fi

    if [ $releaseNeeded -eq 1 ]; then
        compilePackage Release
        ln -sf "${MMRoot}/build/Release/install" "${MMRoot}/build/install"
    fi
}

# 执行主流程
main "$@"
cd ..
exit $makeResult
