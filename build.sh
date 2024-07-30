#!/bin/bash

WS=$(cd $(dirname $0);pwd)

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'
BLUE='\033[0;34m'

function info() {
    (>&2 echo -e "[${WHITE}${BOLD} INFO ${NO_COLOR}] $*")
}

function error() {
    (>&2 echo -e "[${RED} ERROR ${NO_COLOR}] $*")
}

function warning() {
    (>&2 echo -e "[${YELLOW} WARNING ${NO_COLOR}] $*")
}

function ok() {
    (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

function print_delim() {
    echo '=================================================='
}

function get_now() {
    echo $(date +%s)
}

function print_time() {
    END_TIME=$(get_now)
    ELAPSED_TIME=$(echo "$END_TIME - $START_TIME" | bc -l)
    MESSAGE="Took ${ELAPSED_TIME} seconds"
    info "${MESSAGE}"
}

function success() {
    print_delim
    ok "$1"
    print_time
    print_delim
}

function fail() {
    print_delim
    error "$1"
    print_time
    print_delim
}

function print_usage() {
    echo -e "\n${RED}Usage${NO_COLOR}:
    .${BOLD}/build.sh${NO_COLOR} [OPTION]"

    echo -e "\n${RED}Options${NO_COLOR}:
    ${BLUE}all${NO_COLOR}: run the code style check
    ${BLUE}build${NO_COLOR}: run the code build
    ${BLUE}clean${NO_COLOR}: clean the code build
    ${BLUE}cov${NO_COLOR}: run the code test coverage
    ${BLUE}check_code [param]${NO_COLOR}: check code qulity param 'help' for check_code param list
    "
}

function clean() {
    rm -rf build build_dist cov
}

function check_code() {
    git clone https://github.com/FengD/code_check_tools.git
    chmod -R +x code_check_tools
    export WORKSPACE=${WS}
    export CODE_CHECK_EXCLUDE_LIST="3rdparty,tools,gstcamera,test"
    echo -e "${RED}exclude list: $CODE_CHECK_EXCLUDE_LIST ${NO_COLOR}"
    ./code_check_tools/code_check.sh $param
    if [ $? -eq 0 ]; then
        rm -rf code_check_tools
        exit 0
    else
        rm -rf code_check_tools
        exit 1
    fi
}

function gen_coverage() {
  export LD_LIBRARY_PATH=${WS}/build/modules/crdc_airi_common/lib/
  cd ${WS}
  rm -rf cov
  mkdir -p cov
  lcov -d ./build -c -i -o cov/init.info --rc lcov_branch_coverage=1
  cd build
  ctest
  cd ${WS}
  lcov -d ./build -b modules --no-external --rc lcov_branch_coverage=1  -c -o cov/coverage.info
  lcov -a cov/init.info -a cov/coverage.info -o cov/total.info --rc lcov_branch_coverage=1
  lcov -r cov/total.info "*test*" "*cyber/*" "*build/*" "*/usr/include/*" "*/tools/*" "*/usr/local/include/*" "*/usr/lib/*" "*/opt/*" "*/3rdparty/*" -o cov/final.info --rc lcov_branch_coverage=1 'build/*'
  cd cov
  genhtml final.info --rc lcov_branch_coverage=1
}

function build_make() {
    MAX_CPU_NUM=$(nproc)
    cd ${WS}/build
    make -j$[${MAX_CPU_NUM}-1] install/strip
    if [ $? -eq 0 ]; then
        success 'Build passed!'
    else
        fail 'Build failed!'
        exit 1
    fi
}

function build() {
    mkdir -p ${WS}/build && cd ${WS}/build
    cmake -DCMAKE_BUILD_TYPE=RELEASE \
        -DWITH_COV=${WITH_COV} \
        -DDO_TEST=${DO_TEST} \
        -DWITH_ROS2=${WITH_ROS2} \
        -DAFRED_DATA_COLLECT=${AFRED_DATA_COLLECT} \
        ${EXTRA_OPTIONS} \
        -DCMAKE_INSTALL_PREFIX=${WS}/build_dist ..
    build_make
}


function main() {
    local cmd=$1
    local param=${@:2}
    if [ -z $2 ]; then
        param=run
    fi
    cd ${WS}
    if [ -z $1 ]; then
        if [ ! -d "build" ]; then
            cmd=all
        else
            cmd=build
        fi
    fi
    shift

    START_TIME=$(get_now)
    WITH_COV=OFF
    DO_TEST=OFF
    WITH_ROS2=OFF
    AFRED_DATA_COLLECT=OFF


    if [ "${TAG}" = "ROS2" ]; then
        WITH_ROS2=ON
    fi

    if [[ "${PLATFORM}" == "TDA4" ]];then
        EXTRA_OPTIONS="${EXTRA_OPTIONS} -DWITH_TDA4=ON"
    elif [[ "${PLATFORM}" == "X86" ]];then
        EXTRA_OPTIONS="${EXTRA_OPTIONS} -DWITH_IPC=ON -DCALIBRATE=OFF"
    fi

    if [ "${ARCH}" = "arm64" ]; then
        EXTRA_OPTIONS="${EXTRA_OPTIONS} -DWITH_AARCH64=ON"
    fi

    case $cmd in
        build)
            build_make
            ;;
        all)
            build
            ;;
        clean)
            clean
            ;; 
        cov)
            WITH_COV=ON
            DO_TEST=ON
            build
            gen_coverage
            ;;
        check_code)
            clean
            check_code $param
            ;;
        *)
            print_usage
            ;;
    esac
}

main $@
