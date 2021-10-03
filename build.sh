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
    ${BLUE}build_cross${NO_COLOR}: cross compile for aarch64
    "
}

function clean() {
    rm -rf build build_dist
}

function get_dependencies() {
    echo "get-dependencies"
}

function build_make() {
    MAX_CPU_NUM=$(nproc)
    cd ${WS}/build
    make -j$[${MAX_CPU_NUM}-1]

    if [ $? -eq 0 ]; then
        success 'Build passed!'
    else
        fail 'Build failed!'
    fi
}

function build() {
    mkdir -p ${WS}/build && cd ${WS}/build
    cmake -DWITH_COV=${WITH_COV} \
          -DDO_TEST=${DO_TEST} \
          -DBUILD_AARCH64=${WITH_AARCH64} ..
    build_make
}

function main() {
    local cmd=$1
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
    WITH_AARCH64=OFF
    case $cmd in
        build)
            build_make
            ;;
        all)
            get_dependencies
            build
            ;;
        clean)
            clean
            ;; 
        cov)
            WITH_COV=ON
            DO_TEST=ON
            get_dependencies
            build
            ;;
        build_cross)
            WITH_AARCH64=ON
            get_dependencies
            build
            ;;
        *)
            print_usage
            ;;
    esac
}

main $@