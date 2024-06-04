# CYBER_RT

## Description
Apollo cyber is a great runtime framework.

* cyber: The apollo [cyber](https://github.com/ApolloAuto/apollo/tree/master/cyber)

## Build
`./build.sh` build the project.
`./build.sh clean` clean the build.
...
run `./build.sh help` to see the details.

## for aarch64
export ARCH="arm64" (normally if you are in the cross compile enviroment the variable should be already exists.)
`./build.sh` for cross build.

## for tools
`./build.sh tools` for build the tools.

## Install
`./build.sh install` install to install path.

## Tools
* deps: qt5

## Dependencies
* sudo apt install asio-dev
* sudo apt install libtinyxml2-dev
* uuid
* rt
* pthread
* protobuf 3.15.0
* fastrtps (in 3rd_party for x86_64/aarch64) copy `lib` in /usr/local/lib and `include` in /usr/local/include
* poco 1.9.0
* gflags 2.2.2
* glog 0.4.0

## Best practice
[test_project](https://github.com/FengD/apollo_cyber_test)