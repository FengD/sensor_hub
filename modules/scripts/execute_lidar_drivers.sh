if [ -z ${CYBER_PATH} ];then
    export CYBER_PATH=/usr/local/apollo/cyber/
    echo "CYBER_PATH not given. Use default path ${CYBER_PATH}"
fi

if [ -z ${MAIN_WS} ];then
    export MAIN_WS=../
    echo "MAIN_WS not given. Use default path ${MAIN_WS}"
fi

if [ -z ${VIN} ];then
    export VIN=default_vin
    echo "VIN not given. Use the default ${VIN}"
fi

if [ -z $1 ];then
    echo "No lidar_config given. Use the default."
    ./lidar_drivers --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 0 --colorlogtostderr true
else
    echo "Use the given config $1"
    ./lidar_drivers --config_file $1 --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 0 --colorlogtostderr true
fi
