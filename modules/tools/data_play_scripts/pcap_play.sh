#! /bin/bash
ip=$1
channel=$2
pcap_dir=$3
pcap=$4
looptime=$5
broadcast=$6

interface=enxf8e43b8d250a
smac=f8:e4:3b:8d:25:0a

#choose board
if [[ $ip -eq 137 ]]
then
    dmac=48:49:52:41:62:89
    src_old_port=10000
    src_new_port=10000
    if [[ $broadcast = 'multi' ]]
    then
        src_old_ip=192.168.5.191
        src_new_ip=192.168.1.191
        dst_old_ip=239.76.68.1
        dst_new_ip=192.168.1.137
        dst_old_port=21324
        dst_new_port=21324
    else
        src_old_ip=192.168.7.191
        src_new_ip=192.168.1.191
        dst_old_ip=192.168.7.137
        dst_new_ip=192.168.1.137
        dst_old_port=21323
        dst_new_port=21323
    fi
elif [[ $ip -eq 153 ]]
then
    dmac=48:49:52:41:a1:99
    src_old_port=10000
    src_new_port=10000
    if [[ $broadcast = 'multi' ]]
    then
        src_old_ip=192.168.5.190
        src_new_ip=192.168.1.190
        dst_old_ip=239.76.68.1
        dst_new_ip=192.168.1.153
        dst_old_port=21323
        dst_new_port=21323
    else
        src_old_ip=192.168.7.190
        src_new_ip=192.168.1.190
        dst_old_ip=192.168.7.153
        dst_new_ip=192.168.1.153
        dst_old_port=21323
        dst_new_port=21323
    fi
elif [[ $ip -eq 140 ]]
then
    dmac=48:49:52:41:72:8c
    src_old_port=10000
    src_new_port=10000
    if [[ $broadcast = 'multi' ]]
    then
        src_old_ip=192.168.5.194
        src_new_ip=192.168.1.194
        dst_old_ip=239.76.68.1
        dst_new_ip=192.168.1.140
        dst_old_port=21325
        dst_new_port=21325
    else
        src_old_ip=192.168.7.194
        src_new_ip=192.168.1.194
        dst_old_ip=192.168.7.140
        dst_new_ip=192.168.1.140
        dst_old_port=21323
        dst_new_port=21323
    fi
elif [[ $ip -eq 154 ]]
then
    dmac=48:49:52:41:b1:9a
    src_old_port=10000
    src_new_port=10000
    if [[ $broadcast = 'multi' ]]
    then
        src_old_ip=192.168.5.195
        src_new_ip=192.168.1.195
        dst_old_ip=239.76.68.1
        dst_new_ip=192.168.1.154
        dst_old_port=21326
        dst_new_port=21326
    else
        src_old_ip=192.168.7.195
        src_new_ip=192.168.1.195
        dst_old_ip=192.168.7.154
        dst_new_ip=192.168.1.154
        dst_old_port=21323
        dst_new_port=21323
    fi
elif [[ $ip -eq 133 ]]
then
    dmac=48:49:52:41:41:85
    dst_old_ip=192.168.10.133
    dst_new_ip=192.168.1.133
    if [[ $channel = 'right' ]]
    then
        src_old_ip=192.168.10.145
        src_new_ip=192.168.1.145
        src_old_port=10000
        src_new_port=10000
        dst_old_port=2366
        dst_new_port=2366
    else
        src_old_ip=192.168.10.144
        src_new_ip=192.168.1.144
        src_old_port=2368
        src_new_port=2368
        dst_old_port=2367
        dst_new_port=2367
    fi
else
    dmac=48:49:52:41:51:87
    dst_old_ip=192.168.10.135
    dst_new_ip=192.168.1.135
    if [[ $channel = 'left' ]]
    then
        src_old_ip=192.168.10.146
        src_new_ip=192.168.1.146
        src_old_port=10000
        src_new_port=10000
        dst_old_port=2368
        dst_new_port=2368
    else
        src_old_ip=192.168.10.147
        src_new_ip=192.168.1.147
        src_old_port=50112
        src_new_port=50112
        dst_old_port=50112
        dst_new_port=50112
    fi
fi

replay_data(){
    pacp_name=$1
    echo 播放${pacp_name}
    tcpprep -a client -i ${pacp_name} -o lidarPointCloud.cache
    tcprewrite  --enet-smac=${smac},${smac} --enet-dmac=${dmac},${dmac} -i ${pacp_name} --pnat=${src_old_ip}:${src_new_ip},${dst_old_ip}:${dst_new_ip} --portmap=${src_old_port}:${src_new_port},${dst_old_port}:${dst_new_port}  -o test10.pcap
   
    sudo tcpreplay --loop $2 --intf1=${interface} -P -x 1 test10.pcap
}

if [ -f $pcap ]
then
    loop_i=1
    while [ $loop_i -le $looptime ]
    do
        echo "--------开始--------"
        echo "第$loop_i次循环："
        replay_data $pcap $looptime
        let loop_i++
        echo "--------结束--------"
    done
else
    echo 需要输入一个文件或文件夹！
fi
