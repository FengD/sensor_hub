#! /bin/bash
ps -aux|grep ./pcap_play.sh|grep $1 > 1.txt
if [[ -s 1.txt ]]
then
    echo 'this board is using'
    exit 0
else
    echo 'start broast'
    if [[ $1 = "--help" ]] || [[ $1 = "-h" ]]
    then
        a="param1:ip(eg_137)\nparam2:which channel(hav:fl,fr,rl or rr;j6p:left,right,up or rear)\nparam3:which pcap dir\nparam4:which pcap\nparam5:loop time(eg_100)\nparam6:broadcast choice(multi or single)"
        b="-h or --help:help\n--listdir:get remote pcap dir info --listpcap &param2:get remote pcap data info of the pcap dir"
        echo -e $b
        echo -e $a
        exit 0
    elif [[ $1 = "--listdir" ]]
    then
        expect -c 'spawn ssh -p 10049 root@10.30.5.19; expect "password:"; send "123\r";expect "*root*";send "python data_extract.py\n";send "exit\n";interact'
        exit 0
    elif [[ $1 = "--listpcap" ]] 
    then
        expect -c 'spawn ssh -p 10049 root@10.30.5.19; expect "password:"; send "123\r";expect "*root*";send "cd /data/ai_data/collect_data/dataset/'$2'/raw_data/pcap\n";send "ls\n";send "exit\n";interact'
        exit 0
    else
        if [ ! -e $4 ]
        then
        expect -c 'spawn scp -P 10049 root@10.30.5.19:/data/ai_data/collect_data/dataset/'$3'/raw_data/pcap/'$4' remote/; expect "password:"; send "123\r";interact'
        ./pcap_play.sh $1 $2 $3 remote/$4 $5 $6
        else
        ./pcap_play.sh $1 $2 $3 $4 $5 $6 #$1:ip,$2:which channel,$3:remote dir or not,$4:which pcap,$5:loop time,$6: broadcast choose multi or single
        fi   
    fi
fi