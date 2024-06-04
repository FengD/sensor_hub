## Used for collecting afred data
- `getprop | grep product` to get product number
- `cp init.afred_data_collect.rc /etc/framework/init/`, e.g. `cp init.afred_data_collect.rc /etc/framework/init/`
- `cd /etc/framework/init/ && chmod 644 init.afred_data_collect.rc`
- `cp afred_data_collect.sh /apps/data_collect/`
- `cd /apps/data_collect/ && chmod a+x afred_data_collect.sh`
- `reboot`
- then afred data in format of raw will being collected in /extdata/${PathMap[${product_num}]}/