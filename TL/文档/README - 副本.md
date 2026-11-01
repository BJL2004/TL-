# 启动上电
bash ~/Desktop/TL/scripts/run_driver.sh

# 启动手眼标定
bash ~/Desktop/TL/TL_Dobot/scripts/run_handeye_collect.sh

# 启动抓取、放置种子


mosquitto_sub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' -t node/arm/01/up -q 1 -v &
mosquitto_pub -h 116.205.245.172 -u shapandevice -P 'qwer1234!!' \
  -t node/arm/01/down -m '{"cmd":"seed","targetId":"A2-01"}' -q 1


