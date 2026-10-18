# esp32_base_driver

Node ROS 2 Humble nối `/cmd_vel` với firmware ESP32-S3, dựng `/odom` + TF `odom -> base_link`
từ xung encoder. Chạy trên Raspberry Pi 4, cổng `/dev/serial0`.

## Cài

```bash
cd ~/<ws>/src
cp -r /duong/dan/esp32_base_driver .
cd ~/<ws>
sudo apt install python3-serial ros-humble-teleop-twist-keyboard
colcon build --packages-select esp32_base_driver --symlink-install
source install/setup.bash
```

## Chạy

```bash
ros2 launch esp32_base_driver base.launch.py        # chi driver
ros2 launch esp32_base_driver test_drive.launch.py  # driver + 6 pha kiem tra
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Đấu dây (Pi 4 header 40 chân)

| ESP32-S3 | Pi 4 |
|---|---|
| GPIO13 (TX) | pin 10 (RXD0) |
| GPIO12 (RX) | pin 8 (TXD0) |
| GND | pin 6 |

Pi cần `enable_uart=1` và đã tắt serial console (`raspi-config` -> Interface Options
-> Serial Port -> login shell **No**, hardware **Yes**).

## Tham số

Sửa trong `config/base_params.yaml`. Hai tham số quan trọng nhất cần hiệu chuẩn
trên sàn sau khi test xong: `wheel_diameter` và `wheel_separation`.
