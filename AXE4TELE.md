#mimic mathias Desrochers eltopchi1@gmail.com

Added changes 

new leadera nd follower (axe4)
added imu tm171 support -> must be set on quaternions

imu : "/lerobot/imu_reader/IMU_Project$ ./imu_udp " or ./imu_udp /dev/ttyACM1


verify its sending data 

python imu_reader/test_receiver.py

start arm 

lerobot-teleoperate \
    --robot.type=axe4_follower \
    --robot.cameras={} \
    --robot.udp_port=5005 \
    --teleop.type=axe4_leader \
    --teleop.port=/dev/ttyACM4 \
    --teleop.imu_port=5000 \
    --teleop.id=axe \
    --display_data=false 


test 

python simple_udp_receiver.py

for more testing , instead of isaac Lab : 

pip install mujoco

python tester_eef/eef_tester.py


