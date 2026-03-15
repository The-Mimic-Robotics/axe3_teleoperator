#mimic mathias Desrochers eltopchi1@gmail.com

Added changes 

new leader and follower (axe4)
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
    --teleop.port=/dev/ttyACM0 \
    --teleop.imu_port=5000 \
    --teleop.id=axe \
    --display_data=false 


test 

python simple_udp_receiver.py

for more testing , instead of isaac Lab : 

pip install mujoco

python tester_eef/eef_tester.py

nathanael mccooeye nathanaelmccooeye@gmail.com

To connect to the servo driver and get datastream
1. connect usbc and barrel adapter power to servo driver
2. run "ls /dev/ttyACM*" should output "/dev/ttyACM0"
3. run "conda info --envs" to check environments
4. run "conda activate lerobot" 
5. (this should already have been run to give access to the port: "usermod -a -G dialout $USER")
6. in the code above starting with "lerobot-teleoperate \", ensure the teleop.port is correct (based on step 2)
7. copy and run the mentioned code above 