@echo off
set SERVER=root@192.168.0.51
set KH=%USERPROFILE%\.ssh\known_hosts
set KEY=%USERPROFILE%\.ssh\id_ed25519


scp -i "%KEY%" -o UserKnownHostsFile="%KH%" -o StrictHostKeyChecking=no -r "%SERVER%:/home/pi/.vs/IMU_G366/out/build/linux-debug/sample" "C:\Local\Matlab\IMU\sample"
scp -i "%KEY%" -o UserKnownHostsFile="%KH%" -o StrictHostKeyChecking=no -r "%SERVER%:/root/projects/TCS_RPi_HJ/TCS_dump" "C:\Local\Matlab\IMU\TCS_dump"
pause