# hw3

Compile part : 
1. mkdir -p ~/ee2405new
2. cp -r ~/ee2405/mbed-os ~/ee2405new
3. cd ~/ee2405new
4. mbed compile --library --no-archive -t GCC_ARM -m B_L4S5I_IOT01A --build ~/ee2405new/mbed-os-build2

then compile the main.cpp with this command : 
sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -ff 

Set up part : 
1. Wait for the Wifi to connect(Message will be shown on screen).
2. type "/gc/run" to start gesture UI mode.
  (1) LED1 will blink 3 times.
  (2) Use gesture "slope" and "ring" to increase or decrease the angle.
  (3) Press user button to confirm the angle
3. type "/tilt/run" to start tilt angle mode.
  (1) LED2 will blink 3 times, put the mbed on table.
  (2) Tilt the mbed, when its angle over the chosen angle, the current angle will show on screen.
  (3) When it over the chosen angle for 10 times, the tilt angle mode will stop.
