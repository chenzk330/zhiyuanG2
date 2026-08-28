Goal:
  Create a MQTT based robot control server. The robot hardware is agiBot-G2. 

Steps:

1 Read all the *.py files from /home/agi/app/gdk/examples , to understand the agiBot-G2 robot control commands and how to use them.
2 read all the *.py in services_backup/ , to understand the current logic of the robot control server.
  I have already create 4 services for the robot control server, they are:
  - camera.py , to read the camera data from the robot and send it to the client, and receive commands from the client to control the camera , like save photo
  - g2_minth_app_service.py , to accept the commands from the client and send them to the robot.
  - g2_minth_data.service.py , to read all the datas from /datas folder, and send them to the client, mainly about robot joints 
  - gs_minth_status_publisher.py, to read the robot status and send it to the client, like joint angles

3 read all the files in /web/js/ and vue_app.html , to understand the logic of the web client, and how to send commands to the robot control server, and how to receive data from the robot control server.

4 follow the steps below to see the aigBot-G2's DDS topics :
   source /home/app/env.sh
   dds_view

5 refactor the code in the robot control server to make it more clear and easy to understand, and add more comments to explain the logic of the code.
  try to fit the dds_view's structure.

rules:

1 the mqtt topic structure should be like this:
   - /humanoid/camera/
   - /humanoid/joints/
   - /humanoid/status/
   - /humanoid/commands/

2 mqtt message is json format, and the message structure should be like this:
   - camera: {"command": "save_photo"}
   - joints: {"joint1": 0.0, "joint2": 0.0, ...}
   - status: {"battery": 100, "temperature": 30, ...}
   - commands: {"command": "move_forward", "speed": 1.0}

3 there should be 1 service only. 
  only 1 service entry python file, and it would import other components , like
  - camera.py
  - joints.py
  - programs.py
  - status.py
  - map.py