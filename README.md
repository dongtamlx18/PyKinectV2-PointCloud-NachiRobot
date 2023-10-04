# PyKinectV2-PointCloud-NachiRobot
Image Processing-based Control of a 6-DOF Robotic Arm for Assemblig Tilted Objects on a ConveyorBelt. Using OpenCV and Open3D library to process in this project.
-------------------------- Python version 3.6.3 -----------------------------
<h2 style="font-size: 20px;">Description</h2>
I apply 2D and 3D image processing in this project. My goal is to process the coordinates (x, y, z, roll, pitch, yaw) of an object in space, and then control the robotic arm to perform tasks such as grasping the object and fitting it into a designated position. The conveyor belt speed in this project is 10mm/s, and the assembly accuracy of the radius is 2mm. This project is undertaken to fulfill the requirements of my graduation thesis, and I sincerely hope that those who review its content can gain some insights. Thank you.
<h2 style="font-size: 20px;">Take notice</h2>
Carefully read the 'requirements.txt' file to install the correct versions of the libraries used in this project.
<h2 style="font-size: 20px;">Instruction</h2>
*_Open3D.py_*

Connect your laptop to KinectV2 and execute the code. You can modify the file name, and the file will be saved with a .ply extension. Here is a picture of the PointCloud process captured by KinectV2.
![image](https://github.com/dongtamlx18/PyKinectV2-PointCloud-NachiRobot/assets/44941558/75b1515a-aedf-444e-b09a-4a3ac52f5c42)
*_Open3D.py_*
Choose some models from the "3D_Models" folder and run them to familiarize yourself with the code. Here are pictures show of how code process
![image](https://github.com/dongtamlx18/PyKinectV2-PointCloud-NachiRobot/assets/44941558/f14d25a6-a50b-4bf8-9356-7a5e4be94ae6)
![image](https://github.com/dongtamlx18/PyKinectV2-PointCloud-NachiRobot/assets/44941558/4e0ebd0e-2e7f-4fc5-ad52-9e8de25e52fa)
