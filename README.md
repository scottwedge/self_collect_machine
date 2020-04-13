# Self Collect Machine

This project is about to design a self collecting system which ease the customer 
collect thier purchased items during office or after office hours -- by scanning
the code (BAR/QR)

### Required Package:
1.  Python barcode decoding library -- Zbar
> sudo apt-get install libzbar0
> pip install pyzbar

2. UbiquityRobotics/raspicam_node
> https://github.com/UbiquityRobotics/raspicam_node

3. cv_camera
> http://wiki.ros.org/cv_camera

## Scripts
> USB Camera
>> 1. camera_preview.py
>> 2. camera_barcode_recognition.py
>> 3. camera_barcode_record.py

> Raspicam
>> 1. raspicam_preview.py
>> 2. raspicam_barcode_recognition.py
>> 3. raspicam_barcode_record.py

## Launch
1. camera_robot.launch
2. raspicam_robot.launch
