
# Description for behavioral apparatus assembly & 3D printing


## General info

very grateful to Seoul National University Hadong idea factory...

+ 3D printing machine: one of 3D WOX DP200, DP103 or 2X
+ layer: 0.15mm 
+ program: Autodesk Fusion, 3DWOX Desktop x64

❗ proper post-processing is required. (I mostly used sand paper and soldering iron)

💡 If you need .f3d file, please contact.

📃 **The PDF file includes assembly images and detailed instructions for a simple capacitive lickometer.**

## What you need

### Key devices
|name|for what?|ea.|note|
|:---:|:---:|:---:|:---:|
|arduino mega 2650|main controller|1||
|linear actuator|liquid delivery|4|100mm range, 15mm/s model. I bought it from AliExpress, so I don't know exact model name|
|RB-35GM W/EC|servo motor with encoder for spout selection|1|https://www.dnjmall.com/goods/goods_list.php?cateCd=009|
|LHDA1231115H|solenoid valve for liquid control|10|from The LEE company|
|GJ-5FA-L [SY-SSR01]|DC to DC SSR for liquid control|10||
|L293D(DIP)|motor driver for linear actuator|3||

In addition, you will need enough amount of blunt needle(liquid spout for mouse), silicon tubing, arduino 4*4 keypad (for convenience), 2M ohmn resistors(for manual lickometer), and 12V DC power supply.


## Assembly
I mostly used **blu tack** for overall assembly.


- **Essential folder** contains essnetial parts for spout selection and delivery
  - There are thin/thick, long/short versions. (schematics are same, only length is different.)
  - I used thin and short versions, but newly made long and thick version to add precision.
  - It is okay to use thin and short versions, but if there occurs some errors, please consider long and thick versions. 


- **Accessory folder** contains liquid delivery tower, motor supporter, etc.