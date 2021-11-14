# ECE470 Codligrapher
This github project intended to implement a computer-vision based automatical drawing robot for ECE470 of the U of I at Urbana Champaign.
## Update 1 
10/16/2021 <br />
Including the basic moving functions or arm and basic sensors test with gazebo simulator <br />
contributed by Haochen Zhang and Yuxuan Guo <br />
##Update 2
11/13/2021 <br />
## Update 2
### What We have done
1. We changed simulation tool from gazebo to coppeliasim <br />
2. We use the python remote api from coppeliasim to implement the most functionality of the drawing robot. <br />
3. We build a simple demo scene for the update 2 demo <br />
4. We also make some changes to the threaded child script of the UR3 robot and the paint gun <br />
5. We revise the inverse kinematics from lab4 due to the minor difference between real world UR3 and model of UR3 in coppeliasim <br />
6. We add image into the scene by attaching the image as texture to a cuboid with tiny height <br />
7. We implemented the image processing to the image detected by vision sensor with opencv <br />
8. We successfully let our drawing robot draw out the contour of the image we inserted into the scene on flat ground <br />
### What We will improve
1. Implementing drawing on inclined surface 
2. Implementing some writing function 
3. Implementing basic multi-class classification for the image we inserted 
## Contributed by Haochen Zhang and Yuxuan Guo <br />
