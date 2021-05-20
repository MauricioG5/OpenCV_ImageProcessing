# OpenCV_ImageProcessing    **WIP**

This project is intended to detect the face and eyes of a person getting the images from PC camera and sending it through a ROS topic. OpenCV 2.4.9 is the library that will be used for doing it.

The following templates, contained on *src* folder, are the base for this project:

1. opencv_template_node: This is the base template wich serves as an example of how OpenCV works with images, it contains the basic processing from taking the images by subscribing to camera node, converting them from ROS format to OpenCV format
, drawing an example circle, showing the processed image and publishing it trough a ROS topic, for wich it is necessary to convert to ROS format again.
2. opencv_template_node_hh: This template is pretty the same than the previous one, but implementing some defines and improving the original documentation.
3. opencv_grayImage_hh: This template shows how to convert an RGB color image to a gray scale one, for wich it uses cvtcolor function.
4. opencv_smoothingImages_hh: The objective of this template is to reduce the size of the image, it helps to reduce the time needed to process the image, due to a reduction on its resolution and it serves to avoid false positives on face and eyes detection.
5. Opencv_ChangeContrast: On this template, parameters alpha and beta are modified, changing the contrast of the image this way, this is useful for the program can recognize shapes more easily.
6. opencv_houghCircleTransform_hh: This template mix all previous templates for a greater implementation. With fully processed image (in the order they were described below) the program uses houghcircle method to find circles on the processed image, with this information, it draws a cirle on the original image.

**For executing this templates, this line must be executed from the console:**

> $ rosrun <folder_name>  <template_name>.cpp

# Author

- Mauricio Gómez Menjura
