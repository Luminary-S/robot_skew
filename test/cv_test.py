#!/usr/bin/python
# -*- coding: utf-8 -*-

# 下面的程序会按灰度方式加载图片，显示，如果你敲了's'就会保存图片并退出，或者你如果敲了ESC键就会不保存直接退出
import numpy as np
import cv2
img = cv2.imread('/home/sgl/catkin_new/src/demo_ur_skew/test/th.jpeg',0)
cv2.imshow('lala',img)
key = cv2.waitKey(0)
if key == 27:
    cv2.destroyAllWindows()
elif key == ord('s'):  #它以一个字符（长度为1的字符串）作为参数，返回对应的 ASCII 数值
    cv2.imwrite('gray_touxiang.jpg',img)
    cv2.destroyAllWindows()





