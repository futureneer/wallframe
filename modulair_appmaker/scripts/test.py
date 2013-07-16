#!/usr/bin/env python

import os
import re
from PIL.Image import *

img_file_types = ['.jpg', '.png', '.jpeg', '.bmp']	

def find_image_files(file_path):
	files = os.listdir(file_path)

	file_regex = ''
	for file_ext in img_file_types:
		file_regex += '.*' + file_ext + '|'

	return [file_path + img for img in files if re.match(file_regex, img)]

def load_image_files(img_files):
	images = []
	for img in img_files:
		images.append(open(img).tostring("raw", "RGBX", 0, -1))
	return images

# a = find_image_files("/home/kel/modulair/modulair_appmaker/scripts/pics/")
# print a
b = load_image_files(find_image_files("/home/kel/modulair/modulair_appmaker/scripts/pics/"))
print b