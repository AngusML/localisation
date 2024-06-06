// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
const char apriltag_py_type_docstring[] =
"AprilTag detector\n"
"\n"
"SYNOPSIS\n"
"\n"
"    import cv2\n"
"    import numpy as np\n"
"    from apriltag import apriltag\n"
"\n"
"    imagepath = '/tmp/tst.jpg'\n"
"    image     = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)\n"
"    detector = apriltag(\"tag36h11\")\n"
"\n"
"    detections = detector.detect(image)\n"
"\n"
"    print(\"Saw tags {} at\\n{}\". \\\n"
"          format([d['id']     for d in detections],\n"
"                 np.array([d['center'] for d in detections])))\n"
"\n"
"    ----> Saw tags [3, 5, 7, 8, 10, 10, 14] at\n"
"          [[582.42911184 172.90587335]\n"
"           [703.32149701 271.50587376]\n"
"           [288.1462089  227.01502779]\n"
"           [463.63679264 227.91185418]\n"
"           [ 93.88534443 241.61109765]\n"
"           [121.94062798 237.97010936]\n"
"           [356.46940849 260.20169159]]\n"
"\n"
"DESCRIPTION\n"
"\n"
"The AprilTags visual fiducial system project page is here:\n"
"https://april.eecs.umich.edu/software/apriltag\n"
"\n"
"This is a Python class to provide AprilTags functionality in Python programs. To\n"
"run the detector you\n"
"\n"
"1. Construct an object of type apriltag.apriltag()\n"
"\n"
"2. Invoke the detect() method on this object\n"
"\n"
"The constructor takes a number of arguments:\n"
"\n"
"- family: a string for the tag type we're detecting. This argument is required.\n"
"  If an invalid string is given, the known list of tag families will be\n"
"  reported. At the time of this writing the known families are:\n"
"\n"
"  - \"tag36h11\"\n"
"  - \"tag25h9\"\n"
"  - \"tag16h5\"\n"
"  - \"tagCircle21h7\"\n"
"  - \"tagCircle49h12\"\n"
"  - \"tagStandard41h12\"\n"
"  - \"tagStandard52h13\"\n"
"  - \"tagCustom48h12\"\n"
"\n"
"All the other arguments are optional:\n"
"\n"
"- threads: how many threads the detector should use. Default is 1\n"
"\n"
"- maxhamming: max number of corrected bits. Larger values guzzle RAM. Default is\n"
"  1\n"
"\n"
"- decimate: detection of quads can be done on a lower-resolution image,\n"
"  improving speed at a cost of pose accuracy and a slight decrease in detection\n"
"  rate. Decoding the binary payload is still done at full resolution. Default is 1.0\n"
"\n"
"- blur: What Gaussian blur should be applied to the segmented image (used for\n"
"  quad detection?) Parameter is the standard deviation in pixels. Very noisy\n"
"  images benefit from non-zero values (e.g. 0.8). Default is 0.0\n"
"\n"
"- refine_edges: When non-zero, the edges of the each quad are adjusted to \"snap\n"
"  to\" strong gradients nearby. This is useful when decimation is employed, as it\n"
"  can increase the quality of the initial quad estimate substantially. Generally\n"
"  recommended to be on. Very computationally inexpensive. Option is ignored if\n"
"  decimate == 1. Default is True\n"
"\n"
"- debug: When non-zero, write a variety of debugging images to the current\n"
"  working directory at various stages through the detection process. (Somewhat\n"
"  slow). Default is False\n"
"\n"
"The detect() method takes a single argument: an image array\n"
"";
