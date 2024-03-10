# import cv2 
# import numpy as np


# markersX = 5           #X轴上标记的数量
# markersY = 7            #EY轴上标记的数量   本例生成5x7的棋盘
# markerLength = 50 #标记的长度，单位是像素
# markerSeparation = 10 #每个标记之间的间隔，单位像素
# margins = markerSeparation #标记与边界之间的间隔
# borderBits = 10 #标记的边界所占的bit位数
# showImage = True 


# width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins 
# height =markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins 


# # dictionary = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_250)
# dictionary = cv2.aruco.Dictionary_get( cv2.aruco.DICT_7X7_50)
# board = cv2.aruco.GridBoard_create(markersX, markersY, float(markerLength),float(markerSeparation), dictionary) 
# print(cv2.aruco_GridBoard.getGridSize(board))
# img= cv2.aruco_GridBoard.draw(board,(5000,6000), borderBits) 
# cv2.imwrite('frame.png', img)
import cv2 as cv
import numpy as np

dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_50)

num_markers_x = 4
num_markers_y = 4
marker_length = 0.04
marker_separation = 0.01

board = cv.aruco.GridBoard_create(
    num_markers_x,              # Number of markers in the X direction.
    num_markers_y,              # Number of markers in the Y direction.
    marker_length,              # Length of the marker side.
    marker_separation,          # Length of the marker separation.
    dictionary                  # The dictionary of the markers.
                                # (optional) Ids of all the markers (X*Y markers).
)

img = np.zeros((330,450), np.uint8) #4x4

# help(cv.aruco_GridBoard.draw)
# draw(outSize[, img[, marginSize[, borderBits]]]) -> img
img = board.draw( 
    img.shape,                  # Size of the output image in pixels.
    img,                        # Output image with the board
    0,                          # Minimum margins (in pixels) of the board in the output image
    1,                           # Width of the marker borders
)

extension = ".jpg"
cv.imwrite("aruco_" +
            str(num_markers_x) + "x" + str(num_markers_y) + "_" +
            str(int(marker_length*100)) + "cm_length_" +
            str(int(marker_separation*100)) + "cm_space"
            + extension, img)
cv.imshow("aruco_board", img)
cv.waitKey(0)