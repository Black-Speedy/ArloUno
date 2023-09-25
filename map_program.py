from robot import *
from landMark import *
# visuelize the tvecs as an image, draw a box for each tvec and save it in a file

img = np.zeros((720, 1280, 3), np.uint8)


results = lookBox(-1)


print(results)

for tvec in results:
    img = cv2.rectangle(img, (int(tvec[0][0]), int(tvec[0][1])), (int(tvec[0][0]) + 10, int(tvec[0][1]) + 10), (0, 255, 0), 2)

cv2.imwrite("test.png", img)

