from robot import *
from landMark import *
# visuelize the tvecs as an image, draw a box for each tvec and save it in a file

img = np.zeros((720, 1280, 3), np.uint8)


results = lookBox(-1)


print(results)

for tvec in results:
    img = cv2.circle(img, (int(tvec[0][2]*100), 100), 150, (255, 255, 255), 3)

cv2.imwrite("tests.png", img)

