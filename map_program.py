from robot import *
from landMark import *
# visuelize the tvecs as an image, draw a box for each tvec and save it in a file

img = np.zeros((720, 1280, 3), np.uint8)


results = lookBox(-1)

angle_error = 11

print(results)

for tvec in results:
    radians = tvec[0][0]
    degrees = np.degrees(radians)+ angle_error
    if(degrees < 0):
        x = int(640 - tvec[0][2]*100 * np.cos(radians + np.deg2rad(11)))
    else:
        x = int(640 + tvec[0][2]*100 * np.cos(radians + np.deg2rad(11)))
        
    y = int((720 - int(tvec[0][2]*100))/2)
    print(f"x value = {x} y value = {y-640}")
    img = cv2.circle(img, (x, y), 50, (255, 255, 255), 3)

cv2.imwrite("tests.png", img)
