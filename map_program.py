from robot import *
from landMark import *
# visuelize the tvecs as an image, draw a box for each tvec and save it in a file

img = np.zeros((720, 1280, 3), np.uint8)


results, ids = lookBox(-1)

angle_error = 11

print(results)

for i in range(0, len(results)):
    radians = results[i][0][0]
    degrees = np.degrees(radians)+ angle_error
    if(degrees < 0):
        x = int(640 - results[i][0][2]*100 * np.cos(radians + np.deg2rad(11)))
    else:
        x = int(640 + results[i][0][2]*100 * np.cos(radians + np.deg2rad(11)))
        
    y = int((720 - int(results[i][0][2]*100))/2)
    print(f"x value = {x} y value = {y-640}")
    img = cv2.circle(img, (x, y), 35, (255, 255, 255), 3)
    # write the id of the box
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, str(ids[i]), (x, y),
                font, 1, (255, 255, 255), 2, cv2.LINE_AA)

cv2.imwrite("tests.png", img)

