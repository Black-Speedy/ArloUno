from robot import *
from landMark import *
# visuelize the tvecs as an image, draw a box for each tvec and save it in a file

img = np.zeros((720, 720, 3), np.uint8)



angle_error = 11

cv2.imwrite("tests.png", img)

# open a window


results, ids = lookBox(-1)


for i in range(0, len(results)):
    radians = results[i][0][0]
    degrees = np.degrees(radians) + angle_error
    if (degrees < 0):
        x = int(360 - results[i][0][2]*100 * np.sin(radians + np.deg2rad(angle_error)))
    else:
        x = int(360 + results[i][0][2]*100 *
                np.sin(radians + np.deg2rad(angle_error)))

    print(
        f"offset = {results[i][0][2]*100 * np.sin(radians + np.deg2rad(angle_error))}")
    print(f"offset w/o = {results[i][0][2]*100 * np.sin(radians)}")
    y = int((720)/2) - int(results[i][0][2]*100)
    # print(f"x value = {x} y value = {y}")
    print(f"radians: {radians} degrees: {degrees}")
    img = cv2.circle(img, (x, y), 35, (255, 255, 255), 3)
    # write the id of the box
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, str(ids[i]), (x, y),
                font, 1, (255, 255, 255), 2, cv2.LINE_AA)

cv2.imwrite("tests.png", img)


        
