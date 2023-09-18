import robot
r = robot.Robot()

ipt = input("Choose a program: test, eight, avoid, sensor\n")


if ipt == "test":
    while(True):
        dir = input()

        if (dir == "a"):
            turn(r, "left", 30, 196)
        elif (dir == "d"):
            turn(r, "right", 30, 196)
        elif (dir == "w"):
            straight64(r, 1.14)
        
        r.stop()
        
elif ipt == "eight":    
    amount = input("How many times do you want to do the eight? ")
    for i in range(0, int(amount)):
        smoothTurn(r, "right")  
        smoothTurn(r, "left")  


elif ipt == "avoid":
    turn_time = input("Input turn time: ")
    dist = int(input("Input distance: "))
    distSide = int(input("Input side distance: "))
    isDriving = True
    turning = False
    turnBackTime = 0
    straight64(r)
    turnTimer = time.perf_counter()
    start = time.perf_counter()
    while (isDriving):  # or some other form of loop
        if (time.perf_counter() - start > 60):
            print (r.stop())
            isDriving = False

        if (turning):
            if time.perf_counter() - turnTimer >  float(turn_time):
                turning = False
                straight64(r)            
        elif (r.read_left_ping_sensor() < distSide) :
            turning = True
            turnTimer = time.perf_counter()
            turn(r, "right")
            print("turning right")
        elif (r.read_right_ping_sensor() < distSide) :
            turning = True
            turnTimer = time.perf_counter()
            turn(r, "left")
            print("turning left")
        elif (r.read_front_ping_sensor() < dist):
            turning = True
            turnTimer = time.perf_counter()
            #choose a random direction from left and right
            random_direction = np.random.choice(["left", "right"], p=[0.5, 0.5])
            turn(r, random_direction)
            print("turning random")


elif ipt == "sensor":
    while(True):
        input("press enter: ")
        print("Distance: " + str(r.read_front_ping_sensor() / 10) + " cm")

elif ipt == "auto sensor":
    lst = []
    dist = float(input("Input distance: "))
    for j in range(0, 5):
        if (j != 0):
            straight64(r)
            sleep(dist)
            r.stop()
            sleep(5)
        for i in range(0, 5):
            lst.append(r.read_front_ping_sensor())
            print(lst[j * 5 + i])

    print(lst)
    
elif ipt == "camera":
    picam2 = Picamera2()
    picam2.start_and_capture_file("test.jpg")
