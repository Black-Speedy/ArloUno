import cv2
import particle
import numpy as np
import time
from timeit import default_timer as timer
import sys
import random_numbers

# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True # Whether or not we are running on the Arlo robot

def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    #if 'arlo' in sys.platform.lower():
    #    onRobot = True
    
    return onRobot

if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../../../../../../../../../../../../../../../..~/Arlo/ArloUno/robot.py")
    print(sys.path.append("~/Arlo/ArloUno/robot.py"))

    #showGUI = False
try:
    #import robot
    onRobot = True
    showGUI = False
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False

# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [8, 5]
landmarks = {
    8: (0.0, 0.0),  # Coordinates for landmark 1
    5: (300.0, 0.0)  # Cordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks


def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = tuple(map(int, jet(particle.getWeight() / max_weight)))
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
                                 ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)



def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles





# Main program #
def Localize(myCam):
    try:
        if showGUI:
            # Open windows
            WIN_RF1 = "Robot view"
            cv2.namedWindow(WIN_RF1)
            cv2.moveWindow(WIN_RF1, 50, 50)

            WIN_World = "World view"
            cv2.namedWindow(WIN_World)
            cv2.moveWindow(WIN_World, 500, 50)

        # Initialize particles
        num_particles = 1000
        particles = initialize_particles(num_particles)

        startTime = timer()

        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

        # Driving parameters
        velocity = 0.0 # cm/sec
        angular_velocity = 0.0 # radians/sec

        # Initialize the robot (XXX: You do this)

        # Allocate space for world map
        world = np.zeros((500,500,3), dtype=np.uint8)

        # Draw map
        draw_world(est_pose, particles, world)

        print("Opening and initializing camera")
        """ if camera.isRunningOnArlo():
            cam = myCam
        else:
            cam = camera.Camera(0, 'macbookpro', useCaptureThread = True) #changed macbookpro to arlo """
        cam = myCam

        while True:
            # Move the robot according to user input (only for testing)
            action = cv2.waitKey(10)
            if action == ord('q'): # Quit
                break
            
            if not isRunningOnArlo():
                if action == ord('w'): # Forward
                    velocity += 1.0
                elif action == ord('x'): # Backwards
                    velocity -= 1.0
                elif action == ord('s'): # Stop
                    velocity = 0.0
                    angular_velocity = 0.0
                elif action == ord('a'): # Left
                    angular_velocity += 0.1
                elif action == ord('d'): # Right
                    angular_velocity -= 0.1


            # Use motor controls to update particles
            # XXX: Make the robot drive
            # XXX: You do this

            particle.add_uncertainty(particles, 0.6, 0.1)

            for p in particles:
                delta_x = np.cos(p.getTheta()) * velocity
                delta_y = np.sin(p.getTheta()) * velocity
                particle.move_particle(p, delta_x, delta_y, angular_velocity)



            # Fetch next frame
            colour = cam.get_next_frame()
            
            
            # Detect objects
            objectIDs, dists, angles = cam.detect_aruco_objects(colour)
            if not isinstance(objectIDs, type(None)):
                # List detected objects
                # XXX: Do something for each detected object - remember, the same ID may appear several times
                unique_object_ids = set()
                detected_objects = [] 
        
                def N(sigma, error): #,stderror         
                    a = (1/(np.sqrt(2 * np.pi * sigma**2))) 
                    b = np.exp(-(error**2)/(2*sigma**2))
                    #print(f"a: {a}, b: {b}, a*b: {a*b}")
                    result = a * b
                    return result
                    #return random_numbers.randn(error, 0.1)
                    #return np.random.normal(x,0, 1)
                
                for i in range(len(objectIDs)):
                    object_id = objectIDs[i]
                    if object_id not in unique_object_ids:
                        unique_object_ids.add(object_id)
                        distance = dists[i]
                        angle = angles[i]
                        detected_objects.append((object_id, distance, angle))
                        #print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
    #distance                
                dist_weight_list = []
                for p in particles:
                    x_i, y_i = p.getX(), p.getY()
                    dist_error = 1

                    for object in detected_objects:
                        # check if id exists in landmarks:
                        if not landmarks.__contains__(object[0]):
                            continue
                        lx = landmarks[(object[0])][0]
                        ly = landmarks[object[0]][1]
                        unique_particle_distance = np.sqrt((lx - x_i) ** 2 + (ly - y_i) ** 2) #d_i
                        
                        dist_error *= N(20, (np.abs(object[1] - (unique_particle_distance)))) #(d_m - d_i)^2
                
                    #stderror_distance = np.std(List_dist, ddof=1) #/ np.sqrt(np.size(List.dist))
                    
                    #dist_weight = (dist_error,stderror_distance)
                    dist_weight_list.append(dist_error)
    #angle                    
                angle_weight_list = []
                for p in particles:
                    x_i, y_i, theta_i = p.getX(), p.getY(), p.getTheta()
                    angle_error = 1
                    for object in detected_objects: 
                        if not landmarks.__contains__(object[0]):
                            continue
                        lx = landmarks[(object[0])][0]
                        ly = landmarks[object[0]][1]
                        delta_x = lx - x_i
                        delta_y = ly - y_i                    
                        distance = np.sqrt(delta_x**2 + delta_y**2)
                        
                        e_theta = np.array([np.cos(theta_i), np.sin(theta_i)])                    
                        e_l = np.array([(delta_x) / distance, delta_y / distance])
                        e_hat = np.array([-np.sin(theta_i), np.cos(theta_i)])
                        phi = np.sign(np.dot(e_l,e_hat))*np.arccos(np.dot(e_l, e_theta))

                        angle_error *= N(0.35, np.abs(object[2] - (phi))) #(\phi_m - \phi_i)
            
                    #angle_weight = (angle_error * 20,std_error_angles)#,std_error_angles
                
                    angle_weight_list.append(angle_error * 20)
                
               
                
                particle_weight_list = np.multiply(dist_weight_list, angle_weight_list)
                
                
                #print(f"before: {particles[0].getWeight()}")
                for p in range(len(particles)):
                    particles[p].setWeight(particle_weight_list[p])
                
                sum = np.sum(particle_weight_list)

                for p in particles:
                    p.setWeight(p.getWeight() / sum)

                chosenParticles = []
                for i in range(num_particles):
                    r = np.random.random()
                    for p in particles:
                        #print(f"r: {r}")
                        #print(p.getWeight())
                        r -= p.getWeight()
                        if r <= 0:
                            #print(f"weight right now: {p.getWeight()}")
                            chosenParticles.append(particle.Particle(p.getX(), p.getY(), p.getTheta(), p.getWeight()))
                            break

                
                chosenParticles = sorted(chosenParticles, key=lambda particle: particle.getWeight())
                # Remove the worst 5%
                chosenParticles = chosenParticles[int(0.05 * len(chosenParticles)):]

                x_values = np.array([p.getX() for p in chosenParticles])
                y_values = np.array([p.getY() for p in chosenParticles])
                theta_values = np.array([p.getTheta() for p in chosenParticles])

                x_covar = np.cov(x_values)
                y_covar = np.cov(y_values)
                theta_covar = np.cov(theta_values)

                print(f"x_covar: {x_covar}, y_covar: {y_covar}, theta_covar: {theta_covar}")

                if (x_covar + y_covar < 10 and theta_covar < 0.015 and startTime + 3 < timer()):
                    # found the robot
                    return particle.estimate_pose(particles)
                
                newParticles = (initialize_particles(num_particles - len(chosenParticles)))

                # combine arrrays
                particles = np.concatenate([newParticles, chosenParticles], axis=0)
                # Draw detected objects…
                cam.draw_aruco_objects(colour)
            else:
                # No observation - reset weights to uniform distribution
                for p in particles:
                    p.setWeight(1.0/num_particles)

        
            est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

            #print(f"est post: {est_pose.getX()}, {est_pose.getY()}, {est_pose.getTheta()}")

            if showGUI:
                # Draw map
                draw_world(est_pose, particles, world)
        
                # Show frame
                cv2.imshow(WIN_RF1, colour)

                # Show world
                cv2.imshow(WIN_World, world)
        
    
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()

        # Clean-up capture thread
        cam.terminateCaptureThread()

