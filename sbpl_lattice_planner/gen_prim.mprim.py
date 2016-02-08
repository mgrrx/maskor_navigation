#! /usr/bin/env python
__author__ = 'broecker'
import matplotlib.pyplot as pyplots
import math
#ackermann steering http://www.robotc.net/wikiarchive/Tutorials/Arduino_Projects/Additional_Info/Turning_Calculations

resolution = 0.025
number_of_angles = 2
intermediate_poses = 10

axis_width = 0.2670
wheel_base = 0.36

#straight actions in cm
straight_actions = [1, 10]
weights_straight = [1, 1]
#turn_radius = []
#steering angles in degrees
#steering_angles = [ -16.30, 16.3]
steering_angles = [-15.8, 15.8]
#steering_angles = []
weights_angles = [5, 5, 5, 5]
#curve_length = [9.0820364313, 9.0820364313, 10, 10, 10, 10]
curve_length = [10.1439806581, 10.1439806581]
steering_angles_backwards = []

#test stuff for angle generation
min_test_angle = 0.0
max_test_angle = 30.0
step_size_angle = 0.1
max_x_value = 12
max_y_value = 6
test_epsilon = 0.0005


def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def find_valid_radius():
    for x in range(1, max_x_value + 1):
        for y in range(1, max_y_value + 1):
            angle = 0.1
            while angle < max_test_angle:
                radius = math.fabs(get_turning_radius(angle))
                if radius - test_epsilon <= distance(x*resolution, y*resolution, 0, radius) <= radius + test_epsilon:
                    angle_seg = math.atan2(y, x)
                    circumference = math.pi * radius * 2
                    factor = angle_seg / (math.pi)
                    length = (circumference * factor) / resolution
                    print "angle : " + str(angle) + " radius : " + str(radius) + " x: " + str(x*resolution) + " y: " + str(y*resolution) + " length: " + str(length) + " angle_seg: " + str(factor)
                angle += step_size_angle


def get_circle_seg_points(steering_angle, length, number_of_points, forward=True):
    x = []
    y = []
    angle = []
    radius = math.fabs(get_turning_radius(steering_angle))
    circumference = math.pi * radius * 2
    end_angle = ((length ) / (circumference/resolution)) * 360
    tmp_angle = (length ) / (circumference/resolution)

    centre_x = 0
    #circle counterclockwise, angle 0 is on the right side of the circle
    # which mean angle smaller 0 = clockwise
    if forward:
        #leftcurve
        if steering_angle < 0:
            centre_y = radius
            start_angle = -90
            end_angle = end_angle - 90
        #rightcurve
        else:
            centre_y = -radius
            start_angle = 90
            end_angle = 90 - end_angle
    else:
        if steering_angle < 0:
            centre_y = radius
            start_angle = -90
            end_angle = -end_angle - 90
        #rightcurve
        else:
            centre_y = -radius
            start_angle = 90
            end_angle = 90 + end_angle

    one_step = (end_angle - start_angle)/ (number_of_points - 1)
    current_angle = start_angle

    for i in (range(0, number_of_points)):
        rayStart = current_angle * math.pi/180.0
        x.append(centre_x + radius*math.cos(rayStart))
        y.append(centre_y + radius*math.sin(rayStart))
        #print "x: " + str(centre_x + radius*math.cos(rayStart)) + " y: " + str(centre_y + radius*math.sin(rayStart))
        if centre_y > 0:
            angle.append(rayStart + math.pi/2)
        else:
            angle.append(rayStart - math.pi/2)

        current_angle += one_step

    return x, y, angle

def get_line_seg_points(length, number_of_points):
    x = []
    y = []
    angle = []
    c = 0.0
    step = ((length) * resolution)/float(number_of_points - 1)
    for i in range(0, number_of_points):
        x.append(step * i)
        #y.append(math.cos(((0 / float(number_of_angles)) * 2*math.pi)+ math.pi) * factor)
        y.append(0)
        angle.append(0)
        #c += step

    return x, y ,angle

def rotate_points(x, y, angle, rotate_angle):
    rotate_angle *= math.pi / 180.0
    new_x = []
    new_y = []
    for i in range(0, len(x)):
        new_x.append((x[i] * math.cos(rotate_angle) - y[i] * math.sin(rotate_angle)))
        new_y.append((x[i] * math.sin(rotate_angle) + y[i] * math.cos(rotate_angle)))
        angle[i] += rotate_angle

    return new_x, new_y, angle


def plot_orientation(x, y, angle):
    for i in range(0, len(x)):
        pyplots.plot(x[i], y[i], marker=(3, 0, (angle[i]*180) /  math.pi) ,linestyle='')

def get_turning_radius(steering_angle):
    return (wheel_base/ math.tan((steering_angle/180) * math.pi))


def generate_motion_file(file):
    prims_per_angle = (len(straight_actions) + len(steering_angles) + len(steering_angles_backwards))
    total_number_primitives = (len(straight_actions) + len(steering_angles) + len(steering_angles_backwards)) * number_of_angles
    x = []
    y = []
    angle = []
    prim_writer = PrimSerialiser(file, total_number_primitives, resolution, number_of_angles, intermediate_poses)


    #pyplots.ylim(-8,8)
    primID = 0
    for i in range(0, number_of_angles):
        for j in range(0, len(steering_angles)):
            x, y, angle = get_circle_seg_points(steering_angles[j], curve_length[j], 10)
            x, y, angle = rotate_points(x, y, angle, 360.0 / number_of_angles * i)
            prim_writer.write_prim(i, x, y, angle, weights_angles[j], primID)
            primID = (primID + 1) % prims_per_angle
            plot_orientation(x, y, angle)

        for j in range(0, len(steering_angles_backwards)):
            x, y, angle = get_circle_seg_points(steering_angles_backwards[j], curve_length[j], 10, False)
            x, y, angle = rotate_points(x, y, angle, 360.0 / number_of_angles * i)
            prim_writer.write_prim(i, x, y, angle, 1)
            primID = (primID + 1) % prims_per_angle
            plot_orientation(x, y, angle)

        for j in range(0, len(straight_actions)):
            x, y, angle = get_line_seg_points(straight_actions[j], 10)
            x, y, angle = rotate_points(x, y, angle, 360.0 / number_of_angles * i)
            prim_writer.write_prim(i, x, y, angle, weights_straight[j], primID)
            primID = (primID + 1) % prims_per_angle
            plot_orientation(x, y, angle)

        #x, y, angle = rotate_points(x,y,angle, 120)
        #plot_orientation(x, y, angle)

    prim_writer.close()
    pyplots.show()


class PrimSerialiser:

    def __init__(self, fileName, total_number_primitives, resolution = 0.025, number_of_angles = 4, intermediate_poses =10):
        self.intermediate_poses = intermediate_poses
        self.number_of_angles = number_of_angles
        self.resolution = resolution
        self.total_number_primitives = total_number_primitives
        self.file = open(fileName, "w+")
        self.prim_id = 0
        self.write_header()


    def write_header(self):
        self.file.write("resolution_m: " + str(self.resolution) + "\n")
        self.file.write("numberofangles: " + str(self.number_of_angles) + "\n")
        self.file.write("totalnumberofprimitives: " + str(self.total_number_primitives) + "\n")

    #int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS)
    #{
    #double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
    #return (int)(normalizeAngle(fTheta + thetaBinSize / 2.0) / (2.0 * PI_CONST) * (NUMOFANGLEVALS));
    #}

    def cont_theta_2disc(self, theta):
        theta_bin_size = 2.0 * math.pi / number_of_angles
        return int((self.normlize_angle(theta + theta_bin_size / 2.0)) / (2.0 * math.pi) * number_of_angles)

    def normlize_angle(self, angle):
        retangle = angle
        if math.fabs(retangle) > 2 * math.pi:
            retangle -= (int(retangle/ (2 * math.pi))) * 2 * math.pi
            #retangle = retangle - ((int)(retangle / (2 * PI_CONST))) * 2 * PI_CONST;

        if retangle < 0:
            retangle += 2 * math.pi

        if retangle < 0 or retangle > 2 * math.pi:
            print 'ERROR: after normalization of angle'

        return retangle

    #input angle should be in radians
    #counterclockwise is positive
    #output is an angle in the range of from 0 to 2*PI
    #double normalizeAngle(double angle)
    #{
    #double retangle = angle;

    #//get to the range from -2PI, 2PI
    #if (fabs(retangle) > 2 * PI_CONST) retangle = retangle - ((int)(retangle / (2 * PI_CONST))) * 2 * PI_CONST;

    #//get to the range 0, 2PI
    #if (retangle < 0) retangle += 2 * PI_CONST;

    #if (retangle < 0 || retangle > 2 * PI_CONST) {
     #   SBPL_ERROR("ERROR: after normalization of angle=%f we get angle=%f\n", angle, retangle);
    #}

    #return retangle;}

    #define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))
    def cont_xy_disc(self, x, cell_size):
        if x >= 0:
            return int(x / cell_size)
        else:
            return int(x / cell_size) - 1

    def disc_xy_cont(self, x, cell):#   ISCXY2CONT(X, CELLSIZE) ((X)*(CELLSIZE) + (CELLSIZE)/2.0)
        return (x * cell + cell /2.0)

    def pos_to_grid_vector(self, x, y, angle):
        src_x = self.disc_xy_cont(0, resolution)
        src_y = self.disc_xy_cont(0, resolution)
       # print src_x, src_y
        return self.cont_xy_disc(x + src_x, self.resolution), self.cont_xy_disc(y +src_y, self.resolution), self.cont_theta_2disc(angle)

    def write_prim(self, start_angle, x, y, angle, weight, prim_id):
        last_element = len(x) - 1
        self.file.write("primID: " + str(prim_id) + "\n")
        self.file.write("startangle_c: " + str(start_angle) + "\n")
        x_pos, y_pos, angle_pos = self.pos_to_grid_vector(round(x[last_element], 4), round(y[last_element], 4), round(angle[last_element], 4))
        self.file.write("endpose_c: " + str(x_pos) + ' ' + str(y_pos) + ' ' + str(angle_pos) + "\n")
        self.file.write("additionalactioncostmult: " + str(weight) + "\n")
        self.file.write("intermediateposes: " + str(self.intermediate_poses) + "\n")
        for i in range(0, len(x)):
            self.file.write(str(round(x[i], 4)) + ' ' + str(round(y[i], 4)) + ' ' + str(round(angle[i], 4)) + "\n")



    def close(self):
        self.file.close()



if __name__ == '__main__':
    find_valid_radius()
    generate_motion_file("test.mprim")

