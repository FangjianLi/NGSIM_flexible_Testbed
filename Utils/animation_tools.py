import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from get_avail_car_index import TIME_INTERVAL

class NGSIM_animation_generator:
    def __init__(self, car_interested, car_tested, car_list_environemntal, centerline, total_length,  lanes, dict_present_car, dict_X_Y_point):
        self.car_interested = car_interested
        self.car_tested = car_tested
        self.car_list_environmental = car_list_environemntal
        self.animation_list = []
        self.right_front_vehicle = dict_present_car['right_front_vehicle']
        self.right_rear_vehicle = dict_present_car['right_rear_vehicle']
        self.left_front_vehicle = dict_present_car['left_front_vehicle']
        self.left_rear_vehicle = dict_present_car['left_rear_vehicle']
        self.front_vehicle = dict_present_car['front_vehicle']
        self.X_f = dict_X_Y_point["X_f"]
        self.Y_f = dict_X_Y_point["Y_f"]
        self.X_l_f = dict_X_Y_point["X_l_f"]
        self.Y_l_f = dict_X_Y_point["Y_l_f"]
        self.X_l_r = dict_X_Y_point["X_l_r"]
        self.Y_l_r = dict_X_Y_point["Y_l_r"]
        self.X_r_f = dict_X_Y_point["X_r_f"]
        self.Y_r_f = dict_X_Y_point["Y_r_f"]
        self.X_r_b = dict_X_Y_point["X_r_b"]
        self.Y_r_b = dict_X_Y_point["Y_r_b"]
        self.near_point_X = dict_X_Y_point["near_point_X"]
        self.near_point_Y = dict_X_Y_point["near_point_Y"]

        self.total_length = total_length


        self.centerline = centerline
        self.lanes = lanes
        self.fig = plt.figure()
        self.fig.set_dpi(100)
        self.fig.set_size_inches(7, 7)
        self.generate_canvas()
        self.patch_the_cars()


    def patch_the_cars(self):
        self.patch_car_interested = plt.Rectangle((0, 0), self.car_interested.length, self.car_interested.width, -40, color='b')
        self.animation_list.append(self.patch_car_interested)
        self.ax.add_patch(self.patch_car_interested)
        self.patch_car_tested = plt.Rectangle((0, 0), self.car_tested.length, self.car_tested.width, -40, color='r')
        self.animation_list.append(self.patch_car_tested)
        self.ax.add_patch(self.patch_car_tested)
        self.patched_car_environmental = [self.get_patch(car_env) for car_env in self.car_list_environmental]

        for patched_car in self.patched_car_environmental:
            self.ax.add_patch(patched_car)
            self.animation_list.append(patched_car)




    def generate_canvas(self):

        self.ax = plt.axes(xlim=(1966360, 1966660), ylim=(570600, 570900))
        plt.plot(self.car_interested.X, self.car_interested.Y, '--c', linewidth=1)
        plt.ticklabel_format(style="plain", scilimits=(0, 0))
        plt.xlabel('x(m)')
        plt.ylabel('y(m)')


        plt.plot(self.lanes[:, 0], self.lanes[:, 1], 'lightsteelblue', label="lane1")  # boundary 1
        plt.plot(self.lanes[:, 2], self.lanes[:, 3], 'lightsteelblue', label="lane2")
        plt.plot(self.lanes[:, 4], self.lanes[:, 5], 'lightsteelblue', label="lane3")
        plt.plot(self.lanes[:, 6], self.lanes[:, 7], 'lightsteelblue', label="lane4")
        plt.plot(self.lanes[:, 8], self.lanes[:, 9], 'lightsteelblue', label="lane5")
        plt.plot(self.lanes[:, 10], self.lanes[:, 11], 'lightsteelblue', label="lane6")
        plt.plot(self.lanes[:, 12], self.lanes[:, 13], 'lightsteelblue', label="lane7")  # boundary_5
        plt.plot(self.lanes[:, 14], self.lanes[:, 15], 'lightsteelblue', label="lane8")  # boundary_4
        plt.plot(self.lanes[:, 16], self.lanes[:, 17], 'lightsteelblue', label="lane9")  # boundary_3
        plt.plot(self.lanes[:, 18], self.lanes[:, 19], 'lightsteelblue', label="lane10")  # boundary_2
        plt.plot(self.lanes[:, 20], self.lanes[:, 21], 'lightsteelblue', label="lane11")  # boundary_6
        plt.plot(self.lanes[:, 22], self.lanes[:, 23], 'lightsteelblue', label="lane12")
        centerline_1a = self.centerline[1]
        centerline_2a = self.centerline[2]
        centerline_3a = self.centerline[3]
        plt.plot(centerline_1a[:, 0], centerline_1a[:, 1], 'orange', ls=':')
        plt.plot(centerline_2a[:, 0], centerline_2a[:, 1], 'orange', ls=':')
        plt.plot(centerline_3a[:, 0], centerline_3a[:, 1], 'orange', ls=':')


        self.line1, = self.ax.plot([0, 1], [0, 1], 'r')
        self.animation_list.append(self.line1)
        self.line2, = self.ax.plot([0, 1], [0, 1], 'darkviolet')
        self.animation_list.append(self.line2)
        self.line3, = self.ax.plot([0, 1], [0, 1], 'lime')
        self.animation_list.append(self.line3)
        self.line4, = self.ax.plot([0, 1], [0, 1], 'k')
        self.animation_list.append(self.line4)
        self.line5, = self.ax.plot([0, 1], [0, 1], 'magenta')
        self.animation_list.append(self.line5)
        self.line6, = self.ax.plot([0, 1], [0, 1], 'k')
        self.animation_list.append(self.line6)

        self.line1.set_xdata([0, 1])
        self.line1.set_ydata([0, 1])
        self.line2.set_xdata([0, 1])
        self.line2.set_ydata([0, 1])
        self.line3.set_xdata([0, 1])
        self.line3.set_ydata([0, 1])
        self.line4.set_xdata([0, 1])
        self.line4.set_ydata([0, 1])
        self.line5.set_xdata([0, 1])
        self.line5.set_ydata([0, 1])
        self.line6.set_ydata([0, 1])

    def animate(self, i):
        if self.right_front_vehicle[i] == 0:
            self.line4.set_visible(False)
        else:
            self.line4.set_visible(True)

        if self.right_rear_vehicle[i] == 0:
            self.line5.set_visible(False)
        else:
            self.line5.set_visible(True)

        if self.left_front_vehicle[i] == 0:
            self.line2.set_visible(False)
        else:
            self.line2.set_visible(True)

        if self.left_rear_vehicle[i] == 0:
            self.line3.set_visible(False)
        else:
            self.line3.set_visible(True)

        if self.front_vehicle[i] == 0:
            self.line1.set_visible(False)
        else:
            self.line1.set_visible(True)



        self.patch_car_interested.xy = self.get_xy(self.car_interested, i)
        self.patch_car_tested.xy = self.get_xy(self.car_tested, i)

        self.ax.set_xlim(self.car_interested.X[i] - 30, self.car_interested.X[i] + 30)
        self.ax.set_ylim(self.car_interested.Y[i] - 30, self.car_interested.Y[i] + 30)

        self.line1.set_xdata([self.car_interested.X[i], self.X_f[i]])
        self.line1.set_ydata([self.car_interested.Y[i], self.Y_f[i]])
        self.line2.set_xdata([self.car_interested.X[i], self.X_l_f[i]])
        self.line2.set_ydata([self.car_interested.Y[i], self.Y_l_f[i]])
        self.line3.set_xdata([self.car_interested.X[i], self.X_l_r[i]])
        self.line3.set_ydata([self.car_interested.Y[i], self.Y_l_r[i]])
        self.line4.set_xdata([self.car_interested.X[i], self.X_r_f[i]])
        self.line4.set_ydata([self.car_interested.Y[i], self.Y_r_f[i]])
        self.line5.set_xdata([self.car_interested.X[i], self.X_r_b[i]])
        self.line5.set_ydata([self.car_interested.Y[i], self.Y_r_b[i]])

        self.line6.set_xdata([self.car_tested.X[i], self.near_point_X[i]])
        self.line6.set_ydata([self.car_tested.Y[i], self.near_point_Y[i]])

        # animation_list = [line1, line2, line3, line4, line5, patch_car_interested, patch_car_test]

        for patched_car, car_env in zip(self.patched_car_environmental, self.car_list_environmental):
            patched_car.xy = self.get_xy(car_env, i)


        return self.animation_list


    def show_animation(self):
        self.anim = animation.FuncAnimation(self.fig, self.animate, init_func=None, frames=np.arange(0, self.total_length, 1),
                                       interval=TIME_INTERVAL, blit=False)

        # Writer = animation.writers['ffmpeg']
        # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        writergif = animation.PillowWriter(fps=30)
        self.anim.save("experiment.gif", writer=writergif)

        plt.show()






    @staticmethod
    def get_xy(car_no, i):
        x1 = car_no.X[i] - 0.5 * car_no.width * np.sin(np.radians(40)) - car_no.length * np.cos(np.radians(40))
        y1 = car_no.Y[i] - 0.5 * car_no.width * np.cos(np.radians(40)) + car_no.length * np.sin(np.radians(40))
        return (x1, y1)


    @staticmethod
    def get_patch(car_no):
        patch_no = plt.Rectangle((0, 0), car_no.length, car_no.width, -40, color='g')
        return patch_no