import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from tkinter import *


class JoyButton:
    def __init__(self, parent_canvas, left_space, v_space, height, i):
        self.parent_canvas = parent_canvas
        l = left_space + 15
        t = v_space + (height + v_space)*i
        self.parent_canvas.create_text(left_space, t+height/2, text=str(i))
        
        self.circle_obj = self.parent_canvas.create_oval(l, 
                                                t, 
                                                l + height, 
                                                t + height, 
                                                width=2, fill="white")

    def update_value(self, value):
        if (value > 0):
            self.parent_canvas.itemconfigure(self.circle_obj, fill="#FF0000")
        else:
            self.parent_canvas.itemconfigure(self.circle_obj, fill="#FFFFFF")



class JoyAxis:
    def __init__(self, parent_canvas, left_space, v_space, height, width, i):
        self.parent_canvas = parent_canvas
        self.left_space = left_space
        self.v_space = v_space
        self.height = height
        self.width = width
        self.i = i

        l = left_space + 60
        t = v_space + (height + v_space)*i
        self.parent_canvas.create_text(left_space+50, t+height/2, text=str(i))
        
        self.fill_obj = self.parent_canvas.create_rectangle(l, 
                                                t, 
                                                l + width, 
                                                t + height, 
                                                width=0, fill="green")
        self.outline_obj = self.parent_canvas.create_rectangle(l, 
                                                t, 
                                                l + width, 
                                                t + height, 
                                                width=2, outline="black")

        self.val_txt = self.parent_canvas.create_text(left_space+60 + width + 30, t+height/2, text=str(i))

    def update_value(self, value):
        l = self.left_space + 60
        t = self.v_space + (self.height + self.v_space)*self.i

        ww = self.width * (value + 1)/2

        self.parent_canvas.coords(self.fill_obj, l, t, l+ww, t+self.height)
        self.parent_canvas.itemconfigure(self.val_txt, text=str(f'{value:.3f}'))



class JoyTester(Node):

    def __init__(self):
        super().__init__('test_joy')
        self.get_logger().info('Testing Joystick...')

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 5)
        self.subscription  # prevent unused variable warning

        self.buttons = []
        self.axes = []
        self.initialised = False

        self.tk = Tk()
        
        self.canvas = Canvas(self.tk, width=800, height=480)
        self.tk.title("Joystick Test")
        self.tk.geometry("800x480+0+0")
        self.canvas.pack(anchor='nw')

        self.tk.update()
        



    def joy_callback(self, joy_msg):


        left_space = 10
        height = 25
        width = 80
        v_space = 5

        # Handle first receive

        if not self.initialised:
            for i in range(len(joy_msg.buttons)):
                self.buttons.append(JoyButton(self.canvas, left_space, v_space, height, i))

            for i in range(len(joy_msg.axes)):
                self.axes.append(JoyAxis(self.canvas, left_space, v_space, height, width, i))

            self.initialised = True

        # Update Values

        for i in range(len(joy_msg.buttons)):
            self.buttons[i].update_value(joy_msg.buttons[i])

        for i in range(len(joy_msg.axes)):
            self.axes[i].update_value(joy_msg.axes[i])


        # Redraw
        self.tk.update()

        return




def main(args=None):
    
    rclpy.init(args=args)
    joy_tester = JoyTester()
    rclpy.spin(joy_tester)
    joy_tester.destroy_node()
    rclpy.shutdown()