import tkinter as tk
import const
from robot_message import RobotMessage



class PathVisualization:
    def __init__(self, robot: RobotMessage):
        self.robot = robot

        self.root = tk.Tk()

        self.main_frame = tk.Frame(self.root)
        self.main_frame.grid(row=0, column=0)

        self.path_entry = tk.Entry(self.main_frame)
        self.path_entry.grid(row=0, column=0)
        self.path_entry.insert(0, '[]')
        self.set_path_btn = tk.Button(self.main_frame, text='Set Path', command=self.set_path)
        self.set_path_btn.grid(row=1, column=0)

        self.make_message = tk.Button(self.main_frame, text='make message', command=self.make_message)
        self.make_message.grid(row=2, column=0)

        self.canvas = tk.Canvas(height=const.CANVAS_HEIGHT, width=const.CANVAS_WIDTH, bg='black')
        self.canvas.grid(row=0, column=1)
        # global x axes
        self.canvas.create_rectangle(0,
                                     0,
                                     0 + const.X_AXES_LENGTH + 50,
                                     0 + const.X_AXES_WIDTH,
                                     fill='red'
                                     ),
        # global y axes
        self.canvas.create_rectangle(0,
                                     0,
                                     0 + const.Y_AXES_WIDTH,
                                     0 + const.Y_AXES_LENGTH + 50,
                                     fill='red'
                                     ),
        scaled_x_robot, scaled_y_robot = self.scaled_point(self.robot.robot_coordinates[0], self.robot.robot_coordinates[1])
        self.virtual_robot_info = {
            'x_coor': self.robot.robot_coordinates[0],
            'y_coor': self.robot.robot_coordinates[1],
            'virtual_robot': self.canvas.create_rectangle(scaled_x_robot,
                                                          scaled_y_robot,
                                                          scaled_x_robot + const.ROBOT_WIDTH,
                                                          scaled_y_robot + const.ROBOT_HEIGHT,
                                                          fill='blue'
                                                          ),
            'roll': 0,
            'local_x_axis': self.canvas.create_rectangle(scaled_x_robot,
                                                         scaled_y_robot,
                                                         scaled_x_robot + const.X_AXES_LENGTH,
                                                         scaled_y_robot + const.X_AXES_WIDTH,
                                                         fill='red'
                                                         ),
            'local_y_axis': self.canvas.create_rectangle(scaled_x_robot,
                                                         scaled_y_robot,
                                                         scaled_x_robot + const.Y_AXES_WIDTH,
                                                         scaled_y_robot + const.Y_AXES_LENGTH,
                                                         fill='red'
                                                         ),
        }

        self.root.mainloop()

    def set_path(self):
        path = eval(self.path_entry.get())

        for (x, y) in path:
            scaled_x, scaled_y = self.scaled_point(x, y)

            self.canvas.create_oval(scaled_x,
                                    scaled_y,
                                    scaled_x + const.POINT_RADIUS,
                                    scaled_y + const.POINT_RADIUS,
                                    fill='green')

    def scaled_point(self, x: float, y: float) -> 'tuple[float, float]':
        scaled_x = x * const.RESOLUTION_X
        scaled_y = y * const.RESOLUTION_Y

        coordinates = (scaled_x, scaled_y)

        return coordinates
    def make_message(self):
        path = eval(self.path_entry.get())
        self.robot.make_message_from_path(path)
        for key, value in self.robot.trace.items():
            scaled_x, scaled_y = self.scaled_point(key, value)
            self.canvas.create_oval(scaled_x,
                                    scaled_y,
                                    scaled_x + 5,
                                    scaled_y + 5,
                                    fill='white')

        #print(self.robot.messages)




