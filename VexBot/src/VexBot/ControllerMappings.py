class XboxMapping:

    def __init__(self):
        self.guide = 8
        self.A = 0
        self.B = 1
        self.X = 2
        self.Y = 3
        self.back = 6
        self.start = 7
        self.LeftBumper = 4
        self.RightBumper = 5
        self.LeftStick = 9
        self.RightStick = 10
        self.DpadUp = 13
        self.DpadDown = 14
        self.DpadLeft = 11
        self.DpadRight = 12
        self.LeftTrigger = 2
        self.RightTrigger = 5
        self.LeftStickUD = 1
        self.LeftStickLR = 0
        self.RightStickUD = 4
        self.RightStickLR = 3

    def msg_to_dict(self, msg):
        msg_dict = {}
        msg_dict["guide"] = msg.buttons[self.guide]
        msg_dict["A"] = msg.buttons[self.A]
        msg_dict["B"] = msg.buttons[self.B]
        msg_dict["X"] = msg.buttons[self.X]
        msg_dict["Y"] = msg.buttons[self.Y]
        msg_dict["back"] = msg.buttons[self.back]
        msg_dict["start"] = msg.buttons[self.start]
        msg_dict["left_stick_btn"] = msg.buttons[self.LeftStick]
        msg_dict["right_stick_btn"] = msg.buttons[self.RightStick]
        msg_dict["left_bumper"] = msg.buttons[self.LeftBumper]
        msg_dict["right_bumper"] = msg.buttons[self.RightBumper]
        msg_dict["dpad_up"] = msg.buttons[self.DpadUp]
        msg_dict["dpad_down"] = msg.buttons[self.DpadDown]
        msg_dict["dpad_left"] = msg.buttons[self.DpadLeft]
        msg_dict["dpad_right"] = msg.buttons[self.DpadRight]
        msg_dict["left_trigger"] = msg.axes[self.LeftTrigger]
        msg_dict["right_trigger"] = msg.axes[self.RightTrigger]
        msg_dict["left_stick_UDaxis"] = msg.axes[self.LeftStickUD]
        msg_dict["left_stick_LRaxis"] = msg.axes[self.LeftStickLR]
        msg_dict["right_stick_UDaxis"] = msg.axes[self.RightStickUD]
        msg_dict["right_stick_LRaxis"] = msg.axes[self.RightStickLR]
        return msg_dict
