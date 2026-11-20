#!/usr/bin/env python3

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from custom_msgs.msg import CollisionDetectionEnable, ShipLoaderPose


class PlcSimulatorHmi(Node):
    def __init__(self):
        super().__init__("plc_simulator_hmi")

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/plc_simulator_node/input/ship_loader_pose"
        ).value
        self.enable_topic = self.declare_parameter(
            "enable_topic", "/plc_simulator_node/input/collision_detection_enable"
        ).value
        self.publish_period_ms = self.declare_parameter("publish_period_ms", 100).value

        self.pose_pub = self.create_publisher(ShipLoaderPose, self.pose_topic, 10)
        self.enable_pub = self.create_publisher(
            CollisionDetectionEnable, self.enable_topic, 10
        )

        self.root = tk.Tk()
        self.root.title("PLC Simulator HMI")
        self.root.geometry("1280x760")
        self.root.minsize(1080, 680)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.pose_vars = {
            "walking_dist": tk.DoubleVar(value=0.0),
            "slewing_angle": tk.DoubleVar(value=0.0),
            "luffing_angle": tk.DoubleVar(value=0.0),
            "stretch_dist": tk.DoubleVar(value=0.0),
        }
        self.enable_vars = {
            "front": tk.BooleanVar(value=True),
            "left": tk.BooleanVar(value=True),
            "right": tk.BooleanVar(value=True),
            "back": tk.BooleanVar(value=True),
        }

        self.closed = False
        self.build_ui()
        self.schedule_update()

    def build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        title = ttk.Label(
            main,
            text="Ship Loader Pose Control",
            font=("Arial", 14, "bold"),
        )
        title.pack(anchor=tk.W, pady=(0, 10))

        subtitle = ttk.Label(
            main,
            text="按可门二期装船机传感器布置图做了简化侧视/俯视模型，便于联调姿态和防撞点位。",
        )
        subtitle.pack(anchor=tk.W, pady=(0, 10))

        content = ttk.Frame(main)
        content.pack(fill=tk.BOTH, expand=True)

        control_panel = ttk.Frame(content)
        control_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))

        visual_panel = ttk.Frame(content)
        visual_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        pose_frame = ttk.LabelFrame(control_panel, text="Pose", padding=10)
        pose_frame.pack(fill=tk.X)
        self.add_slider(
            pose_frame, "Walking Distance (m)", "walking_dist", -50.0, 50.0
        )
        self.add_slider(
            pose_frame, "Slewing Angle (deg)", "slewing_angle", -180.0, 180.0
        )
        self.add_slider(pose_frame, "Luffing Angle (deg)", "luffing_angle", -30.0, 90.0)
        self.add_slider(pose_frame, "Stretch Distance (m)", "stretch_dist", 0.0, 80.0)

        enable_frame = ttk.LabelFrame(
            control_panel, text="Collision Detection Enable", padding=10
        )
        enable_frame.pack(fill=tk.X, pady=(12, 12))
        for idx, key in enumerate(["front", "left", "right", "back"]):
            ttk.Checkbutton(
                enable_frame,
                text=key.capitalize(),
                variable=self.enable_vars[key],
            ).grid(row=0, column=idx, padx=8, sticky=tk.W)

        button_frame = ttk.Frame(control_panel)
        button_frame.pack(fill=tk.X, pady=(8, 0))

        ttk.Button(button_frame, text="Reset Pose", command=self.reset_pose).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(button_frame, text="Publish Now", command=self.publish_state).pack(
            side=tk.LEFT
        )

        info_frame = ttk.LabelFrame(control_panel, text="Model Legend", padding=10)
        info_frame.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(
            info_frame,
            text=(
                "红: Front  黄: Left\n"
                "蓝: Right  绿: Back\n"
                "紫: GNSS  青: 物位计"
            ),
            justify=tk.LEFT,
        ).pack(anchor=tk.W)

        self.status_var = tk.StringVar(
            value=f"Publishing to {self.pose_topic} and {self.enable_topic}"
        )
        ttk.Label(control_panel, textvariable=self.status_var, wraplength=300).pack(
            anchor=tk.W, pady=(16, 0)
        )

        model_frame = ttk.LabelFrame(visual_panel, text="Ship Loader Model", padding=8)
        model_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            model_frame,
            background="#f7f9fc",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self.on_canvas_configure)

        for variable in [*self.pose_vars.values(), *self.enable_vars.values()]:
            variable.trace_add("write", self.on_state_change)

    def add_slider(self, parent, label, key, minimum, maximum):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=4)

        ttk.Label(frame, text=label, width=22).pack(side=tk.LEFT)
        scale = tk.Scale(
            frame,
            from_=minimum,
            to=maximum,
            resolution=0.1,
            orient=tk.HORIZONTAL,
            variable=self.pose_vars[key],
            length=240,
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(frame, textvariable=self.pose_vars[key], width=8).pack(
            side=tk.LEFT, padx=(8, 0)
        )

    def on_state_change(self, *_args):
        self.redraw_model()

    def on_canvas_configure(self, _event):
        self.redraw_model()

    def redraw_model(self):
        if not hasattr(self, "canvas"):
            return

        canvas = self.canvas
        width = max(canvas.winfo_width(), 900)
        height = max(canvas.winfo_height(), 600)
        canvas.delete("all")

        side_area = (28, 28, width - 28, int(height * 0.54))
        top_area = (28, int(height * 0.58), width - 28, height - 28)

        canvas.create_rectangle(*side_area, fill="#ffffff", outline="#d9e1ea")
        canvas.create_rectangle(*top_area, fill="#ffffff", outline="#d9e1ea")
        canvas.create_text(
            side_area[0] + 12,
            side_area[1] + 10,
            anchor=tk.NW,
            text="Side View",
            font=("Arial", 11, "bold"),
            fill="#25364d",
        )
        canvas.create_text(
            top_area[0] + 12,
            top_area[1] + 10,
            anchor=tk.NW,
            text="Top View",
            font=("Arial", 11, "bold"),
            fill="#25364d",
        )

        self.draw_side_view(side_area)
        self.draw_top_view(top_area)

    def draw_side_view(self, area):
        canvas = self.canvas
        left, top, right, bottom = area
        width = right - left
        height = bottom - top

        walk = float(self.pose_vars["walking_dist"].get())
        luff = float(self.pose_vars["luffing_angle"].get())
        stretch = float(self.pose_vars["stretch_dist"].get())

        def sx(value):
            return left + (value / 100.0) * width

        def sy(value):
            return bottom - (value / 100.0) * height

        walk_shift = (walk / 50.0) * width * 0.16

        quay_y = sy(16)
        rail_y = sy(11)
        base_left = sx(20) + walk_shift
        base_right = sx(47) + walk_shift
        tower_x = sx(38) + walk_shift
        tower_top_y = sy(62)
        pivot_x = sx(39) + walk_shift
        pivot_y = sy(55)

        canvas.create_rectangle(left + 14, quay_y, right - 14, bottom - 14, fill="#eef3f8", outline="")
        canvas.create_line(left + 14, quay_y, right - 14, quay_y, fill="#7d8ea3", width=2)
        canvas.create_line(left + 14, rail_y, right - 14, rail_y, fill="#46576b", width=2)
        for tick in range(18):
            tx = left + 20 + tick * ((right - left - 40) / 17)
            canvas.create_line(tx, rail_y - 8, tx, rail_y + 8, fill="#66768a")

        canvas.create_rectangle(base_left, sy(18), base_right, sy(11.8), fill="#55697f", outline="#2f3e4d", width=2)
        wheel_centers = [base_left + 24, base_left + 72, base_right - 72, base_right - 24]
        for cx in wheel_centers:
            canvas.create_oval(cx - 9, rail_y - 9, cx + 9, rail_y + 9, fill="#26323d", outline="")

        canvas.create_rectangle(sx(34) + walk_shift, sy(55), sx(42) + walk_shift, sy(18), fill="#bcc9d6", outline="#596d80", width=2)
        canvas.create_rectangle(sx(30) + walk_shift, sy(61), sx(45) + walk_shift, sy(55), fill="#dde6ef", outline="#596d80", width=2)
        canvas.create_rectangle(sx(44) + walk_shift, sy(49), sx(61) + walk_shift, sy(42), fill="#c3d0dc", outline="#596d80", width=2)

        canvas.create_line(tower_x, sy(18), tower_x, tower_top_y, fill="#46576b", width=5)
        canvas.create_line(sx(34.8) + walk_shift, sy(28), sx(41.2) + walk_shift, sy(50), fill="#7b8ea3", width=2)
        canvas.create_line(sx(41.2) + walk_shift, sy(28), sx(34.8) + walk_shift, sy(50), fill="#7b8ea3", width=2)

        main_boom_len = width * 0.24
        boom_extension = (stretch / 80.0) * width * 0.14
        boom_len = main_boom_len + boom_extension
        boom_angle = math.radians(max(-15.0, min(65.0, luff)))
        boom_end_x = pivot_x + math.cos(boom_angle) * boom_len
        boom_end_y = pivot_y - math.sin(boom_angle) * boom_len * 0.72
        extension_start_x = pivot_x + math.cos(boom_angle) * main_boom_len
        extension_start_y = pivot_y - math.sin(boom_angle) * main_boom_len * 0.72
        boom_offset = 10
        canvas.create_line(
            pivot_x,
            pivot_y - boom_offset,
            extension_start_x,
            extension_start_y - boom_offset,
            fill="#f4a43b",
            width=7,
        )
        canvas.create_line(
            pivot_x,
            pivot_y + boom_offset,
            extension_start_x,
            extension_start_y + boom_offset,
            fill="#f4a43b",
            width=7,
        )
        canvas.create_line(
            extension_start_x,
            extension_start_y - boom_offset + 2,
            boom_end_x,
            boom_end_y - boom_offset + 2,
            fill="#ffd28a",
            width=6,
        )
        canvas.create_line(
            extension_start_x,
            extension_start_y + boom_offset - 2,
            boom_end_x,
            boom_end_y + boom_offset - 2,
            fill="#ffd28a",
            width=6,
        )
        for ratio in (0.22, 0.45, 0.68, 0.88):
            px = pivot_x + (boom_end_x - pivot_x) * ratio
            py = pivot_y + (boom_end_y - pivot_y) * ratio
            canvas.create_line(px - 8, py - 12, px + 8, py + 12, fill="#8f6328", width=2)

        cable_anchor_x = sx(39) + walk_shift
        cable_anchor_y = sy(74)
        canvas.create_line(cable_anchor_x, cable_anchor_y, boom_end_x, boom_end_y, fill="#75869a", width=2)
        canvas.create_line(cable_anchor_x, cable_anchor_y, pivot_x + 18, pivot_y - 4, fill="#75869a", width=2)

        chute_base_x = pivot_x + (boom_end_x - pivot_x) * 0.84
        chute_base_y = pivot_y + (boom_end_y - pivot_y) * 0.84
        chute_len = height * 0.27
        chute_end_x = chute_base_x
        chute_end_y = min(bottom - 18, chute_base_y + chute_len)
        canvas.create_line(
            chute_base_x,
            chute_base_y,
            chute_end_x,
            chute_end_y,
            fill="#4d6378",
            width=10,
        )
        canvas.create_line(
            chute_base_x + 6,
            chute_base_y,
            chute_end_x + 6,
            chute_end_y,
            fill="#95a6b7",
            width=3,
        )
        canvas.create_oval(chute_end_x - 14, chute_end_y - 10, chute_end_x + 14, chute_end_y + 10, fill="#7b8ea3", outline="#4d6378")

        ship_hull = [sx(70), sy(20), sx(87), sy(20), sx(91), sy(25), sx(75), sy(25)]
        canvas.create_polygon(*ship_hull, fill="#7a8694", outline="#56616f")
        canvas.create_rectangle(sx(76), sy(30), sx(84), sy(25), fill="#94a1af", outline="#56616f")

        self.draw_sensor_marker(sx(34.5) + walk_shift, sy(58), "GNSS-L", "gnss")
        self.draw_sensor_marker(sx(43.6) + walk_shift, sy(58), "GNSS-R", "gnss")
        self.draw_sensor_marker(sx(39.0) + walk_shift, sy(74), "GNSS-T", "gnss")
        self.draw_sensor_marker(pivot_x + (boom_end_x - pivot_x) * 0.9, pivot_y + (boom_end_y - pivot_y) * 0.9 - 12, "3D Front", "front")
        self.draw_sensor_marker(sx(35.2) + walk_shift, sy(16.6), "MR-LF", "front")
        self.draw_sensor_marker(sx(45.8) + walk_shift, sy(16.6), "MR-RF", "front")
        self.draw_sensor_marker(sx(22.5) + walk_shift, sy(16.6), "MR-LR", "back")
        self.draw_sensor_marker(sx(47.0) + walk_shift, sy(16.6), "MR-RR", "back")
        self.draw_sensor_marker(chute_base_x - 4, chute_base_y + 24, "Quick-L", "left")
        self.draw_sensor_marker(chute_base_x + 10, chute_base_y + 22, "Quick-R", "right")
        self.draw_sensor_marker(chute_end_x - 18, chute_end_y - 6, "Level-L", "level")
        self.draw_sensor_marker(chute_end_x + 18, chute_end_y - 6, "Level-R", "level")

        canvas.create_text(
            right - 14,
            top + 12,
            anchor=tk.NE,
            text=f"Walk {walk:5.1f} m   Luff {luff:5.1f} deg   Stretch {stretch:5.1f} m",
            font=("Arial", 10),
            fill="#425366",
        )

    def draw_top_view(self, area):
        canvas = self.canvas
        left, top, right, bottom = area
        width = right - left
        height = bottom - top

        walk = float(self.pose_vars["walking_dist"].get())
        slew = float(self.pose_vars["slewing_angle"].get())
        stretch = float(self.pose_vars["stretch_dist"].get())

        center_x = left + width * 0.42 + (walk / 50.0) * width * 0.08
        center_y = top + height * 0.54
        main_boom_len = width * 0.2
        boom_extension = (stretch / 80.0) * width * 0.12
        boom_len = main_boom_len + boom_extension
        angle = math.radians(slew)

        canvas.create_rectangle(left + 14, top + 14, right - 14, bottom - 14, fill="#f9fbfd", outline="")
        canvas.create_line(left + 24, center_y, right - 24, center_y, fill="#73859a", width=2)
        canvas.create_line(center_x, top + 22, center_x, bottom - 22, fill="#d9e1ea", dash=(4, 4))
        canvas.create_text(right - 18, top + 12, anchor=tk.NE, text=f"Slew {slew:5.1f} deg", font=("Arial", 10), fill="#425366")

        base_w = width * 0.16
        base_h = height * 0.18
        canvas.create_rectangle(center_x - base_w / 2, center_y - base_h / 2, center_x + base_w / 2, center_y + base_h / 2, fill="#becad6", outline="#596d80", width=2)
        canvas.create_oval(center_x - 22, center_y - 22, center_x + 22, center_y + 22, fill="#8d9bad", outline="#4f6174", width=2)

        extension_start_x = center_x + math.cos(angle) * main_boom_len
        extension_start_y = center_y - math.sin(angle) * main_boom_len * 0.72
        boom_end_x = center_x + math.cos(angle) * boom_len
        boom_end_y = center_y - math.sin(angle) * boom_len * 0.72
        nx = -math.sin(angle) * 12
        ny = -math.cos(angle) * 12
        canvas.create_polygon(
            center_x + nx,
            center_y + ny,
            center_x - nx,
            center_y - ny,
            extension_start_x - nx,
            extension_start_y - ny,
            extension_start_x + nx,
            extension_start_y + ny,
            fill="#f4a43b",
            outline="#8f6328",
            width=2,
        )
        inner_nx = -math.sin(angle) * 9
        inner_ny = -math.cos(angle) * 9
        canvas.create_polygon(
            extension_start_x + inner_nx,
            extension_start_y + inner_ny,
            extension_start_x - inner_nx,
            extension_start_y - inner_ny,
            boom_end_x - inner_nx,
            boom_end_y - inner_ny,
            boom_end_x + inner_nx,
            boom_end_y + inner_ny,
            fill="#ffd28a",
            outline="#8f6328",
            width=2,
        )

        chute_len = height * 0.2
        chute_end_x = boom_end_x
        chute_end_y = boom_end_y + chute_len
        canvas.create_line(boom_end_x, boom_end_y, chute_end_x, chute_end_y, fill="#4d6378", width=9)
        canvas.create_oval(chute_end_x - 15, chute_end_y - 15, chute_end_x + 15, chute_end_y + 15, fill="#7b8ea3", outline="#4d6378")

        ship_left = right - width * 0.23
        ship_top = top + height * 0.24
        ship_bottom = bottom - height * 0.24
        canvas.create_polygon(
            ship_left,
            ship_top,
            right - 42,
            ship_top + 10,
            right - 24,
            (ship_top + ship_bottom) / 2,
            right - 42,
            ship_bottom - 10,
            ship_left,
            ship_bottom,
            ship_left + 18,
            (ship_top + ship_bottom) / 2,
            fill="#90a0b0",
            outline="#5a6775",
        )
        canvas.create_text((ship_left + right - 24) / 2, ship_top - 12, text="Vessel", font=("Arial", 10), fill="#56616f")

        self.draw_sensor_marker(center_x, center_y - base_h * 0.75, "3D Lower", "front")
        self.draw_sensor_marker(center_x - base_w * 0.52, center_y, "3D Left", "left")
        self.draw_sensor_marker(center_x + base_w * 0.52, center_y, "3D Right", "right")
        self.draw_sensor_marker(center_x - base_w * 0.42, center_y + base_h * 0.55, "Rear-L", "back")
        self.draw_sensor_marker(center_x + base_w * 0.42, center_y + base_h * 0.55, "Rear-R", "back")
        self.draw_sensor_marker(boom_end_x - nx * 0.8, boom_end_y - ny * 0.8, "Quick-L", "left")
        self.draw_sensor_marker(boom_end_x + nx * 0.8, boom_end_y + ny * 0.8, "Quick-R", "right")
        self.draw_sensor_marker(chute_end_x - nx * 0.85, chute_end_y - ny * 0.85, "Level-L", "level")
        self.draw_sensor_marker(chute_end_x + nx * 0.85, chute_end_y + ny * 0.85, "Level-R", "level")

    def draw_sensor_marker(self, x, y, label, group):
        canvas = self.canvas
        fill = self.sensor_color(group)
        canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=fill, outline="#1e2b37")
        canvas.create_text(x + 9, y - 8, text=label, anchor=tk.SW, font=("Arial", 8), fill="#1f2e3b")

    def sensor_color(self, group):
        colors = {
            "front": "#ff6b6b" if self.enable_vars["front"].get() else "#d5dbe3",
            "left": "#f2c94c" if self.enable_vars["left"].get() else "#d5dbe3",
            "right": "#56a8ff" if self.enable_vars["right"].get() else "#d5dbe3",
            "back": "#61c48b" if self.enable_vars["back"].get() else "#d5dbe3",
            "gnss": "#9b7bff",
            "level": "#5cc9d8",
        }
        return colors.get(group, "#8898aa")

    def reset_pose(self):
        for var in self.pose_vars.values():
            var.set(0.0)
        self.publish_state()

    def publish_state(self):
        pose_msg = ShipLoaderPose()
        pose_msg.timestamp = self.get_clock().now().to_msg()
        pose_msg.walking_dist = float(self.pose_vars["walking_dist"].get())
        pose_msg.slewing_angle = float(self.pose_vars["slewing_angle"].get())
        pose_msg.luffing_angle = float(self.pose_vars["luffing_angle"].get())
        pose_msg.stretch_dist = float(self.pose_vars["stretch_dist"].get())
        self.pose_pub.publish(pose_msg)

        enable_msg = CollisionDetectionEnable()
        enable_msg.timestamp = self.get_clock().now().to_msg()
        enable_msg.front_collision_detection_enable = bool(
            self.enable_vars["front"].get()
        )
        enable_msg.left_collision_detection_enable = bool(
            self.enable_vars["left"].get()
        )
        enable_msg.right_collision_detection_enable = bool(
            self.enable_vars["right"].get()
        )
        enable_msg.down_collision_detection_enable = bool(
            self.enable_vars["back"].get()
        )
        self.enable_pub.publish(enable_msg)

        self.status_var.set(
            "Published pose: "
            f"walk={pose_msg.walking_dist:.1f}, "
            f"slew={pose_msg.slewing_angle:.1f}, "
            f"luff={pose_msg.luffing_angle:.1f}, "
            f"stretch={pose_msg.stretch_dist:.1f}"
        )

    def schedule_update(self):
        if self.closed:
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self.publish_state()
        self.redraw_model()
        self.root.after(int(self.publish_period_ms), self.schedule_update)

    def on_close(self):
        self.closed = True
        self.root.quit()
        self.root.destroy()


def main():
    rclpy.init()
    node = PlcSimulatorHmi()
    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
