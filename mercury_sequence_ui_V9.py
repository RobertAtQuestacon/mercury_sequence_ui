import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import json
import threading
import time, os
import builtins
import numpy as np  #this is workaround for possibile bug in pymycobot
builtins.np = np 

try:
    from pymycobot import Mercury
except ImportError:
    class Mercury:
        def __init__(self, port, baud):
            print(f"[Init] Mock Mercury: {port} ({baud})")
        def get_angles(self): return [0.0] * 7
        def get_coords(self): return [0.0] * 6
        def send_angles(self, angles, speed): print(f"[API] send_angles: {angles} at speed {speed}")
        def send_angle(self, j_id, angle, speed): print(f"[API] send_angle: J{j_id} to {angle} at speed {speed}")
        def send_coords(self, coords, speed): print(f"[API] send_coords: {coords} at speed {speed}")
        def focus_servo(self, j_id): print(f"[API] focus_servo: J{j_id}")
        def release_servo(self, j_id): print(f"[API] release_servo: J{j_id}")
        def focus_all_servos(self): print("[API] focus_all_servos")
        def release_all_servos(self): print("[API] release_all_servos")
        def get_robot_status(self): return 0 
        def stop(self, deceleration=True): print(f"[API] stop (deceleration={deceleration})")

class MercurySequenceApp(tk.Tk):

    pos_step = 1
    rot_step = 5
    axis_names = ("X","Y","Z","RX","RY","RZ")
    
    jogging = False
    jog_axis = "X"
    jog_amount = 1

    undo_list = [[]]
    undo_index = 1
    
    def __init__(self):
        super().__init__()
        self.title("Mercury A1 Sequence Programmer")
        self.geometry("1100x850")
        
        # Initialize Robot
        self.robot = Mercury("/dev/ttyAMA1", 115200)
        
        # State variables
        self.sequence_steps = []
        self.is_running = False
        self.last_transmit_time = 0
        self.throttle_interval = 0.05  # 50ms
        
        # Robot mechanical limits from docs
        self.joint_limits = [
            (-178, 178), # J1
            (-130, 130), # J2
            (-175, 175), # J3
            (-180, 10),  # J4
            (-175, 175), # J5
            (-20, 273),  # J6
            (-175, 175)  # J7
        ]
        try:
          if self.robot:
            for ji in range(7):
              self.joint_limits[ji] = (self.robot.get_joint_min_angle(ji+1), self.robot.get_joint_max_angle(ji+1))
        except:
          pass
          
        
        self.build_ui()
        self.load_current_pos()
        
        self.bind("<Control-c>",self.copy_selected)
        self.bind("<Control-x>",self.cut_selected)
        self.bind("<Control-v>",self.paste_steps)
        self.bind("<Control-z>",self.undo)
        self.bind("<Control-Z>",self.redo)
        self.bind("<Delete>",self.delete_selected)
        
        # Start Safety / Collision thread
        self.safety_active = True
        self.safety_thread = threading.Thread(target=self.monitor_collision, daemon=True)
        self.safety_thread.start()

        self.status_thread = threading.Thread(target=self.monitor_status, daemon=True)
        self.status_thread.start()
        
    def build_control_row(self, parent, name, ci, step):
        # Configure columns inside the frames
        #parent.columnconfigure(1, weight=1)
        
        
        
        # Current Value Display Label
        #lbl_val = ttk.Label(parent, textvariable=self.vars[name], font=("Consolas", 11), width=10, anchor="center", relief="sunken")
        #lbl_val.grid(row=row_idx, column=1, padx=5, pady=5, sticky="ew")
        # Down/Decrease Arrow Button (◀)
        btn_down = ttk.Button(parent, text="◀", width=2)
        btn_down.bind("<ButtonPress-1>", lambda event: self.start_adjust_value(name, -step))
        btn_down.bind("<ButtonRelease-1>", lambda event: self.stop_adjust_value(name))
        btn_down.grid(row=0, column=ci*4, padx=2, pady=5)
        # Axis Name Label
        lbl_name = ttk.Label(parent, text=f"{name}", font=("Arial", 8, "bold"), width=4)
        lbl_name.grid(row=0, column=ci*4+1, padx=0, pady=5 , sticky="w")      
        # Up/Increase Arrow Button (▶)
        btn_up = ttk.Button(parent, text="▶", width=2)
        btn_up.bind("<ButtonPress-1>", lambda event: self.start_adjust_value(name, step))
        btn_up.bind("<ButtonRelease-1>", lambda event: self.stop_adjust_value(name))
        btn_up.grid(row=0, column=ci*4+2, padx=2, pady=5)
        lbl_name = ttk.Label(parent, text="    ", font=("Arial", 8, "bold"), width=4)
        lbl_name.grid(row=0, column=ci*4+3, padx=0, pady=5 , sticky="w") 
 

    def build_ui(self):   
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH) #, expand=True, padx=10, pady=10)

        # Master frames layout
        control_frame = ttk.Frame(self.notebook, padding="10")
        #control_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        sequence_frame = ttk.Frame(self.notebook, padding="10")
        #sequence_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.notebook.add(control_frame, text="Manual Control & Setup")
        self.notebook.add(sequence_frame, text="Sequence Programmer")
        
        # --- LEFT FRAME: SLIDERS & CONTROLS ---
        # Connection/Global Torque Block
        global_frame = ttk.LabelFrame(control_frame, text="Global Controls", padding="5")
        global_frame.pack(fill=tk.X, pady=5)
        ttk.Button(global_frame, text="Engage All Servos", command=self.engage_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(global_frame, text="Release All Servos", command=self.release_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(global_frame, text="Load Current Position", command=self.load_current_pos).pack(side=tk.LEFT, padx=5)
        
        # Live Transmit Checkbox Fix Frame
        #live_frame = ttk.Frame(control_frame)
        #live_frame.pack(fill=tk.X, pady=5)
        self.live_transmit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(global_frame, text="Live to Robot Arm", variable=self.live_transmit_var).pack(side=tk.LEFT, padx=5)
        
        # Sliders Block
        slider_block = ttk.LabelFrame(control_frame) #, text="Joint Sliders Angles", padding="5")
        slider_block.pack(fill=tk.X) #, expand=True, pady=5)
        
        self.slider_vars = []
        self.slider_widgets = []
        self.joint_engage_vars = []
        
        for i in range(7):
            f = ttk.Frame(slider_block)
            f.pack(fill=tk.X, pady=4)
            
            # Engagement Checkbox
            ev = tk.BooleanVar(value=True)
            self.joint_engage_vars.append(ev)
            cb = ttk.Checkbutton(f, text=f"J{i+1}", variable=ev, command=lambda idx=i: self.toggle_joint_engagement(idx))
            cb.pack(side=tk.LEFT, padx=2)
            
            # Limit bounds
            low, high = self.joint_limits[i]
            sv = tk.DoubleVar(value=0.0)
            self.slider_vars.append(sv)
            
            # Live tracking commands specifically via standard lambda forwarding to trigger on drag updates
            sc = tk.Scale(f, from_=low, to=high, orient=tk.HORIZONTAL, variable=sv, 
                          resolution=0.1, command=lambda val, idx=i: self.on_slider_move(idx, val))
            sc.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.slider_widgets.append(sc)
            
            # Numeric readout text box
            lbl = ttk.Label(f, text="0.0°", width=8)
            lbl.pack(side=tk.RIGHT, padx=2)
            # Link update label logic
            sv.trace_add("write", lambda *args, idx=i, var=sv, l=lbl: l.config(text=f"{var.get():.1f}°"))

        # Cursor Block
        cursor_block = ttk.LabelFrame(control_frame) #, text="Coordinate Adjust", padding="5")
        cursor_block.pack(fill=tk.X) #, expand=True, pady=5)
        for ai,anm in enumerate(self.axis_names):
          self.build_control_row(cursor_block, anm, ai, self.pos_step if ai < 3 else self.rot_step)

        # Manual Entry fields layout for adding/saving individual steps
        entry_frame = ttk.LabelFrame(self, text="Step Profile Creator", padding="5")
        entry_frame.pack(side=tk.BOTTOM,fill=tk.X, pady=5)
        
        ttk.Label(entry_frame, text="Type:").grid(row=0, column=0, padx=2, pady=2)
        self.type_var = tk.StringVar(value="Angles")
        ttk.OptionMenu(entry_frame, self.type_var, "Angles", "Angles", "Coords",command=self.type_change).grid(row=0, column=1, padx=2, pady=2)
        
        ttk.Label(entry_frame, text="Parameters (comma-sep):").grid(row=0, column=2, padx=2, pady=2)
        self.param_entry = ttk.Entry(entry_frame, width=30)
        self.param_entry.grid(row=0, column=3, padx=2, pady=2)
        self.param_entry.insert(0, "0,0,0,0,0,0,0")
        
        ttk.Label(entry_frame, text="Speed:").grid(row=1, column=0, padx=2, pady=2)
        self.speed_entry = ttk.Entry(entry_frame, width=8)
        self.speed_entry.grid(row=1, column=1, padx=2, pady=2)
        self.speed_entry.insert(0, "30")
        
        ttk.Label(entry_frame, text="Delay (s):").grid(row=1, column=2, padx=2, pady=2)
        self.delay_entry = ttk.Entry(entry_frame, width=8)
        self.delay_entry.grid(row=1, column=3, padx=2, pady=2)
        self.delay_entry.insert(0, "1.0")
        
        ttk.Button(entry_frame, text="Add to Steps", command=self.add_step_from_sliders).grid(row=2, column=0, columnspan=2, pady=5)
        ttk.Button(entry_frame, text="Replace step", command=self.replace_step_from_sliders).grid(row=2, column=1, columnspan=2, pady=5)
        ttk.Button(entry_frame, text="Add Manual Entry Step", command=self.add_manual_step).grid(row=2, column=2, columnspan=2, pady=5)

        # --- RIGHT FRAME: SEQUENCE LIST & LOGGER ---
        seq_frame = ttk.LabelFrame(sequence_frame, text="Programmed Motion Sequences", padding="5")
        seq_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Step Table Setup
        tree_frame = ttk.Frame(seq_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self.tree = ttk.Treeview(tree_frame, columns=("Index", "Type", "Target Vector", "Speed", "Delay"), show="headings", height=5, selectmode="extended")
        self.tree.heading("Index", text="Index")
        self.tree.heading("Type", text="Type")
        self.tree.heading("Target Vector", text="Target Vector")
        self.tree.heading("Speed", text="Speed")
        self.tree.heading("Delay", text="Delay")
        self.tree.column("Index", width=50, anchor=tk.CENTER)
        self.tree.column("Type", width=70, anchor=tk.CENTER)
        self.tree.column("Target Vector", width=250)
        self.tree.column("Speed", width=60, anchor=tk.CENTER)
        self.tree.column("Delay", width=60, anchor=tk.CENTER)
        tree_scroll = ttk.Scrollbar(tree_frame, orient="vertical",command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scroll.set)
        self.tree.grid(row=0,column=0,sticky="nsew")
        tree_scroll.grid(row=0,column=1,stick="ns")

        # Core Sequence Action Buttons
        btn_layout = ttk.Frame(seq_frame)
        btn_layout.pack(fill=tk.X, pady=2)
        
        # Added requested layout buttons (Delete Step & Go to Step Position)
        ttk.Button(btn_layout, text="Go to Step Position", command=self.go_to_selected_step).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Edit speed", command=self.edit_speed).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Edit_delay", command=self.edit_dly).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Del", command=self.delete_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Copy", command=self.copy_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Cut", command=self.cut_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Paste", command=self.paste_steps).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Undo", command=self.undo).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_layout, text="Redo", command=self.redo).pack(side=tk.LEFT, padx=2)
        
        # Execution loop settings Block
        exec_frame = ttk.LabelFrame(sequence_frame, text="Execution Controls", padding="5")
        exec_frame.pack(fill=tk.X, pady=5)
        
        self.loop_infinite_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(exec_frame, text="Enable Infinite Looping", variable=self.loop_infinite_var).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(exec_frame, text="Loop Count:").pack(side=tk.LEFT, padx=2)
        self.loop_count_entry = ttk.Entry(exec_frame, width=6)
        self.loop_count_entry.pack(side=tk.LEFT, padx=2)
        self.loop_count_entry.insert(0, "1")
        
        ttk.Button(exec_frame, text="START SEQUENCE", command=self.start_sequence).pack(side=tk.RIGHT, padx=5)
        ttk.Button(exec_frame, text="STOP", command=self.stop_sequence).pack(side=tk.RIGHT, padx=5)
        
        # File Configuration Management
        file_frame = ttk.LabelFrame(sequence_frame, text="File Profiles", padding="5")
        file_frame.pack(fill=tk.X, pady=5)
        ttk.Button(file_frame, text="Save Sequence to JSON", command=self.save_to_file).pack(side=tk.LEFT, padx=5)
        ttk.Button(file_frame, text="Load Sequence from JSON", command=self.load_from_file).pack(side=tk.LEFT, padx=5)
        ttk.Button(file_frame, text="Clear All Steps", command=self.clear_all_steps).pack(side=tk.LEFT, padx=2)
        ttk.Button(file_frame, text="Append Sequence from JSON", command=self.append_from_file).pack(side=tk.LEFT, padx=5)
        
        # System Log Window (Mitigates X11 BadLength Crashes)
        log_frame = ttk.LabelFrame(sequence_frame, text="Status Terminal Logs", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=3, width=50)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        # Enforce strict 500-line history buffer ceiling to avoid X11 BadLength protocol packet crashes
        if os.environ.get('DISPLAY') or True:
            if float(self.log_text.index('end-1c')) > 500.0:
                self.log_text.delete("1.0", "2.0")
        self.log_text.see(tk.END)

    def toggle_joint_engagement(self, idx):
        if self.joint_engage_vars[idx].get():
            self.robot.focus_servo(idx + 1)
            self.log(f"Engaged Joint J{idx+1}")
        else:
            self.robot.release_servo(idx + 1)
            self.log(f"Disengaged Joint J{idx+1} (Compliance Mode Active)")
            
    def monitor_status(self):
        while True:
          for ei,ev in enumerate(self.joint_engage_vars):
            if not ev.get():
              self.load_current_pos()
              break
          time.sleep(0.5)        

    def engage_all(self):
        self.robot.focus_all_servos()
        for ev in self.joint_engage_vars:
            ev.set(True)
        self.load_current_pos()
        self.log("Engaged all robot servo actuators.")

    def release_all(self):
        self.robot.release_all_servos()
        for ev in self.joint_engage_vars:
            ev.set(False)
        self.log("Released all robot joints into compliant mode.")

    def on_slider_move(self, idx, val):
        # Executed directly when manual sliders are swept. 
        # Fixed: val string parameter is parsed and dynamically passed out to send_angle via internal clock gating
        if self.live_transmit_var.get():
            curr_time = time.time()
            #print(f"Axis {idx} moved to {val} delta tm {curr_time - self.last_transmit_time}")
            if curr_time - self.last_transmit_time > self.throttle_interval:
                #try:
                if True:
                    # Extract direct vector arrays currently positioned on the slider UI components
                    vec = [round(self.slider_vars[i].get(), 1) for i in range(7)]
                    #angle_val = float(val)
                    speed_val = int(self.speed_entry.get())
                    # API explicitly demands: send_angle(id, degree, speed)
                    #self.robot.send_angle(idx + 1, angle_val, speed_val)
                    self.robot.send_angles(vec, speed_val)
                    self.last_transmit_time = curr_time
                #except Exception as e:
                #    pass
                
    def type_change(self,typ):
      self.load_current_pos()
      
    def start_adjust_value(self,name,amount):
      self.jogging = True
      self.jog_axis = name
      self.jog_amount = amount
      self.jog_loop()
      
    def jog_loop(self):
      if self.jogging:
        self.adjust_value(self.jog_axis,self.jog_amount)
        self.after(100,self.jog_loop)
      #endif
    #enddef
      
    def stop_adjust_value(self,name):
      self.jogging = False
    #enddef    

    def adjust_value(self, name, amount):
        ai = self.axis_names.index(name)
        if ai < 0:
          return
        try:
        #if True:
          data = self.robot.get_coords()
          if data and len(data) >= 6:
            data[ai] += amount
        except Exception as e:
            self.log(f"Failed to interrogate hardware data streams: {e}")
        finally:
          sp = int(self.speed_entry.get())
          self.robot.send_coords(data, sp)
          self.load_current_pos()
    #enddef       

    def load_current_pos(self):
        try:
            data = self.robot.get_angles()
            if data and len(data) >= 7:
              # Sync workspace sliders and update live UI fields
              for i in range(7):
                self.slider_vars[i].set(round(data[i], 1))
            mode = self.type_var.get()
            if mode == "Angles":
              if data and len(data) >= 7:
                for i in range(7):
                    self.param_entry.delete(0, tk.END)
                    self.param_entry.insert(0, ",".join(f"{x:.1f}" for x in data[:7]))
                    self.log(f"Synced Current Joint Position Matrix: {data[:7]}")
            else:
                data = self.robot.get_coords()
                #print("Coords:",data)
                if data and len(data) >= 6:
                    self.param_entry.delete(0, tk.END)
                    self.param_entry.insert(0, ",".join(f"{x:.1f}" for x in data[:6]))
                    self.log(f"Synced Current Cartesian Spatial Coordinates: {data[:6]}")
        except Exception as e:
            self.log(f"Failed to interrogate hardware data streams: {e}")

    def add_step_from_sliders(self):
        self.save_step()
    
    def replace_step_from_sliders(self):
        self.save_step(True)
    
    def save_step(self,replacing=False):
        # Extract direct vector arrays currently positioned on the slider UI components
        vec = [round(self.slider_vars[i].get(), 1) for i in range(7)]
        try:
            sp = int(self.speed_entry.get())
            dl = float(self.delay_entry.get())
            step = {"type": "Angles", "vector": vec, "speed": sp, "delay": dl}
            selected = self.tree.selection()
            if replacing and selected:
              idx = int(self.tree.item(selected[0], "values")[0]) - 1
              if 0 <= idx < len(self.sequence_steps):
                self.sequence_steps[idx] = step
              #endif  
            else:
              self.sequence_steps.append(step)
            #endif
            self.refresh_sequence_table()
            self.log(f"Added step from active sliders: {vec}")
            self.save_undo()
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric constraints entered for timing configurations.")

    def add_manual_step(self):
        try:
            m_type = self.type_var.get()
            vec_str = self.param_entry.get().split(",")
            vec = [float(x.strip()) for x in vec_str]
            
            if m_type == "Angles" and len(vec) != 7:
                raise ValueError("Joint profiles demand 7 distinct axis angles.")
            if m_type == "Coords" and len(vec) != 6:
                raise ValueError("Spatial paths demand 6 distinct Cartesian dimensions.")
                
            sp = int(self.speed_entry.get())
            dl = float(self.delay_entry.get())
            
            step = {"type": m_type, "vector": vec, "speed": sp, "delay": dl}
            self.sequence_steps.append(step)
            self.refresh_sequence_table()
            self.log(f"Added Manual Entry Step: {m_type} {vec}")
            self.save_undo()
        except Exception as e:
            messagebox.showerror("Validation Failed", f"Bad string parameters parsed: {e}")

    def delete_selected(self,v=0):
        if not self.seq_tab_open():
          return
        # Requested specific item modification layout utility
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("Selection Missing", "Please select a sequence step row to delete.")
            return
        for item in selected:
            idx = int(self.tree.item(item, "values")[0]) - 1
            if 0 <= idx < len(self.sequence_steps):
                del self.sequence_steps[idx]
        self.refresh_sequence_table()
        self.log("Deleted selected movement profile step configuration.")
        self.save_undo()
        
    def edit_speed(self,v=0):
        self.edit_selected(True)
    #enddef
        
    def edit_dly(self,v=0):
        self.edit_selected(False)
    #enddef
        
    def edit_selected(self,edit_speed):
        if not self.seq_tab_open():
          return
        # Requested specific item modification layout utility
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("Selection Missing", "Please select a sequence step row to edit.")
            return
        sp = int(self.speed_entry.get())
        dl = float(self.delay_entry.get())
        # step = {"type": "Angles", "vector": vec, "speed": sp, "delay": dl}
        for item in selected:
            idx = int(self.tree.item(item, "values")[0]) - 1
            if 0 <= idx < len(self.sequence_steps):
              if edit_speed:
                self.sequence_steps[idx]["speed"] = sp 
              else:  
                self.sequence_steps[idx]["delay"] = dl 
        self.refresh_sequence_table()
        self.log("Edited selected movement profile step configuration.")
        self.save_undo()
    

    def go_to_selected_step(self):
        # Added requested direct step position dispatch routine
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("Selection Missing", "Please highlight a step row inside the table matrix first.")
            return
        try:
            idx = int(self.tree.item(selected[0], "values")[0]) - 1
            if 0 <= idx < len(self.sequence_steps):
                step = self.sequence_steps[idx] 
                self.log(f"Executing discrete command for Step #{idx+1}...")
                if step["type"] == "Angles":
                    self.robot.send_angles(step["vector"], step["speed"])
                else:
                    self.robot.send_coords(step["vector"], step["speed"])
                self.load_current_pos()
        except Exception as e:
            self.log(f"Failed to command discrete target execution jump: {e}")

    def clear_all_steps(self):
        self.sequence_steps.clear()
        self.refresh_sequence_table()
        self.log("Cleared sequence memory block.")
        
    def seq_tab_open(self):
        #print("Notetab",self.notebook.index(self.notebook.select()))
        return (self.notebook.index(self.notebook.select()) == 1)
        
    def save_undo(self):
        if self.undo_index >= len(self.undo_list):
          if len(self.undo_list) > 100:
            del self.undo_list[0]
        else:
          self.undo_list = self.undo_list[0:self.undo_index]  
        self.undo_list.append(list(self.sequence_steps))
        self.undo_index = len(self.undo_list)
        #print("Undo_list:",len(self.undo_list))
    
    def undo(self,v=0):
        if self.undo_index > 0:
          self.undo_index -= 1
          self.log(f"Undo event {self.undo_index} of {len(self.undo_list)}")
          if self.undo_index > 0:
            self.sequence_steps = list(self.undo_list[self.undo_index-1])
          else:
            self.sequence_steps = []
          self.refresh_sequence_table()
          #print("Seq_list:",self.sequence_steps)
         #endif
    #enddef
    
    def redo(self,v=0):
      if self.undo_index < len(self.undo_list):
        self.log(f"Redo event {self.undo_index} of {len(self.undo_list)}")
        self.sequence_steps = self.undo_list[self.undo_index]
        self.undo_index += 1
        self.refresh_sequence_table()
      #endif
    #enddef
        
    def copy_selected(self,v=0):
        if not self.seq_tab_open():
          return
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("Selection Missing", "Please select a sequence step row to delete.")
            return
        indexes = [self.tree.index(item) for item in selected]
        indexes.sort()
        indexes.reverse()
        self.clipboard = []
        for i in indexes:
          self.clipboard.append(self.sequence_steps[i])
        #endfor
        self.log(f"Copied {len(indexes)} items to clipboard")
    #enddef
        
    def cut_selected(self,v=0):
        if not self.seq_tab_open():
          return
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("Selection Missing", "Please select a sequence step row to delete.")
            return
        indexes = [self.tree.index(item) for item in selected]
        indexes.sort()
        self.clipboard = []
        for i in indexes:
          self.clipboard.append(self.sequence_steps[i])
        #endfor
        indexes.reverse()
        for i in indexes:
          del self.sequence_steps[i]
        #endfor
        self.log(f"Cut {len(indexes)} items")
        self.refresh_sequence_table()
        self.save_undo()
       
    def paste_steps(self,v=0):
        if not self.seq_tab_open():
          return
        selected = self.tree.selection()
        if len(selected) == 0:
          self.log(f"Pasting {len(self.clipboard)} items to end")   
          self.sequence_steps += self.clipboard
          self.refresh_sequence_table()       
          self.save_undo()
          return
        #endif
        indexes = [self.tree.index(item) for item in selected]
        indexes.sort()
        i = indexes[0]
        self.log(f"Pasting {len(self.clipboard)} items to index {i}")
        for c in self.clipboard:
          self.sequence_steps.insert(i,c)
          i += 1
        self.refresh_sequence_table()       
        self.save_undo()

    def refresh_sequence_table(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for i, s in enumerate(self.sequence_steps):
            vec_str = ", ".join(str(x) for x in s["vector"])
            self.tree.insert("", tk.END, values=(i+1, s["type"], vec_str, s["speed"], s["delay"]))

    def start_sequence(self):
        if not self.sequence_steps:
            messagebox.showwarning("Execution Aborted", "Sequence matrix memory contains zero valid execution paths.")
            return
        if self.is_running:
            return
        self.is_running = True
        threading.Thread(target=self.sequence_runner_loop, daemon=True).start()

    def sequence_runner_loop(self):
        self.log("Beginning automated chronological macro path array loop...")
        try:
            loops_left = 1 if not self.loop_infinite_var.get() else -1
            if loops_left == 1:
                try:
                    loops_left = int(self.loop_count_entry.get())
                except ValueError:
                    loops_left = 1
            
            loop_idx = 0
            while self.is_running and (self.loop_infinite_var.get() or loop_idx < loops_left):
                self.log(f"Starting Profile Sequence Loop Cycle #{loop_idx + 1}")
                for i, step in enumerate(self.sequence_steps):
                    if not self.is_running:
                        break
                        
                    self.log(f"Processing Step {i+1}/{len(self.sequence_steps)} -> {step['type']}")
                    if step["type"] == "Angles":
                        self.robot.send_angles(step["vector"], step["speed"])
                    else:
                        self.robot.send_coords(step["vector"], step["speed"])
                    self.load_current_pos()    
                    # Decoupled timeline pause mapping
                    time.sleep(step["delay"])
                    
                loop_idx += 1
                if not self.loop_infinite_var.get() and loop_idx >= loops_left:
                    break
            self.log("Sequence macro chain fully executed or reached custom termination limits.")
        except Exception as e:
            self.log(f"Sequence loop tracking execution fatal trace: {e}")
        finally:
            self.is_running = False

    def stop_sequence(self):
        self.is_running = False
        self.robot.stop(deceleration=1)  #documents says deceleration is bool but code shows integer
        self.log("EMERGENCY INTERRUPT: Issued deceleration braking command to physical frame controllers.")

    def monitor_collision(self):
        while self.safety_active:
            try:
                # Poll upper computer register error states via doc specifications
                status = self.robot.get_robot_status()
                if status:
                  if len(status) == 2:
                    for i in status[1]:
                      if status[0][i] != 0:
                        self.is_running = False
                        self.robot.stop(deceleration=1)
                        self.log(f"⚠️ SAFETY INTERCEPT CRITICAL: Hardware flag reported error code ({status}). Braking applied!")
                        break
                      #endif
                    #endfor
                  #endif
                #endif  
            except Exception:
                pass
            time.sleep(0.2)

    def save_to_file(self):
        fn = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])
        if fn:
            try:
                with open(fn, "w") as f:
                    json.dump(self.sequence_steps, f, indent=4)
                self.log(f"Profile configurations written successfully to {fn}")
            except Exception as e:
                messagebox.showerror("IO Failure", f"Could not export structured profile traces: {e}")

    def load_from_file(self):
        self.load_json()
    
    def append_from_file(self):
        self.load_json(True)
    
    def load_json(self,is_append=False):
        if not is_append:
          self.undo_list = []
          self.undo_index = 0
        self.save_undo()
        fn = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if fn:
            try:
                with open(fn, "r") as f:
                    data = json.load(f)
                if isinstance(data, list):
                    if is_append:
                      self.sequence_steps += data
                    else:
                      self.sequence_steps = data
                    self.refresh_sequence_table()
                    self.log(f"Successfully loaded {len(data)} profiles from {fn}")
                else:
                    raise ValueError("JSON data layout profile must be structured as an array matrix.")
            except Exception as e:
                messagebox.showerror("IO Failure", f"Could not parse file configurations: {e}")

if __name__ == "__main__":
    app = MercurySequenceApp()
    app.mainloop()
