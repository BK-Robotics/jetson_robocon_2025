import math
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class ConveyorCalculatorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Belt Conveyor Linear Velocity Calculator")
        self.root.geometry("1000x750")  # Increased window size
        self.root.minsize(1000, 750)    # Set minimum size
        self.root.configure(bg="#f0f0f0")
        
        # Configure style
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 11))
        style.configure("TButton", font=("Arial", 11, "bold"))
        style.configure("TRadiobutton", font=("Arial", 11))
        style.configure("TLabelframe.Label", font=("Arial", 12, "bold"))
        
        # Main frame with padding
        main_frame = ttk.Frame(root, padding="20")
        main_frame.pack(fill="both", expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="Belt Conveyor Linear Velocity Calculator", 
                               font=("Arial", 18, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=15)
        
        # Calculation mode frame
        mode_frame = ttk.LabelFrame(main_frame, text="Calculation Mode", padding="15")
        mode_frame.grid(row=1, column=0, columnspan=2, padx=15, pady=(10, 15), sticky="ew")
        
        # Calculation mode selection
        self.mode_var = tk.StringVar(value="forward")
        ttk.Radiobutton(mode_frame, text="Forward: Calculate Belt Speed from RPS", 
                      variable=self.mode_var, value="forward", 
                      command=self.update_mode).grid(row=0, column=0, padx=20, sticky="w")
        ttk.Radiobutton(mode_frame, text="Reverse: Calculate Required RPS from Belt Speed", 
                      variable=self.mode_var, value="reverse", 
                      command=self.update_mode).grid(row=0, column=1, padx=20, sticky="w")
        
        # Input frame
        self.input_frame = ttk.LabelFrame(main_frame, text="Input Parameters", padding="15")
        self.input_frame.grid(row=2, column=0, padx=15, pady=10, sticky="nsew")
        
        # Unit system selection
        ttk.Label(self.input_frame, text="Unit System:").grid(row=0, column=0, sticky="w", pady=8)
        self.unit_var = tk.StringVar(value="metric")
        ttk.Radiobutton(self.input_frame, text="Metric (mm, m/s)", 
                       variable=self.unit_var, value="metric", 
                       command=self.update_labels).grid(row=0, column=1, sticky="w", padx=10)
        ttk.Radiobutton(self.input_frame, text="Imperial (inches, ft/min)", 
                       variable=self.unit_var, value="imperial", 
                       command=self.update_labels).grid(row=1, column=1, sticky="w", padx=10)
        
        # Diameter input
        ttk.Label(self.input_frame, text="Pulley Diameter:").grid(row=2, column=0, sticky="w", pady=8)
        diameter_frame = ttk.Frame(self.input_frame)
        diameter_frame.grid(row=2, column=1, sticky="w")
        self.diameter_var = tk.DoubleVar(value=500)
        self.diameter_entry = ttk.Entry(diameter_frame, width=12, textvariable=self.diameter_var, font=("Arial", 11))
        self.diameter_entry.pack(side="left")
        self.diameter_unit_label = ttk.Label(diameter_frame, text="mm", width=8)
        self.diameter_unit_label.pack(side="left", padx=5)
        
        # RPS input/output row (will be updated based on mode)
        self.rps_label = ttk.Label(self.input_frame, text="Pulley RPS:")
        self.rps_label.grid(row=3, column=0, sticky="w", pady=8)
        rps_frame = ttk.Frame(self.input_frame)
        rps_frame.grid(row=3, column=1, sticky="w")
        self.rps_var = tk.DoubleVar(value=3.5)
        self.rps_entry = ttk.Entry(rps_frame, width=12, textvariable=self.rps_var, font=("Arial", 11))
        self.rps_entry.pack(side="left")
        ttk.Label(rps_frame, text="revolutions per second").pack(side="left", padx=5)
        
        # Belt speed input (only visible in reverse mode)
        self.speed_label = ttk.Label(self.input_frame, text="Desired Belt Speed:")
        self.speed_frame = ttk.Frame(self.input_frame)
        self.speed_var = tk.DoubleVar(value=5.5)
        self.speed_entry = ttk.Entry(self.speed_frame, width=12, textvariable=self.speed_var, font=("Arial", 11))
        self.speed_entry.pack(side="left")
        self.speed_unit_label = ttk.Label(self.speed_frame, text="m/s", width=8)
        self.speed_unit_label.pack(side="left", padx=5)
        
        # Initially hide belt speed input (forward mode default)
        self.speed_label.grid_remove()
        self.speed_frame.grid_remove()
        
        # Calculate button
        self.calculate_button = ttk.Button(self.input_frame, text="Calculate", command=self.calculate, width=15)
        self.calculate_button.grid(row=5, column=0, columnspan=2, pady=15)
        
        # Results frame with more space
        self.results_frame = ttk.LabelFrame(main_frame, text="Results", padding="15")
        self.results_frame.grid(row=2, column=1, padx=15, pady=10, sticky="nsew")
        
        # Circumference result
        ttk.Label(self.results_frame, text="Pulley Circumference:").grid(row=0, column=0, sticky="w", pady=8)
        self.circ_var = tk.StringVar(value="")
        ttk.Label(self.results_frame, textvariable=self.circ_var, width=20).grid(row=0, column=1, sticky="w")
        
        # Forward mode results
        self.forward_results_frame = ttk.Frame(self.results_frame)
        self.forward_results_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
        
        ttk.Label(self.forward_results_frame, text="Belt Speed:", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky="w", pady=(20, 8))
        
        self.ms_var = tk.StringVar(value="")
        self.kmh_var = tk.StringVar(value="")
        self.fpm_var = tk.StringVar(value="")
        
        ttk.Label(self.forward_results_frame, text="Meters per Second:").grid(row=1, column=0, sticky="w", pady=8)
        ttk.Label(self.forward_results_frame, textvariable=self.ms_var, width=20).grid(row=1, column=1, sticky="w")
        
        ttk.Label(self.forward_results_frame, text="Kilometers per Hour:").grid(row=2, column=0, sticky="w", pady=8)
        ttk.Label(self.forward_results_frame, textvariable=self.kmh_var, width=20).grid(row=2, column=1, sticky="w")
        
        ttk.Label(self.forward_results_frame, text="Feet per Minute:").grid(row=3, column=0, sticky="w", pady=8)
        ttk.Label(self.forward_results_frame, textvariable=self.fpm_var, width=20).grid(row=3, column=1, sticky="w")
        
        # Reverse mode results
        self.reverse_results_frame = ttk.Frame(self.results_frame)
        
        ttk.Label(self.reverse_results_frame, text="Required Pulley Speed:", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky="w", pady=(20, 8))
        
        self.rps_result_var = tk.StringVar(value="")
        self.rpm_result_var = tk.StringVar(value="")
        
        ttk.Label(self.reverse_results_frame, text="RPS (Revolutions per Second):").grid(row=1, column=0, sticky="w", pady=8)
        ttk.Label(self.reverse_results_frame, textvariable=self.rps_result_var, width=20).grid(row=1, column=1, sticky="w")
        
        ttk.Label(self.reverse_results_frame, text="RPM (Revolutions per Minute):").grid(row=2, column=0, sticky="w", pady=8)
        ttk.Label(self.reverse_results_frame, textvariable=self.rpm_result_var, width=20).grid(row=2, column=1, sticky="w")
        
        # Hide reverse results initially
        self.reverse_results_frame.grid_remove()
        
        # Visualization frame - larger size
        visual_frame = ttk.LabelFrame(main_frame, text="Conveyor Visualization", padding="15")
        visual_frame.grid(row=3, column=0, columnspan=2, padx=15, pady=15, sticky="nsew")
        
        # Add visualization using matplotlib - larger figure
        self.fig = Figure(figsize=(8, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=visual_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Formula explanation with better text display
        self.formula_frame = ttk.LabelFrame(main_frame, text="Formula", padding="15")
        self.formula_frame.grid(row=4, column=0, columnspan=2, padx=15, pady=10, sticky="ew")
        
        self.formula_label = ttk.Label(self.formula_frame, justify="left", font=("Arial", 11))
        self.formula_label.pack(anchor="w", fill="both")
        
        # Set row and column configurations for better resizing
        for i in range(5):
            main_frame.rowconfigure(i, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Make visualization row expand more
        main_frame.rowconfigure(3, weight=3)
        
        # Update formula text
        self.update_formula_text()
        
        # Initial drawing and calculation
        self.draw_conveyor()
        self.calculate()
    
    def update_mode(self):
        """Update UI based on selected calculation mode"""
        mode = self.mode_var.get()
        
        if mode == "forward":
            # Forward mode (RPS → Belt Speed)
            self.rps_label.config(text="Pulley RPS:")
            self.rps_entry.config(state="normal")
            
            # Show/hide the appropriate fields
            self.speed_label.grid_remove()
            self.speed_frame.grid_remove()
            
            # Show forward results, hide reverse results
            self.forward_results_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
            self.reverse_results_frame.grid_remove()
            
        else:
            # Reverse mode (Belt Speed → RPS)
            self.rps_label.config(text="Required RPS (calculated):")
            self.rps_entry.config(state="readonly")
            
            # Position and show belt speed input fields
            self.speed_label.grid(row=4, column=0, sticky="w", pady=8)
            self.speed_frame.grid(row=4, column=1, sticky="w")
            
            # Show reverse results, hide forward results
            self.forward_results_frame.grid_remove()
            self.reverse_results_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
        
        # Update formula text
        self.update_formula_text()
        
        # Recalculate
        self.calculate()
    
    def update_labels(self):
        """Update unit labels based on selected unit system"""
        if self.unit_var.get() == "metric":
            self.diameter_unit_label.config(text="mm")
            self.speed_unit_label.config(text="m/s")
        else:
            self.diameter_unit_label.config(text="inches")
            self.speed_unit_label.config(text="ft/min")
            
        # Update formula text
        self.update_formula_text()
        
        # Recalculate
        self.calculate()
    
    def update_formula_text(self):
        """Update the formula text based on calculation mode and unit system"""
        mode = self.mode_var.get()
        unit = self.unit_var.get()
        
        if mode == "forward":
            # Forward calculation formula
            formula_text = "Forward Calculation (RPS → Belt Speed):\n"
            formula_text += "Linear Velocity = π × Pulley Diameter × RPS\n\n"
            
            if unit == "metric":
                formula_text += "Where:\n"
                formula_text += "• Linear Velocity: belt speed in meters per second (m/s)\n"
                formula_text += "• Pulley Diameter: diameter in meters (input in mm, converted to m by dividing by 1000)\n"
                formula_text += "• RPS: rotational speed in Revolutions Per Second\n"
                formula_text += "• π: mathematical constant (approximately 3.14159)"
            else:
                formula_text += "Where:\n"
                formula_text += "• Linear Velocity: belt speed in feet per minute (ft/min)\n"
                formula_text += "• Pulley Diameter: diameter in feet (input in inches, converted to ft by dividing by 12)\n"
                formula_text += "• RPS: rotational speed in Revolutions Per Second\n"
                formula_text += "• π: mathematical constant (approximately 3.14159)"
        else:
            # Reverse calculation formula
            formula_text = "Reverse Calculation (Belt Speed → RPS):\n"
            formula_text += "RPS = Linear Velocity / (π × Pulley Diameter)\n\n"
            
            if unit == "metric":
                formula_text += "Where:\n"
                formula_text += "• RPS: required rotational speed in Revolutions Per Second\n"
                formula_text += "• Linear Velocity: desired belt speed in meters per second (m/s)\n"
                formula_text += "• Pulley Diameter: diameter in meters (input in mm, converted to m by dividing by 1000)\n"
                formula_text += "• π: mathematical constant (approximately 3.14159)"
            else:
                formula_text += "Where:\n"
                formula_text += "• RPS: required rotational speed in Revolutions Per Second\n"
                formula_text += "• Linear Velocity: desired belt speed in feet per minute (ft/min)\n"
                formula_text += "• Pulley Diameter: diameter in feet (input in inches, converted to ft by dividing by 12)\n"
                formula_text += "• π: mathematical constant (approximately 3.14159)"
        
        self.formula_label.config(text=formula_text)
    
    def draw_conveyor(self, speed=0):
        """Draw a clearer conveyor belt visualization"""
        self.ax.clear()
        
        # Draw improved conveyor components
        # Frame/supports
        self.ax.plot([1, 9], [1, 1], 'k-', linewidth=4)  # Bottom frame
        self.ax.plot([1, 1], [1, 2], 'k-', linewidth=4)  # Left support
        self.ax.plot([9, 9], [1, 2], 'k-', linewidth=4)  # Right support
        
        # Pulleys - larger and clearer
        tail_x, head_x = 2, 8
        pulley_y = 3
        pulley_r = 1.2
        
        tail_pulley = plt.Circle((tail_x, pulley_y), pulley_r, fill=False, color='blue', linewidth=3)
        head_pulley = plt.Circle((head_x, pulley_y), pulley_r, fill=False, color='blue', linewidth=3)
        self.ax.add_patch(tail_pulley)
        self.ax.add_patch(head_pulley)
        
        # Add details to pulleys (shaft)
        self.ax.plot([tail_x-0.3, tail_x+0.3], [pulley_y, pulley_y], 'k-', linewidth=2)
        self.ax.plot([head_x-0.3, head_x+0.3], [pulley_y, pulley_y], 'k-', linewidth=2)
        
        # Belt
        # Use curved lines for belt around pulleys using arcs
        
        # Top and bottom straight sections
        self.ax.plot([tail_x, head_x], [pulley_y + pulley_r, pulley_y + pulley_r], 'k-', linewidth=3)
        self.ax.plot([tail_x, head_x], [pulley_y - pulley_r, pulley_y - pulley_r], 'k-', linewidth=3)
        
        # Left arc (tail pulley)
        theta = np.linspace(np.pi/2, -np.pi/2, 30)
        x = tail_x + pulley_r * np.cos(theta)
        y = pulley_y + pulley_r * np.sin(theta)
        self.ax.plot(x, y, 'k-', linewidth=3)
        
        # Right arc (head pulley)
        theta = np.linspace(-np.pi/2, np.pi/2, 30)
        x = head_x + pulley_r * np.cos(theta)
        y = pulley_y + pulley_r * np.sin(theta)
        self.ax.plot(x, y, 'k-', linewidth=3)
        
        # Material on belt (optional)
        if speed > 0:
            for i in range(4):
                pos = tail_x + 1 + i * 1.2
                box_width = 0.8
                box_height = 0.5
                rect = plt.Rectangle((pos, pulley_y + pulley_r), box_width, box_height, 
                                    color='gray', alpha=0.7)
                self.ax.add_patch(rect)
        
        # Draw arrows to show direction and speed
        arrow_y = pulley_y + pulley_r + 0.8
        num_arrows = min(int(speed * 1.5) + 1, 8)
        if num_arrows < 1:
            num_arrows = 1
            
        arrow_spacing = (head_x - tail_x - 1) / num_arrows
        for i in range(num_arrows):
            pos = tail_x + 0.5 + (i * arrow_spacing)
            self.ax.arrow(pos, arrow_y, arrow_spacing * 0.7, 0, 
                         head_width=0.3, head_length=0.4, fc='red', ec='red', linewidth=2)
        
        # Add speed indicator text in a clearer box
        unit_text = "m/s" if self.unit_var.get() == "metric" else "ft/min"
        speed_text = f"Speed: {speed:.2f} {unit_text}"
        self.ax.text(5, pulley_y + pulley_r + 1.8, speed_text, ha='center', va='center',
                    fontsize=12, weight='bold', 
                    bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))
        
        # Add pulley rotation indicator
        if self.mode_var.get() == "forward":
            rps = self.rps_var.get()
        else:
            rps = float(self.rps_result_var.get().split()[0]) if self.rps_result_var.get() else 0
            
        if rps > 0:
            # Add rotation indicators to pulleys
            rotation_arrow_size = min(rps * 0.3, 0.9)  # Scale arrow size with RPS, limit to max size
            self.ax.arrow(head_x, pulley_y - 0.3, -rotation_arrow_size, 0, 
                         head_width=0.2, head_length=0.2, fc='green', ec='green', linewidth=2)
            self.ax.text(head_x, pulley_y - 0.7, f"{rps:.2f} RPS", ha='center', fontsize=10,
                        bbox=dict(facecolor='white', alpha=0.7))
        
        # Label pulleys
        self.ax.text(tail_x, pulley_y - pulley_r - 0.5, "Tail Pulley", ha='center', fontsize=10)
        self.ax.text(head_x, pulley_y - pulley_r - 0.5, "Head Pulley", ha='center', fontsize=10)
        
        # Configure plot appearance
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 7)
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        
        self.fig.tight_layout()
        self.canvas.draw()
    
    def calculate(self):
        """Calculate based on the selected mode"""
        try:
            # Get input values and validation
            diameter = self.diameter_var.get()
            unit_system = self.unit_var.get()
            mode = self.mode_var.get()
            
            if diameter <= 0:
                messagebox.showerror("Input Error", "Pulley diameter must be positive!")
                return
            
            # Calculate circumference
            if unit_system == "metric":
                # Diameter in mm to circumference in meters
                circumference = math.pi * diameter / 1000
                circ_unit = "m"
            else:
                # Diameter in inches to circumference in feet
                circumference = math.pi * diameter / 12
                circ_unit = "ft"
                
            # Update circumference display
            self.circ_var.set(f"{circumference:.3f} {circ_unit}")
            
            # Calculate based on mode
            if mode == "forward":
                # Forward calculation: RPS → Belt Speed
                rps = self.rps_var.get()
                
                if rps < 0:
                    messagebox.showerror("Input Error", "RPS cannot be negative!")
                    return
                
                if unit_system == "metric":
                    # Calculate linear velocity in m/s
                    linear_velocity_ms = circumference * rps
                    
                    # Convert to other units
                    linear_velocity_kmh = linear_velocity_ms * 3.6
                    linear_velocity_fpm = linear_velocity_ms * 196.85
                    
                    # Display in metric units
                    self.ms_var.set(f"{linear_velocity_ms:.2f} m/s")
                    self.kmh_var.set(f"{linear_velocity_kmh:.2f} km/h")
                    self.fpm_var.set(f"{linear_velocity_fpm:.2f} ft/min")
                    
                    speed_for_display = linear_velocity_ms
                    
                else:
                    # Calculate linear velocity in ft/min
                    linear_velocity_fpm = circumference * rps * 60
                    
                    # Convert to other units
                    linear_velocity_ms = linear_velocity_fpm / 196.85
                    linear_velocity_kmh = linear_velocity_ms * 3.6
                    
                    # Display in imperial units
                    self.ms_var.set(f"{linear_velocity_ms:.2f} m/s")
                    self.kmh_var.set(f"{linear_velocity_kmh:.2f} km/h")
                    self.fpm_var.set(f"{linear_velocity_fpm:.2f} ft/min")
                    
                    speed_for_display = linear_velocity_fpm / 20  # Scale down for display
                
            else:
                # Reverse calculation: Belt Speed → RPS
                if unit_system == "metric":
                    # Get desired speed in m/s
                    desired_speed = self.speed_var.get()
                    
                    if desired_speed <= 0:
                        messagebox.showerror("Input Error", "Belt speed must be positive!")
                        return
                    
                    # Calculate required RPS
                    required_rps = desired_speed / circumference
                    required_rpm = required_rps * 60
                    
                    # Update result display
                    self.rps_result_var.set(f"{required_rps:.2f} RPS")
                    self.rpm_result_var.set(f"{required_rpm:.2f} RPM")
                    
                    # Update RPS input field (now display only)
                    self.rps_var.set(required_rps)
                    
                    speed_for_display = desired_speed
                    
                else:
                    # Get desired speed in ft/min
                    desired_speed = self.speed_var.get()
                    
                    if desired_speed <= 0:
                        messagebox.showerror("Input Error", "Belt speed must be positive!")
                        return
                    
                    # Calculate required RPS (convert ft/min to ft/sec first)
                    required_rps = (desired_speed / 60) / circumference
                    required_rpm = required_rps * 60
                    
                    # Update result display
                    self.rps_result_var.set(f"{required_rps:.2f} RPS")
                    self.rpm_result_var.set(f"{required_rpm:.2f} RPM")
                    
                    # Update RPS input field (now display only)
                    self.rps_var.set(required_rps)
                    
                    speed_for_display = desired_speed / 20  # Scale down for display
            
            # Update visualization
            self.draw_conveyor(speed=speed_for_display)
            
        except Exception as e:
            messagebox.showerror("Calculation Error", f"An error occurred: {str(e)}")

# Run the application
if __name__ == "__main__":
    root = tk.Tk()
    app = ConveyorCalculatorApp(root)
    root.mainloop()