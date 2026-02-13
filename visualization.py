# visualization.py
"""
Visualization system for quadcopter FPV-style display
"""

import pygame
import math


class Visualizer:
    """
    Renders quadcopter state in FPV-style first-person view
    """
    
    def __init__(self, width=1000, height=700):
        """
        Initialize visualizer
        
        Args:
            width: Screen width (pixels)
            height: Screen height (pixels)
        """
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Quadcopter Motion Control - RTS")
        
        # Colors
        self.SKY_COLOR = (135, 206, 235)      # Light blue
        self.GROUND_COLOR = (100, 80, 60)     # Brown
        self.HORIZON_COLOR = (180, 140, 80)   # Tan
        self.TEXT_COLOR = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.WHITE = (255, 255, 255)
        self.CYAN = (0, 255, 255)
        
        # Fonts
        self.font_small = pygame.font.SysFont("monospace", 14)
        self.font_medium = pygame.font.SysFont("monospace", 16)
        self.font_large = pygame.font.SysFont("monospace", 20, bold=True)
        
        # State history
        self.history = []
        self.max_history = 15
        
        # Display scale
        self.pos_scale = 10  # pixels per meter

    def draw_background(self):
        """Draw sky and ground"""
        # Horizon position
        horizon = int(self.height * 0.6)
        
        # Sky
        pygame.draw.rect(self.screen, self.SKY_COLOR, (0, 0, self.width, horizon))
        
        # Horizon line
        pygame.draw.line(self.screen, self.HORIZON_COLOR, 
                        (0, horizon), (self.width, horizon), 2)
        
        # Ground
        pygame.draw.rect(self.screen, self.GROUND_COLOR,
                        (0, horizon, self.width, self.height - horizon))
        
        # Grid on ground
        grid_spacing = 50
        for x in range(0, self.width, grid_spacing):
            pygame.draw.line(self.screen, (80, 60, 40), (x, horizon), (x, self.height), 1)
        for y in range(horizon, self.height, grid_spacing):
            pygame.draw.line(self.screen, (80, 60, 40), (0, y), (self.width, y), 1)

    def draw_crosshair(self):
        """Draw center crosshair and drone model for FPV"""
        cx = self.width // 2
        cy = self.height // 2
        size = 30
        
        # Draw drone model (quadcopter top view) - BIGGER
        drone_size = 25  # Increased from 15
        # Main body (center square)
        pygame.draw.rect(self.screen, self.WHITE, 
                        (cx - drone_size//2, cy - drone_size//2, 
                         drone_size, drone_size), 3)  # Thicker border
        
        # Four motors (circles at corners) - BIGGER
        motor_offset = drone_size // 2 + 5  # Increased from 3
        motor_radius = 6  # Bigger motors
        motor_positions = [
            (cx - motor_offset, cy - motor_offset),  # Top-left
            (cx + motor_offset, cy - motor_offset),  # Top-right
            (cx - motor_offset, cy + motor_offset),  # Bottom-left
            (cx + motor_offset, cy + motor_offset),  # Bottom-right
        ]
        for pos in motor_positions:
            pygame.draw.circle(self.screen, self.GREEN, pos, motor_radius, 2)
        
        # Direction indicator (front of drone) - BIGGER
        pygame.draw.line(self.screen, self.BLUE, 
                        (cx, cy - drone_size//2), 
                        (cx, cy - motor_offset - 3), 4)  # Thicker
        
        # Crosshair lines
        pygame.draw.line(self.screen, self.RED, (cx - size, cy), (cx - motor_offset - 12, cy), 1)
        pygame.draw.line(self.screen, self.RED, (cx + motor_offset + 12, cy), (cx + size, cy), 1)
        pygame.draw.line(self.screen, self.RED, (cx, cy - size), (cx, cy - motor_offset - 12), 1)
        pygame.draw.line(self.screen, self.RED, (cx, cy + motor_offset + 12), (cx, cy + size), 1)

    def draw_attitude_indicator(self, roll, pitch):
        """Draw attitude indicator (attitude ball)"""
        x_start = 100
        y_start = 50
        radius = 45
        
        # Draw outer circle border (blue)
        pygame.draw.circle(self.screen, (0, 150, 255), (x_start, y_start), radius, 2)
        
        # Draw reference marks (cross)
        pygame.draw.line(self.screen, (100, 200, 255), 
                        (x_start - radius, y_start), (x_start + radius, y_start), 1)
        pygame.draw.line(self.screen, (100, 200, 255),
                        (x_start, y_start - radius), (x_start, y_start + radius), 1)
        
        # Draw center aircraft symbol (larger circle)
        pygame.draw.circle(self.screen, (0, 255, 100), (x_start, y_start), 5, 2)
        
        # Draw roll indicator line (rotated)
        line_length = radius * 0.8
        end_x = x_start + line_length * math.sin(roll)
        end_y = y_start - line_length * math.cos(roll)
        pygame.draw.line(self.screen, (255, 200, 0), (x_start, y_start), (end_x, end_y), 2)

    def draw_altimeter(self, altitude):
        """Draw vertical altitude bar on right side"""
        x = self.width - 60
        y_start = 80
        bar_height = 250
        bar_width = 35
        
        # Bar border (green)
        pygame.draw.rect(self.screen, (0, 255, 0),
                        (x - bar_width//2, y_start, bar_width, bar_height), 2)
        
        # Scale markings
        for i in range(0, 21):
            y = y_start + (bar_height * i // 20)
            alt = 10 - (i * 10 // 20)  # 10m to 0m
            
            if i % 5 == 0:
                pygame.draw.line(self.screen, (0, 255, 0), (x - bar_width//2 - 8, y),
                               (x - bar_width//2, y), 2)
                text = self.font_small.render(f"{alt}m", True, (0, 255, 0))
                self.screen.blit(text, (x - bar_width//2 - 35, y - 5))
            elif i % 2 == 0:
                pygame.draw.line(self.screen, (0, 150, 150), (x - bar_width//2 - 4, y),
                               (x - bar_width//2, y), 1)
        
        # Current altitude indicator (triangle on right)
        if altitude < 10:
            indicator_y = y_start + (bar_height * altitude / 10)
            pygame.draw.polygon(self.screen, (0, 255, 100),
                              [(x + bar_width//2 + 3, indicator_y - 6),
                               (x + bar_width//2 + 15, indicator_y),
                               (x + bar_width//2 + 3, indicator_y + 6)])
            
            # Altitude value display
            alt_text = self.font_medium.render(f"{altitude:.1f}m", True, (0, 255, 0))
            self.screen.blit(alt_text, (x - 70, indicator_y - 10))

    def draw_velocity_indicator(self, vx, vy, vz):
        """Draw velocity vectors on left side"""
        x_start = 20
        y_start = self.height - 100
        
        speed_h = math.sqrt(vx**2 + vy**2)
        
        # Background box
        pygame.draw.rect(self.screen, (30, 30, 30), (x_start - 5, y_start - 60, 140, 80), 0)
        pygame.draw.rect(self.screen, (100, 200, 100), (x_start - 5, y_start - 60, 140, 80), 2)
        
        # Title
        title = self.font_small.render("VELOCITY", True, (0, 255, 0))
        self.screen.blit(title, (x_start + 5, y_start - 55))
        
        # Horizontal velocity
        color_vh = (0, 255, 100) if speed_h < 5 else ((255, 200, 0) if speed_h < 15 else (255, 100, 0))
        text1 = self.font_small.render(f"Vh: {speed_h:5.2f} m/s", True, color_vh)
        self.screen.blit(text1, (x_start, y_start - 35))
        
        # Vertical velocity
        color_vz = (100, 200, 255) if vz > 0 else ((255, 100, 100) if vz < -0.1 else (100, 255, 100))
        text2 = self.font_small.render(f"Vz: {vz:+5.2f} m/s", True, color_vz)
        self.screen.blit(text2, (x_start, y_start - 18))

    def draw_status_panel(self, state):
        """Draw main status panel with telemetry"""
        panel_x = 10
        panel_y = self.height - 230  # Moved higher to make room for history
        panel_width = 320
        panel_height = 110  # Reduced height
        
        # Panel background
        pygame.draw.rect(self.screen, (20, 20, 20), 
                        (panel_x, panel_y, panel_width, panel_height), 0)
        pygame.draw.rect(self.screen, (0, 200, 0),
                        (panel_x, panel_y, panel_width, panel_height), 2)
        
        # Title
        title = self.font_medium.render("◆ TELEMETRY ◆", True, (0, 255, 0))
        self.screen.blit(title, (panel_x + 10, panel_y + 3))
        
        # Draw status lines with proper spacing
        y_offset = 20
        line_height = 15
        
        # Row 1: Altitude
        text = self.font_small.render(
            f"Alt: {state['z']:.2f}m  Ref: {state['z_ref']:.2f}m  Vz: {state['vz']:+.2f}m/s", 
            True, (0, 255, 0))
        self.screen.blit(text, (panel_x + 10, panel_y + y_offset))
        y_offset += line_height
        
        # Row 2: Position
        text = self.font_small.render(
            f"Pos: X={state['x']:+.1f}m  Y={state['y']:+.1f}m", 
            True, (100, 200, 255))
        self.screen.blit(text, (panel_x + 10, panel_y + y_offset))
        y_offset += line_height
        
        # Row 3: Roll
        text = self.font_small.render(
            f"Roll: {math.degrees(state['roll']):+6.1f}°  Ref: {math.degrees(state['roll_ref']):+6.1f}°", 
            True, (255, 100, 100))
        self.screen.blit(text, (panel_x + 10, panel_y + y_offset))
        y_offset += line_height
        
        # Row 4: Pitch
        text = self.font_small.render(
            f"Pitch: {math.degrees(state['pitch']):+6.1f}°  Ref: {math.degrees(state['pitch_ref']):+6.1f}°", 
            True, (255, 100, 100))
        self.screen.blit(text, (panel_x + 10, panel_y + y_offset))
        y_offset += line_height
        
        # Row 5: Yaw + Thrust combined
        text = self.font_small.render(
            f"Yaw: {math.degrees(state['yaw']):+6.1f}°  Thrust: {state['thrust']:.2f}N  Time: {state['sim_time']:.2f}s", 
            True, (200, 200, 200))
        self.screen.blit(text, (panel_x + 10, panel_y + y_offset))

    def draw_history(self):
        """Draw state history at bottom right"""
        history_width = 500
        history_height = 75
        history_x = self.width - history_width - 10
        history_y = self.height - history_height - 10
        
        if not self.history:
            return
        
        # Background
        pygame.draw.rect(self.screen, (20, 20, 20),
                        (history_x, history_y, history_width, history_height), 0)
        pygame.draw.rect(self.screen, (0, 200, 200),
                        (history_x, history_y, history_width, history_height), 2)
        
        # Title
        title = self.font_small.render("◆ HISTORY (Last 10) ◆", True, (0, 255, 255))
        self.screen.blit(title, (history_x + 5, history_y + 3))
        
        # Display in horizontal format - 10 columns (compact)
        y_offset = 18
        col_width = history_width // 10
        
        for i, entry in enumerate(self.history[-10:]):
            col = i
            x_pos = history_x + (col * col_width)
            y_pos = history_y + y_offset
            
            text = self.font_small.render(
                f"{i}: z={entry['z']:.1f}",
                True, (0, 255, 200)
            )
            self.screen.blit(text, (x_pos + 2, y_pos))

    def draw_info_bar(self):
        """Draw information bar at top"""
        info_y = 5
        
        info_text = "SPACE: Alt↑  |  SHIFT: Alt↓  |  W/S: Pitch  |  A/D: Roll  |  Q/E: Yaw  |  R: Reset  |  P: Pause  |  ESC: Exit"
        text = self.font_small.render(info_text, True, (255, 255, 0))
        
        # Background
        pygame.draw.rect(self.screen, (30, 30, 0), (0, 0, self.width, 20), 0)
        pygame.draw.line(self.screen, (200, 200, 0), (0, 19), (self.width, 19), 1)
        self.screen.blit(text, (10, info_y))

    def update(self, state):
        """
        Update display with current state
        
        Args:
            state: Current quadcopter state
        """
        self.screen.fill((0, 0, 0))
        
        # Draw scene
        self.draw_background()
        self.draw_crosshair()
        self.draw_attitude_indicator(state["roll"], state["pitch"])
        self.draw_altimeter(state["z"])
        self.draw_velocity_indicator(state["vx"], state["vy"], state["vz"])
        
        # Draw telemetry and info
        self.draw_status_panel(state)
        self.draw_info_bar()
        
        # Update history
        self.history.append(state.copy())
        if len(self.history) > self.max_history:
            self.history.pop(0)
        self.draw_history()
        
        # Update display
        pygame.display.update()

    def clear(self):
        """Clear display"""
        self.screen.fill((0, 0, 0))
        pygame.display.update()