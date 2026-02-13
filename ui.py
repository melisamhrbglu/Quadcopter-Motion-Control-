# ui.py
"""
User interface and main application controller
"""

import pygame
from enum import Enum


class AppState(Enum):
    """Application state enumeration"""
    MENU = 1
    RUNNING = 2
    PAUSED = 3
    SETTINGS = 4
    EXIT = 5


class UserInterface:
    """
    Main application UI controller
    Manages application state and events
    """
    
    def __init__(self, width=1000, height=700):
        """
        Initialize UI
        
        Args:
            width: Window width
            height: Window height
        """
        self.width = width
        self.height = height
        self.running = True
        self.state = AppState.RUNNING
        
        # Display
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        
        # Fonts
        self.font_title = pygame.font.SysFont("Arial", 32, bold=True)
        self.font_normal = pygame.font.SysFont("Arial", 16)
        self.font_small = pygame.font.SysFont("Arial", 12)
        
        # Colors
        self.BG_COLOR = (20, 20, 20)
        self.TEXT_COLOR = (255, 255, 255)
        self.HIGHLIGHT_COLOR = (0, 200, 200)
        self.BUTTON_COLOR = (60, 60, 60)
        self.BUTTON_HOVER = (100, 100, 100)
        self.BUTTON_ACTIVE = (0, 150, 150)
        
        # Settings
        self.show_debug_info = False
        self.paused = False
        self.reset_requested = False

    def handle_events(self):
        """
        Handle user input and window events
        
        Returns:
            Dictionary of state changes
        """
        changes = {
            "pause_toggle": False,
            "reset": False,
            "quit": False,
            "debug_toggle": False
        }
        
        for event in pygame.event.get():
            # Window close
            if event.type == pygame.QUIT:
                self.running = False
                changes["quit"] = True
            
            # Keyboard input
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    changes["quit"] = True
                
                # Pause/Resume (P key)
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
                    changes["pause_toggle"] = True
                
                # Reset (R key)
                elif event.key == pygame.K_r:
                    changes["reset"] = True
                
                # Debug info toggle (F1)
                elif event.key == pygame.K_F1:
                    self.show_debug_info = not self.show_debug_info
                    changes["debug_toggle"] = True
        
        return changes

    def draw_menu(self):
        """Draw main menu"""
        self.screen.fill(self.BG_COLOR)
        
        title = self.font_title.render("Quadcopter Motion Control", True, self.HIGHLIGHT_COLOR)
        subtitle = self.font_normal.render("Real-Time Systems Project", True, self.TEXT_COLOR)
        
        self.screen.blit(title, (self.width//2 - title.get_width()//2, 100))
        self.screen.blit(subtitle, (self.width//2 - subtitle.get_width()//2, 150))
        
        instructions = [
            "CONTROLS:",
            "SPACE - Increase Altitude",
            "SHIFT - Decrease Altitude",
            "W/S - Pitch Forward/Backward",
            "A/D - Roll Left/Right",
            "Q/E - Yaw Counter-Clockwise/Clockwise",
            "P - Pause/Resume",
            "R - Reset",
            "ESC - Exit",
            "",
            "Press any key to start..."
        ]
        
        y_pos = 250
        for text in instructions:
            if text:
                surf = self.font_small.render(text, True, self.TEXT_COLOR)
                self.screen.blit(surf, (self.width//2 - surf.get_width()//2, y_pos))
            y_pos += 30
        
        pygame.display.update()

    def draw_pause_overlay(self):
        """Draw pause overlay"""
        # Semi-transparent overlay
        overlay = pygame.Surface((self.width, self.height))
        overlay.set_alpha(200)
        overlay.fill((0, 0, 0))
        self.screen.blit(overlay, (0, 0))
        
        # Pause text
        pause_text = self.font_title.render("PAUSED", True, self.HIGHLIGHT_COLOR)
        resume_text = self.font_normal.render("Press P to Resume", True, self.TEXT_COLOR)
        
        self.screen.blit(pause_text, 
                        (self.width//2 - pause_text.get_width()//2, 
                         self.height//2 - 50))
        self.screen.blit(resume_text,
                        (self.width//2 - resume_text.get_width()//2,
                         self.height//2 + 20))

    def draw_debug_info(self, sim_time, fps, step_count):
        """Draw debug information overlay"""
        debug_lines = [
            f"Time: {sim_time:.2f}s",
            f"Step: {step_count}",
            f"FPS: {fps:.1f}",
            f"Paused: {'Yes' if self.paused else 'No'}"
        ]
        
        y_pos = 10
        for line in debug_lines:
            text = self.font_small.render(line, True, self.HIGHLIGHT_COLOR)
            self.screen.blit(text, (10, y_pos))
            y_pos += 20

    def get_frame_rate(self, target_fps=20):
        """
        Get current frame rate and tick clock
        
        Args:
            target_fps: Target frames per second
            
        Returns:
            Actual FPS
        """
        self.clock.tick(target_fps)
        return self.clock.get_fps()

    def is_running(self):
        """Check if application is still running"""
        return self.running
    
    def is_paused(self):
        """Check if simulation is paused"""
        return self.paused
    
    def toggle_pause(self):
        """Toggle pause state"""
        self.paused = not self.paused


class SimulationConfig:
    """Configuration for simulation parameters"""
    
    def __init__(self):
        """Initialize default configuration"""
        # Quadcopter parameters
        self.mass = 1.0           # kg
        self.length = 0.3         # m
        self.Ixx = 0.01           # kg*m^2
        self.Iyy = 0.01           # kg*m^2
        self.Izz = 0.02           # kg*m^2
        
        # Simulation parameters
        self.dt = 0.05            # time step (50ms) - 20 Hz
        self.gravity = 9.81       # m/s^2
        self.max_sim_time = 600   # max simulation time (seconds)
        
        # Initial conditions
        self.initial_z = 2.5      # initial altitude (m)
        self.initial_vz = 0.0     # initial vertical velocity
        
        # Visualization
        self.screen_width = 1000  # pixels
        self.screen_height = 700  # pixels
        self.update_frequency = 20  # Hz (same as dt)

    def to_dict(self):
        """Convert to dictionary"""
        return {
            "mass": self.mass,
            "length": self.length,
            "Ixx": self.Ixx,
            "Iyy": self.Iyy,
            "Izz": self.Izz,
            "dt": self.dt,
            "gravity": self.gravity,
            "initial_z": self.initial_z
        }
