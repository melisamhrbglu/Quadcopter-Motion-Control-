# main.py
"""
Main application - integrates simulator, visualization, and UI
Quadcopter Motion Control Real-Time System
"""

import sys
import pygame
from controller import ManualController
from simulator import QuadcopterSimulator
from visualization import Visualizer
from ui import UserInterface, SimulationConfig, AppState


def main():
    """
    Main application loop
    """
    # Initialize Pygame
    pygame.init()
    
    # Load configuration
    config = SimulationConfig()
    
    # Initialize components
    ui = UserInterface(width=config.screen_width, height=config.screen_height)
    controller = ManualController()
    simulator = QuadcopterSimulator(
        mass=config.mass,
        length=config.length,
        Ixx=config.Ixx,
        Iyy=config.Iyy,
        Izz=config.Izz
    )
    visualizer = Visualizer(width=config.screen_width, height=config.screen_height)
    
    # Simulation variables
    dt = config.dt
    sim_time = 0.0
    step_count = 0
    
    # Initialize drone slightly higher to avoid ground contact
    initial_z = 3.5  # Daha yüksekten başla
    
    print("=" * 60)
    print("Quadcopter Motion Control - Real-Time Systems")
    print("=" * 60)
    print(f"Configuration:")
    print(f"  Mass: {config.mass} kg")
    print(f"  Time step: {dt} s ({1/dt:.0f} Hz)")
    print(f"  Screen: {config.screen_width}x{config.screen_height}")
    print(f"  Initial Altitude: {initial_z} m")
    print("=" * 60)
    print("\nControls:")
    print("  SPACE/SHIFT: Altitude up/down")
    print("  W/S: Pitch forward/backward")
    print("  A/D: Roll left/right")
    print("  Q/E: Yaw counter-clockwise/clockwise")
    print("  P: Pause/Resume")
    print("  R: Reset")
    print("  F1: Debug info")
    print("  ESC: Exit")
    print("=" * 60)
    print("\n[WAIT] Initializing drone...\n")
    
    # Initialize drone
    simulator.quad.z = initial_z
    simulator.z_ref = initial_z
    
    # Warm-up: Run 50 steps to stabilize
    print("[...] Stabilizing control system...")
    for _ in range(50):
        state = simulator.step(dt)
        sim_time += dt
        step_count += 1
    
    print(f"✓ Ready! Alt: {simulator.quad.z:.2f}m - Press SPACE to fly\n")
    
    # Main simulation loop
    while ui.is_running():
        # Handle events
        changes = ui.handle_events()
        
        # Handle state changes
        if changes["quit"]:
            break
        
        if changes["reset"]:
            simulator.reset()
            controller.reset()
            sim_time = 0.0
            step_count = 0
            print("Simulation reset!")
        
        if changes["pause_toggle"]:
            if ui.is_paused():
                print("Simulation paused")
            else:
                print("Simulation resumed")
        
        # Update simulation if not paused
        if not ui.is_paused():
            # Get user input and update references
            roll_ref, pitch_ref, yaw_ref, z_ref = controller.update_from_keyboard()
            simulator.set_references(roll_ref, pitch_ref, yaw_ref, z_ref)
            
            # Perform simulation step
            state = simulator.step(dt)
            sim_time += dt
            step_count += 1
        else:
            # Get current state without stepping
            state = simulator.get_extended_state()
        
        # Get extended state with references and errors
        state = simulator.get_extended_state()
        
        # Update visualization
        visualizer.update(state)
        
        # Get frame rate and control update frequency
        fps = ui.get_frame_rate(target_fps=int(1/dt))
        
        # Print statistics periodically
        if step_count % 100 == 0 and not ui.is_paused():
            print(f"Step: {step_count:6d} | Time: {sim_time:7.2f}s | "
                  f"Alt: {state['z']:6.2f}m | Roll: {state['roll']:6.3f}rad | "
                  f"Pitch: {state['pitch']:6.3f}rad | FPS: {fps:5.1f}")
        
        # Draw debug info if enabled
        if ui.show_debug_info:
            ui.draw_debug_info(sim_time, fps, step_count)
        
        # Draw pause overlay if paused
        if ui.is_paused():
            ui.draw_pause_overlay()
    
    # Cleanup
    print("\n" + "=" * 60)
    print(f"Simulation ended at:")
    print(f"  Time: {sim_time:.2f}s")
    print(f"  Steps: {step_count}")
    print(f"  Final altitude: {state['z']:.2f}m")
    print("=" * 60)
    
    pygame.quit()
    print("Thank you for using Quadcopter Motion Control!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        pygame.quit()
        sys.exit(1)