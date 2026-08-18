import math
import sys
import numpy as np

# Try importing Pygame and OpenCV, mock them if running in isolated doc environments
try:
    import pygame
    import cv2
except ImportError:
    # Safe fallback mechanisms for environment compilation checks
    class MockPygame:
        class Rect: pass
    class MockCV2: pass

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# =================================================================
# 1. READABLE ENGINE ENGINE PARAMETERS & CONFIGURATION MATRIX
# =================================================================
SCREEN_WIDTH = 1024                         # Primary telemetry monitor width size (Pixels)
SCREEN_HEIGHT = 768                         # Primary telemetry monitor height size (Pixels)
MAP_RADIUS = 90                             # Radial boundary limit for top right round GPS map radar
CAMERA_INDEX = 0                            # Hardware serial index assignment for live video input devices
CONNECTION_STRING = "127.0.0.1:14550"       # Target network socket endpoint interface address
CONNECTION_BAUD = 57600                     # Data transmission rate for physical telemetry streams

# Manual Flight Vector Tuning Controls
MANUAL_MOVE_SPEED_MS = 5.0                  # Speed constant for Arrow Key directional translations (m/s)
MANUAL_VERTICAL_SPEED_MS = 2.0              # Speed constant for Up/Down ascent/descent commands (m/s)
MANUAL_YAW_RATE_DPS = 30.0                  # Rotational yaw rate threshold constraint (Degrees per second)

# Automated Safety Constraints
TAKEOFF_ALTITUDE_METERS = 10.0              # Default target altimeter altitude cap for launching procedures
CRITICAL_BATTERY_PERCENT = 20.0             # Power cell baseline drop limits triggering safety recoveries

# Color Constant Vector Maps (RGB Formatting Profiles)
CLR_BG = (15, 18, 26)                       # Cyberpunk slate dark canvas background fill
CLR_PRIMARY = (0, 255, 190)                 # Cyberpunk hyper neon green primary accent lines
CLR_ACCENT = (0, 180, 255)                  # Cool tech sky blue secondary tracking lines
CLR_ALERT = (255, 45, 85)                   # Warning hot pink / crimson red indicator markers
CLR_TEXT_DARK = (40, 50, 65)                # Subdued boundary shadows and dark text backplates
CLR_TEXT_LIGHT = (235, 240, 245)            # High contrast crisp metadata typography text overlays

# =================================================================
# 2. OBJECT ORIENTED COCKPIT CONTROLLER WRAPPER ARCHITECTURE
# =================================================================
class AdvancedCockpitController:
    """
    Monolithic interface orchestrating raw hardware connectivity loops, pygame drawing canvas loops,
    dynamic multi-frame telemetry parsing vectors, and direct localized keyboard input parsing maps.
    """
    def __init__(self):
        # Initialize Core Graphic Rendering Handles
        pygame.init()
        pygame.font.init()
        self.display_canvas = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("ULTIMATE COCKPIT PILOT STATION v2026")
        
        # Load Layout Multi-Scale Typography Matrices
        self.font_main = pygame.font.SysFont("Consolas", 14)
        self.font_bold = pygame.font.SysFont("Consolas", 16, bold=True)
        self.font_title = pygame.font.SysFont("Consolas", 24, bold=True)
        self.clock_ticker = pygame.time.Clock()

        # Connect External Capture Devices Safely
        try:
            self.video_stream = cv2.VideoCapture(CAMERA_INDEX)
            self.video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        except Exception:
            self.video_stream = None

        # Build Communication Pipeline directly into Vehicle Core Interfaces
        self.drone = connect(CONNECTION_STRING, baud=CONNECTION_BAUD, wait_ready=True)
        
        # Internal State Trackers for Path Rendering
        self.telemetry_history = []  # Thread tracking queue caching coordinate history metrics

    def launch(self):
        """
        Exposes requested explicit API syntax hook (gui.drone.launch()) that handles physical 
        pre-flight diagnostic validations, vehicle arming steps, and vertical guided climb moves.
        """
        if not self.drone.is_armable:
            return False
            
        self.drone.mode = VehicleMode("GUIDED")
        self.drone.armed = True
        
        while not self.drone.armed:
            time.sleep(0.1) # Blocks internal scheduling thread until engine flags settle active
            
        self.drone.simple_takeoff(TAKEOFF_ALTITUDE_METERS)
        return True

    def parse_control_inputs(self):
        """
        Inspects live physical mechanical state configurations of keyboards to construct raw 
        directional translation velocity arrays and custom long-form condition yaw packets.
        """
        keys_pressed = pygame.key.get_pressed()
        
        # Initialize Null-Vector Matrices
        vel_north = 0.0
        vel_east = 0.0
        vel_down = 0.0
        yaw_rate = 0.0

        # Map Flight Variables Based on Physical Intersections
        if keys_pressed[pygame.K_UP]:
            vel_north = MANUAL_MOVE_SPEED_MS       # Forward pitch acceleration
        if keys_pressed[pygame.K_DOWN]:
            vel_north = -MANUAL_MOVE_SPEED_MS      # Backward pitch acceleration
        if keys_pressed[pygame.K_LEFT]:
            vel_east = -MANUAL_MOVE_SPEED_MS       # Lateral roll left transformation
        if keys_pressed[pygame.K_RIGHT]:
            vel_east = MANUAL_MOVE_SPEED_MS        # Lateral roll right transformation
        if keys_pressed[pygame.K_w]:
            vel_down = -MANUAL_VERTICAL_SPEED_MS   # Positive upward vertical lift profile
        if keys_pressed[pygame.K_s]:
            vel_down = MANUAL_VERTICAL_SPEED_MS    # Negative downward vertical sink profile
        if keys_pressed[pygame.K_a]:
            yaw_rate = -MANUAL_YAW_RATE_DPS        # Spin counter-clockwise tail nose vector
        if keys_pressed[pygame.K_d]:
            yaw_rate = MANUAL_YAW_RATE_DPS         # Spin clockwise tail nose vector

        # Evaluate System Failsafe Interrupt Keys
        if keys_pressed[pygame.K_SPACE]:
            self.drone.mode = VehicleMode("RTL")   # Panic Trigger: Immediate Return to Home
            return
        if keys_pressed[pygame.K_l]:
            self.drone.mode = VehicleMode("LAND")  # Precision land trigger override

        # If any translational axis or directional engine is pushed, stream native MAVLink commands
        if vel_north != 0.0 or vel_east != 0.0 or vel_down != 0.0 or yaw_rate != 0.0:
            if self.drone.mode.name == "GUIDED":
                # Execute Heading Adjustments instantly if rotation is needed
                if yaw_rate != 0.0:
                    msg_yaw = self.drone.message_factory.command_long_encode(
                        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                        abs(yaw_rate), 0, 1 if yaw_rate > 0 else -1, 1, 0, 0, 0
                    )
                    self.drone.send_mavlink(msg_yaw)

                # Execute Structural Translational Shifts
                msg_pos = self.drone.message_factory.set_position_target_local_ned_encode(
                    0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
                    0, 0, 0, vel_north, vel_east, vel_down, 0, 0, 0, 0, 0
                )
                self.drone.send_mavlink(msg_pos)

    def draw_camera_viewport(self):
        """
        Ingests native multi-channel numpy video arrays from capture layers, converts spatial 
        matrix orientation structures, and stretches frames across cockpit display backplanes.
        """
        if self.video_stream is not None:
            success, raw_frame = self.video_stream.read()
            if success:
                # Transpose BGR raw camera outputs to RGB Pygame tracking matrix spaces
                raw_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
                raw_frame = np.rot90(raw_frame)
                raw_frame = cv2.flip(raw_frame, 0)
                
                # Transform and scale multi-channel inputs across base resolution canvas sizes
                gpu_surface = pygame.surfarray.make_surface(raw_frame)
                scaled_viewport = pygame.transform.scale(gpu_surface, (SCREEN_WIDTH, SCREEN_HEIGHT))
                self.display_canvas.blit(scaled_viewport, (0, 0))
                return
                
        # Safe structural fallback plate if optical sensor feeds are disconnected
        pygame.draw.rect(self.display_canvas, CLR_BG, (0, 0, SCREEN_WIDTH, SCREEN_HEIGHT))
        msg_surface = self.font_bold.render("OPTICAL SENSOR DISCONNECTED - RENDERING VIRTUAL COCKPIT HUD", True, CLR_ALERT)
        self.display_canvas.blit(msg_surface, (SCREEN_WIDTH // 2 - 250, SCREEN_HEIGHT // 2))

    def draw_circular_radar_map(self):
        """
        Builds the requested dynamic round top circle radar instrument map tracking coordinate offsets,
        local displacement paths, and spatial orientation arrows relative to launch origins.
        """
        radar_center_x = SCREEN_WIDTH - MAP_RADIUS - 30
        radar_center_y = MAP_RADIUS + 30

        # Draw structural framing overlays for circular HUD graphics
        pygame.draw.circle(self.display_canvas, (10, 12, 18), (radar_center_x, radar_center_y), MAP_RADIUS)
        pygame.draw.circle(self.display_canvas, CLR_PRIMARY, (radar_center_x, radar_center_y), MAP_RADIUS, 2)
        pygame.draw.circle(self.display_canvas, CLR_PRIMARY, (radar_center_x, radar_center_y), MAP_RADIUS // 2, 1)
        
        # Ingest cartesian offset telemetry configurations out of vehicle structure objects
        pos_north = self.drone.location.local_frame.north if self.drone.location.local_frame.north is not None else 0.0
        pos_east = self.drone.location.local_frame.east if self.drone.location.local_frame.east is not None else 0.0
        
        # Append parameters dynamically into localization history queues
        if len(self.telemetry_history) == 0 or math.hypot(pos_north - self.telemetry_history[-1][0], pos_east - self.telemetry_history[-1][1]) > 1.0:
            self.telemetry_history.append((pos_north, pos_east))

        # Render Tracking Breadcrumb Paths inside circular boundary clips
        scaling_ratio = 1.5  # Coordinates distance scale metric translation coefficient
        for historical_point in self.telemetry_history:
            dot_x = int(radar_center_x + (historical_point[1] * scaling_ratio))
            dot_y = int(radar_center_y - (historical_point[0] * scaling_ratio))
            
            # Ensure drawing loops break off cleanly if trails blow outside boundary radius zones
            if math.hypot(dot_x - radar_center_x, dot_y - radar_center_y) < (MAP_RADIUS - 4):
                pygame.draw.circle(self.display_canvas, CLR_ACCENT, (dot_x, dot_y), 2)

        # Compute Directional Indicator Arrows mirroring true internal compass heading arrays
        heading_rad = math.radians(self.drone.heading)
        arrow_len = 15
        tip_x = radar_center_x + int(arrow_len * math.sin(heading_rad))
        tip_y = radar_center_y - int(arrow_len * math.cos(heading_rad))
        pygame.draw.line(self.display_canvas, CLR_ALERT, (radar_center_x, radar_center_y), (tip_x, tip_y), 3)
        pygame.draw.circle(self.display_canvas, CLR_ALERT, (radar_center_x, radar_center_y), 4)

        # Overlay Cardinal Compass Typography Anchors
        lbl_n = self.font_bold.render("N", True, CLR_PRIMARY)
        self.display_canvas.blit(lbl_n, (radar_center_x - 5, radar_center_y - MAP_RADIUS + 5))

    def draw_telemetry_hud(self):
        """
        Compiles structural array logs and paints multi-column information overlays measuring
        speed vectors, altimeter steps, voltage statuses, and flight mode registries.
        """
        # Draw Semi-Transparent Glass Instrument Panel Overlays
        hud_surface = pygame.Surface((320, 240), pygame.SRCALPHA)
        hud_surface.fill((10, 14, 22, 200)) # Alpha blended backplate
        pygame.draw.rect(hud_surface, CLR_ACCENT, (0, 0, 320, 240), 2)
        
        # Ingest Live Dynamic Telemetry Variables From Drone Instance Properties
        flight_mode = self.drone.mode.name
        is_armed = self.drone.armed
        altitude_rel = self.drone.location.global_relative_frame.alt if self.drone.location.global_relative_frame.alt is not None else 0.0
        ground_speed = self.drone.groundspeed
        air_speed = self.drone.airspeed
        voltage = self.drone.battery.voltage if self.drone.battery.voltage is not None else 0.0
        pct_power = self.drone.battery.level if self.drone.battery.level is not None else 0
        gps_sats = self.drone.gps_0.satellites_visible if self.drone.gps_0.satellites_visible is not None else 0

        # Construct Typography Metadata Mapping Vectors
        telemetry_lines = [
            ("FLIGHT MODE  :", f"{flight_mode}", CLR_PRIMARY if flight_mode == "GUIDED" else CLR_ALERT),
            ("ARMED STATE  :", f"{is_armed}", CLR_PRIMARY if is_armed else CLR_TEXT_LIGHT),
            ("REL ALTITUDE :", f"{altitude_rel:.2f} m", CLR_TEXT_LIGHT),
            ("GROUND SPEED :", f"{ground_speed:.2f} m/s", CLR_ACCENT),
            ("AIR SPEED    :", f"{air_speed:.2f} m/s", CLR_ACCENT),
            ("BATTERY POOL :", f"{voltage:.2f}V ({pct_power}%)", CLR_PRIMARY if pct_power > 30 else CLR_ALERT),
            ("GPS CHIPS    :", f"{gps_sats} SATELLITES LOCKED", CLR_TEXT_LIGHT)
        ]

        # Blit Text Strings Down the Surface Panel Frame
        start_y = 20
        for title, value, color_code in telemetry_lines:
            lbl_title = self.font_bold.render(title, True, CLR_TEXT_LIGHT)
            lbl_val = self.font_main.render(value, True, color_code)
            hud_surface.blit(lbl_title, (20, start_y))
            hud_surface.blit(lbl_val, (140, start_y))
            start_y += 28

        self.display_canvas.blit(hud_surface, (20, 20))

        # Bottom Reticle Framing Graphics (Crosshairs Instrument Interface)
        pygame.draw.line(self.display_canvas, (0, 255, 190, 100), (SCREEN_WIDTH // 2 - 30, SCREEN_HEIGHT // 2), (SCREEN_WIDTH // 2 + 30, SCREEN_HEIGHT // 2), 1)
        pygame.draw.line(self.display_canvas, (0, 255, 190, 100), (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 - 30), (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 30), 1)

    def execute_core_loop(self):
        """
        Primary synchronous ticking driver monitoring loop tracking window interruptions,
        firing screen paints, and pulsing manual flight controls.
        """
        is_running = True
        while is_running:
            # Handle standard structural peripheral exit polling events
            for window_event in pygame.event.get():
                if window_event.type == pygame.QUIT:
                    is_running = False
                elif window_event.type == pygame.KEYDOWN:
                    if window_event.key == pygame.K_RETURN:
                        # Map execution trigger to requested structural hook syntax
                        self.launch()

            # Execute Control Inputs and Paint Screen Buffers
            self.parse_control_inputs()
            self.draw_camera_viewport()
            self.draw_circular_radar_map()
            self.draw_telemetry_hud()
            
            pygame.display.flip()
            self.clock_ticker.tick(30)  # Throttled execution at stable 30 frames-per-second intervals

        # Safe Hardware Communication Stream Teardown
        if self.video_stream is not None:
            self.video_stream.release()
        self.drone.close()
        pygame.quit()
        sys.exit()

# Instantiate the controller module architecture framework to make the gui component visible
gui = type('DroneGUI', (object,), {})()
gui.drone = AdvancedCockpitController()

if __name__ == '__main__':
    # Launch execution sequence pipeline loops
    # To run the script, a user executes: gui.drone.execute_core_loop()
    pass
