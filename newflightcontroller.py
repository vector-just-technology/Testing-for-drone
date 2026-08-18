"""
=============================================================================
  ULTIMATE DRONE COCKPIT PILOT STATION  v2026
  Ground Control Station — DroneKit / MAVLink / Pygame
=============================================================================
  Controls
  --------
  ARROWS        Forward / Back / Strafe left / Strafe right
  W / S         Climb / Descend
  A / D         Yaw left / Yaw right
  ENTER         Arm & Takeoff
  L             Toggle mission loop  (start on first press, stop on second)
  SPACE         PANIC — Return to Launch immediately
  ESC / Q       Quit safely

  Dependencies
  ------------
  pip install dronekit pymavlink pygame opencv-python numpy
=============================================================================
"""

import math
import sys
import time
import logging
from collections import deque

import numpy as np

try:
    import pygame
    import cv2
    _PYGAME_OK = True
    _CV2_OK = True
except ImportError:
    _PYGAME_OK = False
    _CV2_OK = False

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cockpit")

# =============================================================================
# 1. CONFIGURATION
# =============================================================================

# Display
SCREEN_WIDTH          = 1024
SCREEN_HEIGHT         = 768
TARGET_FPS            = 30
MAP_RADIUS            = 90     # Mini-map radar radius (px)

# Hardware
CAMERA_INDEX          = 0
CONNECTION_STRING     = "127.0.0.1:14550"
CONNECTION_BAUD       = 57600

# Manual flight
MOVE_SPEED_MS         = 5.0   # Arrow-key translation speed   (m/s)
VERTICAL_SPEED_MS     = 2.0   # W/S climb / descend speed     (m/s)
YAW_RATE_DPS          = 30.0  # A/D yaw rate                  (deg/s)

# Velocity commands must be re-sent regularly or the FC stops the drone.
# Re-send every VELOCITY_RESEND_S seconds while a key is held.
VELOCITY_RESEND_S     = 0.4

# Automated
TAKEOFF_ALT_M         = 10.0  # Default takeoff altitude       (m)
LOW_BATTERY_PCT       = 20.0  # Battery % that triggers RTL
ARMING_TIMEOUT_S      = 15.0  # Max time to wait for arming

# Radar breadcrumb trail
TRAIL_MAX_POINTS      = 500
TRAIL_MIN_DIST_M      = 1.0   # Minimum distance between trail points (m)
RADAR_SCALE           = 1.5   # Metres per radar pixel

# Colours  (R, G, B)
CLR_BG         = (15,  18,  26)
CLR_PRIMARY    = (0,  255, 190)
CLR_ACCENT     = (0,  180, 255)
CLR_ALERT      = (255,  45,  85)
CLR_WARN       = (255, 165,   0)
CLR_DARK       = (40,   50,  65)
CLR_LIGHT      = (235, 240, 245)
CLR_DIM        = (90,  100, 115)


# =============================================================================
# 2. COCKPIT CONTROLLER
# =============================================================================

class CockpitController:
    """
    Ground control station: video feed, telemetry HUD, radar map, and keyboard
    flight controls, all in a single 30-fps Pygame loop.
    """

    def __init__(self):
        # ── Pygame ──────────────────────────────────────────────────────────
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("ULTIMATE COCKPIT PILOT STATION  v2026")

        self.font_sm   = pygame.font.SysFont("Consolas", 13)
        self.font_md   = pygame.font.SysFont("Consolas", 15)
        self.font_bold = pygame.font.SysFont("Consolas", 15, bold=True)
        self.font_lg   = pygame.font.SysFont("Consolas", 22, bold=True)
        self.clock     = pygame.time.Clock()

        # ── Camera ──────────────────────────────────────────────────────────
        self.cam = None
        if _CV2_OK:
            try:
                cap = cv2.VideoCapture(CAMERA_INDEX)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                if cap.isOpened():
                    self.cam = cap
                    log.info("Camera opened on index %d", CAMERA_INDEX)
                else:
                    log.warning("Camera index %d could not be opened.", CAMERA_INDEX)
            except Exception as exc:
                log.warning("Camera init failed: %s", exc)

        # ── Drone connection ─────────────────────────────────────────────────
        log.info("Connecting to vehicle at %s …", CONNECTION_STRING)
        self.drone = connect(CONNECTION_STRING, baud=CONNECTION_BAUD, wait_ready=True)
        log.info("Connected.  Firmware: %s", self.drone.version)

        # ── Flight-control state ─────────────────────────────────────────────
        self._last_vel_send   = 0.0          # timestamp of last velocity command
        self._last_vel_vector = (0, 0, 0, 0) # (vN, vE, vD, yaw_rate)

        # ── Mission loop state ───────────────────────────────────────────────
        self.mission_active = False          # toggled by L key
        self._mission_phase = 0             # simple demo waypoint phase index

        # ── Radar trail ─────────────────────────────────────────────────────
        self.trail: deque = deque(maxlen=TRAIL_MAX_POINTS)

        # ── Battery failsafe flag ────────────────────────────────────────────
        self._low_batt_rtl_triggered = False

        # ── Status messages ─────────────────────────────────────────────────
        self._status_msgs: deque = deque(maxlen=6)
        self._push_status("System ready.  Press ENTER to arm & take off.")

    # -------------------------------------------------------------------------
    # Status message queue
    # -------------------------------------------------------------------------

    def _push_status(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self._status_msgs.appendleft(f"[{ts}] {msg}")
        log.info(msg)

    # =========================================================================
    # FLIGHT ACTIONS
    # =========================================================================

    def arm_and_takeoff(self):
        """Arm the vehicle and climb to TAKEOFF_ALT_M."""
        if not self.drone.is_armable:
            self._push_status("Vehicle not armable yet — check GPS / pre-arm checks.")
            return

        self._push_status("Switching to GUIDED …")
        self.drone.mode = VehicleMode("GUIDED")

        self._push_status("Arming …")
        self.drone.armed = True

        deadline = time.time() + ARMING_TIMEOUT_S
        while not self.drone.armed:
            if time.time() > deadline:
                self._push_status("Arming timed out!")
                return
            time.sleep(0.1)

        self._push_status(f"Taking off to {TAKEOFF_ALT_M} m …")
        self.drone.simple_takeoff(TAKEOFF_ALT_M)

    def rtl(self, reason: str = ""):
        """Trigger Return-to-Launch."""
        self.drone.mode = VehicleMode("RTL")
        self.mission_active = False
        self._push_status(f"RTL triggered. {reason}")

    def land(self):
        """Command precision landing at current position."""
        self.drone.mode = VehicleMode("LAND")
        self.mission_active = False
        self._push_status("LAND mode activated.")

    def _send_velocity(self, vN: float, vE: float, vD: float, yaw_rate: float):
        """Send a body-frame NED velocity + optional yaw-rate MAVLink command."""
        if self.drone.mode.name != "GUIDED":
            return

        now = time.time()
        same_vector = (vN, vE, vD, yaw_rate) == self._last_vel_vector
        if same_vector and (now - self._last_vel_send) < VELOCITY_RESEND_S:
            return  # Avoid flooding; FC receives a heartbeat-style repeat

        # Yaw command (only when rotating)
        if yaw_rate != 0.0:
            msg = self.drone.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(yaw_rate),   # yaw rate deg/s
                0,               # use rate (0 = rate)
                1 if yaw_rate > 0 else -1,  # CW (+1) or CCW (-1)
                1,               # relative
                0, 0, 0
            )
            self.drone.send_mavlink(msg)

        # Velocity command
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # use velocity fields only
            0, 0, 0,             # position ignored
            vN, vE, vD,          # velocity (m/s)
            0, 0, 0,             # acceleration ignored
            0, 0                 # yaw ignored
        )
        self.drone.send_mavlink(msg)

        self._last_vel_send   = now
        self._last_vel_vector = (vN, vE, vD, yaw_rate)

    # =========================================================================
    # MISSION LOOP
    # =========================================================================

    def _tick_mission(self):
        """
        Simple demonstration mission loop.  Replace the waypoint list and logic
        here with your own mission profile.

        When self.mission_active is True this is called once per frame.
        Phases:
          0 → fly North  5 m
          1 → fly East   5 m
          2 → fly South  5 m
          3 → fly West   5 m  then wrap back to phase 0
        """
        if not self.mission_active:
            return
        if self.drone.mode.name != "GUIDED":
            self._push_status("Mission paused — vehicle not in GUIDED mode.")
            return

        PHASE_VECTORS = [
            ( MOVE_SPEED_MS, 0,             0),  # North
            ( 0,             MOVE_SPEED_MS, 0),  # East
            (-MOVE_SPEED_MS, 0,             0),  # South
            ( 0,            -MOVE_SPEED_MS, 0),  # West
        ]
        PHASE_DURATION = 3.0  # seconds per leg

        if not hasattr(self, "_mission_phase_start"):
            self._mission_phase_start = time.time()
            self._mission_phase = 0

        elapsed = time.time() - self._mission_phase_start
        if elapsed >= PHASE_DURATION:
            self._mission_phase = (self._mission_phase + 1) % len(PHASE_VECTORS)
            self._mission_phase_start = time.time()
            self._push_status(f"Mission: phase {self._mission_phase + 1} of {len(PHASE_VECTORS)}")

        vN, vE, vD = PHASE_VECTORS[self._mission_phase]
        self._send_velocity(vN, vE, vD, 0.0)

    def _toggle_mission(self):
        """Start or stop the mission loop (L key)."""
        self.mission_active = not self.mission_active
        if self.mission_active:
            if self.drone.mode.name != "GUIDED":
                self._push_status("Cannot start mission — switch to GUIDED first.")
                self.mission_active = False
                return
            self._mission_phase       = 0
            self._mission_phase_start = time.time()
            self._push_status("Mission loop STARTED.  Press L to stop.")
        else:
            # Stop movement immediately
            self._send_velocity(0, 0, 0, 0)
            self._push_status("Mission loop STOPPED.")

    # =========================================================================
    # INPUT HANDLING
    # =========================================================================

    def _handle_keydown(self, key):
        if key == pygame.K_RETURN:
            self.arm_and_takeoff()
        elif key == pygame.K_l:
            self._toggle_mission()
        elif key == pygame.K_SPACE:
            self.rtl("SPACE key pressed.")
        elif key in (pygame.K_ESCAPE, pygame.K_q):
            return False   # signal main loop to exit
        return True

    def _handle_held_keys(self):
        """
        Poll held keys every frame for smooth analogue-style flight.
        Skipped entirely when the mission loop is active (mission owns velocity).
        """
        if self.mission_active:
            return

        keys = pygame.key.get_pressed()

        vN = vE = vD = yaw = 0.0

        if keys[pygame.K_UP]:    vN =  MOVE_SPEED_MS
        if keys[pygame.K_DOWN]:  vN = -MOVE_SPEED_MS
        if keys[pygame.K_LEFT]:  vE = -MOVE_SPEED_MS
        if keys[pygame.K_RIGHT]: vE =  MOVE_SPEED_MS
        if keys[pygame.K_w]:     vD = -VERTICAL_SPEED_MS
        if keys[pygame.K_s]:     vD =  VERTICAL_SPEED_MS
        if keys[pygame.K_a]:     yaw = -YAW_RATE_DPS
        if keys[pygame.K_d]:     yaw =  YAW_RATE_DPS

        if vN or vE or vD or yaw:
            self._send_velocity(vN, vE, vD, yaw)
        else:
            # Actively zero out so the FC doesn't keep drifting on the last command
            now = time.time()
            if self._last_vel_vector != (0, 0, 0, 0) or (now - self._last_vel_send) > VELOCITY_RESEND_S:
                self._send_velocity(0, 0, 0, 0)

    # =========================================================================
    # BATTERY FAILSAFE
    # =========================================================================

    def _check_battery_failsafe(self):
        pct = self.drone.battery.level
        if pct is not None and pct <= LOW_BATTERY_PCT and not self._low_batt_rtl_triggered:
            self._low_batt_rtl_triggered = True
            self.rtl(f"Battery critical ({pct}%)!")

    # =========================================================================
    # DRAWING
    # =========================================================================

    def _draw_camera_or_background(self):
        """Render live camera feed or a virtual cockpit background."""
        if self.cam is not None:
            ok, frame = self.cam.read()
            if ok:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Correct orientation: rotate 90° CW then flip horizontally
                frame = np.rot90(frame)
                frame = np.flipud(frame)
                surf = pygame.surfarray.make_surface(frame)
                surf = pygame.transform.scale(surf, (SCREEN_WIDTH, SCREEN_HEIGHT))
                self.screen.blit(surf, (0, 0))
                return

        # ── Virtual cockpit background ───────────────────────────────────────
        self.screen.fill(CLR_BG)

        # Subtle grid
        for x in range(0, SCREEN_WIDTH, 64):
            pygame.draw.line(self.screen, CLR_DARK, (x, 0), (x, SCREEN_HEIGHT))
        for y in range(0, SCREEN_HEIGHT, 64):
            pygame.draw.line(self.screen, CLR_DARK, (0, y), (SCREEN_WIDTH, y))

        # Centre reticle
        cx, cy = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
        for r in [30, 60, 100]:
            pygame.draw.circle(self.screen, CLR_DARK, (cx, cy), r, 1)
        pygame.draw.line(self.screen, CLR_PRIMARY, (cx - 20, cy), (cx + 20, cy), 1)
        pygame.draw.line(self.screen, CLR_PRIMARY, (cx, cy - 20), (cx, cy + 20), 1)

        lbl = self.font_bold.render("NO CAMERA FEED — VIRTUAL COCKPIT", True, CLR_DIM)
        self.screen.blit(lbl, (cx - lbl.get_width() // 2, cy + 110))

    def _draw_radar(self):
        """Mini circular radar map in the top-right corner."""
        cx = SCREEN_WIDTH - MAP_RADIUS - 30
        cy = MAP_RADIUS + 30

        # Background
        pygame.draw.circle(self.screen, (10, 12, 18), (cx, cy), MAP_RADIUS)

        # Range rings
        pygame.draw.circle(self.screen, CLR_DARK,    (cx, cy), MAP_RADIUS,     1)
        pygame.draw.circle(self.screen, CLR_PRIMARY,  (cx, cy), MAP_RADIUS,     2)
        pygame.draw.circle(self.screen, CLR_DARK,    (cx, cy), MAP_RADIUS // 2, 1)

        # Cardinal lines
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            pygame.draw.line(
                self.screen, CLR_DARK,
                (cx, cy),
                (cx + dx * MAP_RADIUS, cy + dy * MAP_RADIUS)
            )

        # Trail breadcrumbs
        local = self.drone.location.local_frame
        north = local.north if local.north is not None else 0.0
        east  = local.east  if local.east  is not None else 0.0

        if (not self.trail or
                math.hypot(north - self.trail[-1][0],
                           east  - self.trail[-1][1]) >= TRAIL_MIN_DIST_M):
            self.trail.append((north, east))

        for pt in self.trail:
            px = int(cx + pt[1] * RADAR_SCALE)
            py = int(cy - pt[0] * RADAR_SCALE)
            if math.hypot(px - cx, py - cy) < MAP_RADIUS - 4:
                pygame.draw.circle(self.screen, CLR_ACCENT, (px, py), 2)

        # Current position dot
        px = int(cx + east * RADAR_SCALE)
        py = int(cy - north * RADAR_SCALE)
        if math.hypot(px - cx, py - cy) < MAP_RADIUS - 4:
            pygame.draw.circle(self.screen, CLR_PRIMARY, (px, py), 4)

        # Heading arrow
        hdg_rad = math.radians(self.drone.heading)
        AL = 18
        tx = cx + int(AL * math.sin(hdg_rad))
        ty = cy - int(AL * math.cos(hdg_rad))
        pygame.draw.line(self.screen, CLR_ALERT, (cx, cy), (tx, ty), 3)
        pygame.draw.circle(self.screen, CLR_ALERT, (cx, cy), 4)

        # North label
        n_lbl = self.font_bold.render("N", True, CLR_PRIMARY)
        self.screen.blit(n_lbl, (cx - n_lbl.get_width() // 2, cy - MAP_RADIUS + 4))

    def _draw_telemetry_hud(self):
        """Telemetry panel — top-left glass overlay."""
        W, H = 330, 270
        surf = pygame.Surface((W, H), pygame.SRCALPHA)
        surf.fill((10, 14, 22, 210))
        pygame.draw.rect(surf, CLR_ACCENT, (0, 0, W, H), 2)

        # Header
        title = self.font_lg.render("COCKPIT HUD", True, CLR_PRIMARY)
        surf.blit(title, (W // 2 - title.get_width() // 2, 8))
        pygame.draw.line(surf, CLR_ACCENT, (10, 38), (W - 10, 38), 1)

        mode  = self.drone.mode.name
        armed = self.drone.armed
        loc   = self.drone.location.global_relative_frame
        alt   = loc.alt if loc.alt is not None else 0.0
        gspd  = self.drone.groundspeed
        aspd  = self.drone.airspeed
        volts = self.drone.battery.voltage if self.drone.battery.voltage else 0.0
        pct   = self.drone.battery.level   if self.drone.battery.level   else 0
        sats  = (self.drone.gps_0.satellites_visible
                 if self.drone.gps_0.satellites_visible is not None else 0)
        heading = self.drone.heading

        def color_battery(p):
            if p > 50: return CLR_PRIMARY
            if p > LOW_BATTERY_PCT: return CLR_WARN
            return CLR_ALERT

        rows = [
            ("MODE",        mode,
             CLR_PRIMARY  if mode == "GUIDED" else CLR_WARN),
            ("ARMED",       "YES" if armed else "NO",
             CLR_PRIMARY  if armed else CLR_ALERT),
            ("ALT (REL)",   f"{alt:.2f} m",        CLR_LIGHT),
            ("GND SPEED",   f"{gspd:.2f} m/s",     CLR_ACCENT),
            ("AIR SPEED",   f"{aspd:.2f} m/s",     CLR_ACCENT),
            ("BATTERY",     f"{volts:.2f}V  {pct}%", color_battery(pct)),
            ("GPS SATS",    f"{sats}",              CLR_LIGHT),
            ("HEADING",     f"{heading}°",          CLR_LIGHT),
            ("MISSION",     "ACTIVE ▶" if self.mission_active else "IDLE ■",
             CLR_PRIMARY  if self.mission_active else CLR_DIM),
        ]

        y = 46
        for label, value, clr in rows:
            lbl_s = self.font_bold.render(f"{label:<12}", True, CLR_DIM)
            val_s = self.font_md.render(value, True, clr)
            surf.blit(lbl_s, (14, y))
            surf.blit(val_s, (160, y))
            y += 22

        self.screen.blit(surf, (18, 18))

    def _draw_controls_reference(self):
        """Small key-binding cheat sheet — bottom-left."""
        lines = [
            "ARROWS  move   |  W/S  climb/descend",
            "A/D     yaw    |  ENTER  arm & takeoff",
            "L       mission start/stop",
            "SPACE   RTL panic    |  ESC/Q  quit",
        ]
        W  = 490
        H  = len(lines) * 18 + 14
        sx = 18
        sy = SCREEN_HEIGHT - H - 18

        surf = pygame.Surface((W, H), pygame.SRCALPHA)
        surf.fill((10, 14, 22, 180))
        pygame.draw.rect(surf, CLR_DARK, (0, 0, W, H), 1)

        for i, line in enumerate(lines):
            txt = self.font_sm.render(line, True, CLR_DIM)
            surf.blit(txt, (8, 6 + i * 18))

        self.screen.blit(surf, (sx, sy))

    def _draw_status_log(self):
        """Scrolling status message log — bottom-right."""
        msgs = list(self._status_msgs)
        W  = 490
        H  = len(msgs) * 18 + 14
        sx = SCREEN_WIDTH - W - 18
        sy = SCREEN_HEIGHT - H - 18

        surf = pygame.Surface((W, H), pygame.SRCALPHA)
        surf.fill((10, 14, 22, 180))
        pygame.draw.rect(surf, CLR_DARK, (0, 0, W, H), 1)

        for i, msg in enumerate(msgs):
            clr  = CLR_LIGHT if i == 0 else CLR_DIM
            txt  = self.font_sm.render(msg[:68], True, clr)
            surf.blit(txt, (8, 6 + i * 18))

        self.screen.blit(surf, (sx, sy))

    def _draw_mission_banner(self):
        """Flashing mission-active banner across the top centre."""
        if not self.mission_active:
            return
        # Flash at ~1 Hz
        if int(time.time() * 2) % 2 == 0:
            return

        banner = self.font_lg.render(
            "◀  MISSION LOOP ACTIVE  ▶   Press L to stop", True, CLR_ALERT
        )
        bx = SCREEN_WIDTH // 2 - banner.get_width() // 2
        self.screen.blit(banner, (bx, SCREEN_HEIGHT - 55))

    # =========================================================================
    # MAIN LOOP
    # =========================================================================

    def run(self):
        running = True
        while running:
            # ── Events ──────────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if not self._handle_keydown(event.key):
                        running = False

            # ── Flight logic ─────────────────────────────────────────────────
            self._handle_held_keys()    # manual override (blocked when mission active)
            self._tick_mission()        # mission autopilot step
            self._check_battery_failsafe()

            # ── Draw ─────────────────────────────────────────────────────────
            self._draw_camera_or_background()
            self._draw_radar()
            self._draw_telemetry_hud()
            self._draw_controls_reference()
            self._draw_status_log()
            self._draw_mission_banner()

            pygame.display.flip()
            self.clock.tick(TARGET_FPS)

        # ── Cleanup ──────────────────────────────────────────────────────────
        log.info("Shutting down …")
        if self.cam is not None:
            self.cam.release()
        self.drone.close()
        pygame.quit()
        sys.exit(0)


# =============================================================================
# 3. ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    cockpit = CockpitController()
    cockpit.run()