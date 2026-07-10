import time

from pymavlink import mavutil

#Pour les logs
import logging
logger = logging.getLogger("DronePilot")

# --------------------------------------------------------------------------------------
# RESET COMMAND
MAVLINK_CMD_SIM_RESET = 31000

# --------------------------------------------------------------------------------------
# MOTOR CONTROLS
# --------------------------------------------------------------------------------------

MOTOR_FRONT_LEFT = 0
MOTOR_FRONT_RIGHT = 0
MOTOR_BACK_LEFT = 0
MOTOR_BACK_RIGHT = 0
# INFO : Normed to -1..+1 where 0 is neutral position. 
# Throttle for single rotation direction motors is 0..1, negative range for reverse direction.

def update_motor_control(mavlink_conn, system_boot_ms):
    motor_rpms = [MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT, 0, 0, 0, 0]
    mavlink_conn.mav.set_actuator_control_target_send(
        int(time.time() * 1e6),
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        0,
        motor_rpms
    )

# --------------------------------------------------------------------------------------
# ATTITUDE CONTROLS
# --------------------------------------------------------------------------------------
#default
PITCH_RATE = -0.3   # rad/s (negative = pitch forward)
ROLL_RATE  = 0.0
YAW_RATE   = 0.0
THRUST     = 0.6    # 0.0 - 1.0

#decide_drone_movement
# PITCH_RATE = 0.0   # rad/s (negative = pitch forward)
# ROLL_RATE  = 0.0
# YAW_RATE   = 0.0
# THRUST     = 0.0    # 0.0 - 1.0

# ZONE_MORTE = 0.10
# THRUST_HOVER = 0.52 # Ta valeur de référence pour le vol stationnaire
########

RATES_ATTITUDE_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
)

def update_attitude_flight_control(mavlink_conn, system_boot_ms):
    now_ms = int(time.time() * 1000)

    """
    Sets a desired vehicle attitude. Used by an external controller to
    command the vehicle (manual controller or other system).
    
    time_boot_ms              : Timestamp (time since system boot). [ms] (type:uint32_t)
    target_system             : System ID (type:uint8_t)
    target_component          : Component ID (type:uint8_t)
    type_mask                 : Bitmap to indicate which dimensions should be ignored by the vehicle. (type:uint8_t, values:ATTITUDE_TARGET_TYPEMASK)
    q                         : Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) (type:float)
    body_roll_rate            : Body roll rate [rad/s] (type:float)
    body_pitch_rate           : Body pitch rate [rad/s] (type:float)
    body_yaw_rate             : Body yaw rate [rad/s] (type:float)
    thrust                    : Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust) (type:float)
    """
    mavlink_conn.mav.set_attitude_target_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        RATES_ATTITUDE_MASK,
        [1, 0, 0, 0],  # dummy quaternion (ignored)
        ROLL_RATE,
        PITCH_RATE,
        YAW_RATE,
        THRUST
    )

# --------------------------------------------------------------------------------------
# POSITION CONTROLS
# --------------------------------------------------------------------------------------
VELOCITY_POSITION_MASK = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |

        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |

        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

def update_position_flight_control(mavlink_conn, system_boot_ms):
    now_ms = int(time.time() * 1000)

    """
    Sets a desired vehicle position in a local north-east-down coordinate
    frame. Used by an external controller to command the vehicle
    (manual controller or other system).

    time_boot_ms              : Timestamp (time since system boot). [ms] (type:uint32_t)
    target_system             : System ID (type:uint8_t)
    target_component          : Component ID (type:uint8_t)
    coordinate_frame          : Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 (type:uint8_t, values:MAV_FRAME)
    type_mask                 : Bitmap to indicate which dimensions should be ignored by the vehicle. (type:uint16_t, values:POSITION_TARGET_TYPEMASK)
    x                         : X Position in NED frame [m] (type:float)
    y                         : Y Position in NED frame [m] (type:float)
    z                         : Z Position in NED frame (note, altitude is negative in NED) [m] (type:float)
    vx                        : X velocity in NED frame [m/s] (type:float)
    vy                        : Y velocity in NED frame [m/s] (type:float)
    vz                        : Z velocity in NED frame [m/s] (type:float)
    afx                       : X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N [m/s/s] (type:float)
    afy                       : Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N [m/s/s] (type:float)
    afz                       : Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N [m/s/s] (type:float)
    yaw                       : yaw setpoint [rad] (type:float)
    yaw_rate                  : yaw rate setpoint [rad/s] (type:float)
    """
    mavlink_conn.mav.set_position_target_local_ned_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_POSITION_MASK,
        0.0, 0, 0.0,    # ignored position NED
        2.0, 0.0, 0.0,  # Vel - 2 m/s forward
        0.0, 0, 0.0,    # ignored acceleration
        0,              # ignored yaw
        0.0             # ignored yaw rate
    )

# --------------------------------------------------------------------------------------
# MOTOR MODE CONTROLS
# --------------------------------------------------------------------------------------
def determine_motor_mode(shared_data):
    global MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT

    # 1. Mode par défaut : Si pas de porte, on avance lentement
    if not shared_data.get('gate_visible', False):
        MOTOR_FRONT_LEFT = 0.2799
        MOTOR_FRONT_RIGHT = 0.2799
        MOTOR_BACK_LEFT = 0.28
        MOTOR_BACK_RIGHT = 0.28
    #TODO : trouver les bons para pour garder le drone droit tout en allant vers l'avant
        return "MODE: AVANCE LENTE"#TODO : desactiver log quand la course pas encore lancé

    gate_x = shared_data.get('gate_x', 0.0)
    gate_y = shared_data.get('gate_y', 0.0)

    ZONE_MORTE = 0.10

    # -------------------------------------------------------------
    # PRIORITÉ 1 : MONTER / DESCENDRE (Axe Y)
    # En OpenCV, Y augmente vers le bas de l'écran. 
    # Donc si gate_y > ZONE_MORTE, la porte est PLUS BASSE que le drone -> Descendre
    # -------------------------------------------------------------

    # if gate_y > ZONE_MORTE:
    #     MOTOR_FRONT_LEFT = 0.25
    #     MOTOR_FRONT_RIGHT = 0.25
    #     MOTOR_BACK_LEFT = 0.25
    #     MOTOR_BACK_RIGHT = 0.25
    
    #     return "MODE: DESCENDRE"
    
    # elif gate_y < -ZONE_MORTE:
    #     MOTOR_FRONT_LEFT = 0.35
    #     MOTOR_FRONT_RIGHT = 0.35
    #     MOTOR_BACK_LEFT = 0.35
    #     MOTOR_BACK_RIGHT = 0.35
    #     return "MODE: MONTER"
        
    # -------------------------------------------------------------
    # PRIORITÉ 2 : GAUCHE / DROITE (Axe X)
    # Si gate_x > ZONE_MORTE, la porte est à DROITE -> Tourner à droite
    # -------------------------------------------------------------

    # if gate_x > ZONE_MORTE:
    #     # DROITE (Yaw à droite) : Moteurs Gauches poussent plus, Moteurs Droits poussent moins
    #     MOTOR_FRONT_LEFT = 0.30
    #     MOTOR_FRONT_RIGHT = 0.299
    #     MOTOR_BACK_LEFT = 0.30
    #     MOTOR_BACK_RIGHT = 0.299

    #     return "MODE: DROITE"
        
    # elif gate_x < -ZONE_MORTE:
    #     # GAUCHE (Yaw à gauche) : Moteurs Droits poussent plus, Moteurs Gauches poussent moins
    #     MOTOR_FRONT_LEFT = 0.299
    #     MOTOR_FRONT_RIGHT = 0.30
    #     MOTOR_BACK_LEFT = 0.299
    #     MOTOR_BACK_RIGHT = 0.30

    #     return "MODE: GAUCHE"

    MOTOR_FRONT_LEFT = 0.279
    MOTOR_FRONT_RIGHT = 0.279
    MOTOR_BACK_LEFT = 0.28
    MOTOR_BACK_RIGHT = 0.28

    return "MODE: AVANCE LENTE (vers porte)"


        # MOTOR_FRONT_LEFT = 0
        # MOTOR_FRONT_RIGHT = 0
        # MOTOR_BACK_LEFT = 0
        # MOTOR_BACK_RIGHT = 0
        # return "MODE: STATIONNEMENT"


def decide_drone_movement(shared_data):
    # On déclare qu'on va modifier les constantes globales en haut du script
    global PITCH_RATE, ROLL_RATE, YAW_RATE, THRUST

    gate_x = shared_data.get('gate_x', 0.0)
    gate_y = shared_data.get('gate_y', 0.0)
    gate_visible = shared_data.get('gate_visible', False)
    
    # Réinitialisation par défaut (Vol stationnaire)
    PITCH_RATE = 0.0
    ROLL_RATE  = 0.0
    YAW_RATE   = 0.0
    THRUST     = THRUST_HOVER
    
    mode = "MODE: RECHERCHE"

    if gate_visible:
        # -------------------------------------------------------------
        # PRIORITÉ 1 : MONTER / DESCENDRE (Axe Y en OpenCV)
        # -------------------------------------------------------------
        if gate_y > ZONE_MORTE:
            THRUST = THRUST_HOVER - 0.12  # On baisse les gaz pour descendre
            mode = "MODE: DESCENDRE"
            
        elif gate_y < -ZONE_MORTE:
            THRUST = THRUST_HOVER + 0.15  # On pousse les gaz pour monter
            mode = "MODE: MONTER"
            
        # -------------------------------------------------------------
        # PRIORITÉ 2 : GAUCHE / DROITE (Axe X)
        # -------------------------------------------------------------
        elif gate_x > ZONE_MORTE:
            YAW_RATE = 0.4   # Rotation vers la droite (rad/s)
            mode = "MODE: DROITE"
            
        elif gate_x < -ZONE_MORTE:
            YAW_RATE = -0.4  # Rotation vers la gauche (rad/s)
            mode = "MODE: GAUCHE"
            
        # -------------------------------------------------------------
        # PRIORITÉ 3 : ALIGNÉ -> AVANCER
        # -------------------------------------------------------------
        else:
            PITCH_RATE = -0.3  # Négatif = pitch forward pour avancer
            THRUST = THRUST_HOVER + 0.05 # Léger filet de gaz pour compenser la translation
            mode = "MODE: AVANCE VERS LA PORTE"
            
    else:
        # Mode recherche si la porte est perdue de vue
        YAW_RATE = 0.3
        THRUST = THRUST_HOVER
        mode = "MODE: RECHERCHE PORTE (Rotation)"

    # --- SÉCURISATION DES PLAGES ---
    THRUST = max(0.0, min(1.0, THRUST))
    PITCH_RATE = max(-1.0, min(1.0, PITCH_RATE))
    YAW_RATE = max(-1.0, min(1.0, YAW_RATE))

    return mode
# --------------------------------------------------------------------------------------
# Control Loop
# --------------------------------------------------------------------------------------

CONTROL_HZ = 250
DEPART = True

class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms

    def update(self):
        global DEPART
        
        # send automated targets to sim flight controller
        # update_attitude_flight_control(self.sim_conn, self.system_boot_ms)
        # alternatively one of
        # update_position_flight_control(self.sim_conn, self.system_boot_ms)
        if DEPART :
            time.sleep(4)
            DEPART = False
        else :
            update_motor_control(self.sim_conn, self.system_boot_ms)
        
        mode_actuel = determine_motor_mode(self.data)
        # mode_actuel = decide_drone_movement(self.data)
        logger.info(f"[LOG] {mode_actuel}")

        time.sleep(1.0 / CONTROL_HZ)

    # -------------------------------
    # Arm the drone
    # -------------------------------
    def arm(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

    # -------------------------------
    # Reset sim
    # -------------------------------
    def send_sim_reset_command(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            MAVLINK_CMD_SIM_RESET,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0
        )
