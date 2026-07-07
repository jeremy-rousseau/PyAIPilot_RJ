import time

from pymavlink import mavutil

import logging
logger = logging.getLogger("DronePilot")

# --------------------------------------------------------------------------------------
# RESET COMMAND
MAVLINK_CMD_SIM_RESET = 31000

# --------------------------------------------------------------------------------------
# MOTOR CONTROLS
# --------------------------------------------------------------------------------------

MOTOR_FRONT_LEFT = 0
MOTOR_FRONT_RIGHT = 1
MOTOR_BACK_LEFT = 0
MOTOR_BACK_RIGHT = 0

# PARAMÈTRES REQUIS POUR LE VOL DIRECT (À AJUSTER SELON TON SIMU)
# Note : Si ton simu utilise des valeurs de 0.0 à 1.0, remplace 5000 par 0.5, et 400 par 0.05
HOVER_RPM = 0.5      # Vitesse moteur de base pour contrer la gravité
BASE_FORWARD = 0.2    # Pour que le drone avance légèrement par défaut

GAIN_YAW = 0.4        # Sensibilité pour pivoter à gauche/droite
GAIN_PITCH = 0.3      # Sensibilité pour monter/descendre

def update_motor_control(mavlink_conn,system_boot_ms, shared_data):
    # 1. Comportement par défaut (Pas de porte visible : Vol stationnaire + avance lente)
    m_fl = HOVER_RPM + BASE_FORWARD  # Avant-Gauche
    m_fr = HOVER_RPM + BASE_FORWARD  # Avant-Droit
    m_bl = HOVER_RPM - BASE_FORWARD  # Arrière-Gauche
    m_br = HOVER_RPM - BASE_FORWARD  # Arrière-Droit

    # 2. Si la vision détecte la porte orange
    if shared_data.get('gate_visible'):
        gate_x = shared_data.get('gate_x', 0.0)
        gate_y = shared_data.get('gate_y', 0.0)

        # Calcul des corrections
        # Yaw (Axe X) : Pour tourner à droite, on accélère à gauche et on ralentit à droite
        corr_yaw = gate_x * GAIN_YAW
        
        # Pitch (Axe Y) : Pour monter (gate_y < 0), on accélère uniformément tous les moteurs 
        # Pour piquer vers l'avant, on accélère l'arrière et ralentit l'avant.
        # Ici on utilise une approche mixte (Hauteur + Compensation)
        corr_pitch = -gate_y * GAIN_PITCH

        # Application de la matrice de mixage sur les moteurs
        m_fl = HOVER_RPM + BASE_FORWARD + corr_pitch - corr_yaw
        m_fr = HOVER_RPM + BASE_FORWARD + corr_pitch + corr_yaw
        m_bl = HOVER_RPM - BASE_FORWARD + corr_pitch - corr_yaw
        m_br = HOVER_RPM - BASE_FORWARD + corr_pitch + corr_yaw

    # Stockage des valeurs pour tes logs
    # shared_data['moteurs'] = [int(m_fl), int(m_fr), int(m_bl), int(m_br)]

    # MAVLink attend un tableau de 8 actuateurs
    motor_rpms = [int(m_fl), int(m_fr), int(m_bl), int(m_br), 0, 0, 0, 0]

    # Envoi direct au simulateur
    mavlink_conn.mav.set_actuator_control_target_send(
        int(time.time() * 1e6), # Timestamp en microsecondes
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        0, # Groupe de contrôle (0 = Moteurs principaux)
        motor_rpms
    )

    # Récupération de la puissance réelle des moteurs reçue du simulateur
    pwms = shared_data.get('moteurs', [0, 0, 0, 0])
    status_moteurs = f"PWM Moteurs -> MOTOR_FRONT_LEFT:{pwms[0]} MOTOR_FRONT_RIGHT:{pwms[1]} MOTOR_BACK_LEFT:{pwms[2]} MOTOR_BACK_RIGHT:{pwms[3]}"

    logger.info(f"[MOTEUR] ETAT -> Ordres envoyés : {status_moteurs}")


# def update_motor_control(mavlink_conn, system_boot_ms):
#     motor_rpms = [MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT, 0, 0, 0, 0]
#     mavlink_conn.mav.set_actuator_control_target_send(
#         int(time.time() * 1e6),
#         mavlink_conn.target_system,
#         mavlink_conn.target_component,
#         0,
#         motor_rpms
#     )

# --------------------------------------------------------------------------------------
# ATTITUDE CONTROLS
# --------------------------------------------------------------------------------------
PITCH_RATE = -0.3   # rad/s (negative = pitch forward)
ROLL_RATE  = 0.0
YAW_RATE   = 0.0
THRUST     = 0.5    # 0.0 - 1.0

GAIN_YAW = 0.1      # Sensibilité pour tourner à gauche/droite
GAIN_PITCH = 0.08    # Sensibilité pour monter/descendre

RATES_ATTITUDE_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
)

def update_attitude_flight_control(mavlink_conn, system_boot_ms, shared_data):
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
    # Récupération de la puissance réelle des moteurs reçue du simulateur
    pwms = shared_data.get('moteurs', [0, 0, 0, 0])
    status_moteurs = f"PWM Moteurs -> M1:{pwms[0]} M2:{pwms[1]} M3:{pwms[2]} M4:{pwms[3]}"


    # Si la vision détecte un portail orange
    if shared_data.get('gate_visible'):

        #Récupération des données
        gate_x = shared_data.get('gate_x')
        gate_y = shared_data.get('gate_y')
        gate_size = shared_data.get('gate_size')

        # 1. Orientation (Lacet / Yaw)
        # EXEMPLE : si le portail est à droite (gate_x > 0), on tourne à droite
        yaw_rate_to_gate = gate_x * GAIN_YAW

        # 2. Hauteur (Tangage / Pitch) : Si le portail est plus haut que le centre (gate_y < 0)
        # Note : En MAVLink, un pitch_rate négatif fait pencher le drone vers l'avant (pour avancer).
        # Ici, on utilise le pitch pour monter/descendre ou ajuster notre approche.
        # On garde PITCH_RATE (-0.3) pour avancer, et on ajoute/soustrait pour monter ou descendre.
        # Si gate_y < 0 (porte haute), "- gate_y" devient positif -> le drone se redresse un peu pour monter.
        pitch_rate_to_gate = PITCH_RATE - (gate_y * GAIN_PITCH)


        # 3. Vitesse / Poussée (Thrust) : 
        # Si le portail est loin (gate_size est petit), on maintient la poussée.
        # Plus on approche (gate_size grandit), plus on pourrait stabiliser.
        thrust_to_gate = THRUST

        mavlink_conn.mav.set_attitude_target_send(
            now_ms - system_boot_ms,
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            RATES_ATTITUDE_MASK,
            [1, 0, 0, 0],  # dummy quaternion (ignored)
            ROLL_RATE,
            pitch_rate_to_gate,
            yaw_rate_to_gate,
            thrust_to_gate
        )
        logger.info(f"[PILOTE] CIBLE VISIBLE -> Ordres envoyés : Pitch={pitch_rate_to_gate:.2f} | Yaw={yaw_rate_to_gate:.2f} | Thrust={thrust_to_gate:.2f} | {status_moteurs}")

    else:
        # Mode recherche / avancement par défaut si aucun portail visible
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
        logger.info(f"[PILOTE] MODE RECHERCHE -> Ordres par défaut : Pitch={PITCH_RATE:.2f} | Yaw={YAW_RATE:.2f} | Thrust={THRUST:.2f} | {status_moteurs}")

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
# Control Loop
# --------------------------------------------------------------------------------------

CONTROL_HZ = 250

class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms

    def update(self):
        # send automated targets to sim flight controller
        # update_attitude_flight_control(self.sim_conn, self.system_boot_ms,self.data)
        # alternatively one of
        # update_position_flight_control(self.sim_conn, self.system_boot_ms)
        update_motor_control(self.sim_conn, self.system_boot_ms,self.data)

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
