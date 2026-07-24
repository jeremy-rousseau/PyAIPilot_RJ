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
THRUST     = 0.1    # 0.0 - 1.0

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

# Ici on stabilise le drone, on le remet "droit" après avoir tourner à gauche ou à droite
def decide_drone_movement(data):
    """
    Calcule les vitesses angulaires (rad/s) et la poussée (0..1)
    en fonction de la position de la porte et de la stabilisation IMU.
    """
    gate_x = data.get('gate_x', None)
    imu_gyro = data.get('imu_gyro', (0.0, 0.0, 0.0))  # (roll_rate, pitch_rate, yaw_rate)
    
    current_roll_rate = imu_gyro[0]
    current_yaw_rate  = imu_gyro[2]

    # --- PARAMÈTRES ADOUCIS ---
    ZONE_MORTE = 0.008  

    # Gains réduits pour éviter la brutalité
    K_P_YAW  = 0.80   # (Était à 1.2 -> divisé par 2.4)
    K_P_ROLL = 0.5   # (Était à 0.8 -> divisé par 2.3)

    K_D_YAW  = 0.15   
    K_D_ROLL = 0.20  

    # Limites maximales de commande (Saturations)
    MAX_YAW_RATE  = 0.08  # Limite la vitesse de rotation max
    MAX_ROLL_RATE = 0.05  # Limite l'inclinaison max

    target_yaw_rate = 0.0
    target_roll_rate = 0.0

    # 3. Décision basée sur la vision
    if gate_x is not None:
        if abs(gate_x) > ZONE_MORTE:
            target_yaw_rate  = K_P_YAW * gate_x
            target_roll_rate = K_P_ROLL * gate_x

            # Saturation des consignes (pour ne pas être "brute")
            target_yaw_rate  = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, target_yaw_rate))
            target_roll_rate = max(-MAX_ROLL_RATE, min(MAX_ROLL_RATE, target_roll_rate))

            state_str = "DROITE" if gate_x > 0 else "GAUCHE"
        else:
            target_yaw_rate  = 0.0
            target_roll_rate = 0.0
            state_str = "CENTRE"
    else:
        target_yaw_rate  = 0.0
        target_roll_rate = 0.0
        state_str = "RECHERCHE"

    # 4. Calcul final (Consigne - Freinage IMU)
    final_yaw_rate  = target_yaw_rate - (K_D_YAW * current_yaw_rate)
    final_roll_rate = target_roll_rate - (K_D_ROLL * current_roll_rate)

    # Avancement adouci pour laisser du temps à l'alignement
    final_pitch_rate = -0.18  # (Était à -0.3)
    final_thrust     = 0.55 

    # 5. LOG DÉTAILLÉ DE DIAGNOSTIC
    if gate_x is not None:
        gx_str = f"{gate_x:.3f}"
        logger.info(
            f"[DIAG 🪛] Action: {state_str:<9} | "
            f"GateX: {gx_str:<6} | "
            f"Gyro(R,Y): ({current_roll_rate:+.2f}, {current_yaw_rate:+.2f}) | "
            f"Cmd(R,Y): ({final_roll_rate:+.2f}, {final_yaw_rate:+.2f})"
        )

    return final_roll_rate, final_pitch_rate, final_yaw_rate, final_thrust

# --------------------------------------------------------------------------------------
# POSITION CONTROLS
# --------------------------------------------------------------------------------------
# VELOCITY_POSITION_MASK = (
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |

#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |

#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
# )

# def update_position_flight_control(mavlink_conn, system_boot_ms):
    # now_ms = int(time.time() * 1000)

    # """
    # Sets a desired vehicle position in a local north-east-down coordinate
    # frame. Used by an external controller to command the vehicle
    # (manual controller or other system).

    # time_boot_ms              : Timestamp (time since system boot). [ms] (type:uint32_t)
    # target_system             : System ID (type:uint8_t)
    # target_component          : Component ID (type:uint8_t)
    # coordinate_frame          : Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9 (type:uint8_t, values:MAV_FRAME)
    # type_mask                 : Bitmap to indicate which dimensions should be ignored by the vehicle. (type:uint16_t, values:POSITION_TARGET_TYPEMASK)
    # x                         : X Position in NED frame [m] (type:float)
    # y                         : Y Position in NED frame [m] (type:float)
    # z                         : Z Position in NED frame (note, altitude is negative in NED) [m] (type:float)
    # vx                        : X velocity in NED frame [m/s] (type:float)
    # vy                        : Y velocity in NED frame [m/s] (type:float)
    # vz                        : Z velocity in NED frame [m/s] (type:float)
    # afx                       : X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N [m/s/s] (type:float)
    # afy                       : Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N [m/s/s] (type:float)
    # afz                       : Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N [m/s/s] (type:float)
    # yaw                       : yaw setpoint [rad] (type:float)
    # yaw_rate                  : yaw rate setpoint [rad/s] (type:float)
    # """
    # mavlink_conn.mav.set_position_target_local_ned_send(
    #     now_ms - system_boot_ms,
    #     mavlink_conn.target_system,
    #     mavlink_conn.target_component,
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    #     VELOCITY_POSITION_MASK,
    #     0.0, 0, 0.0,    # ignored position NED
    #     2.0, 0.0, 0.0,  # Vel - 2 m/s forward
    #     0.0, 0, 0.0,    # ignored acceleration
    #     0,              # ignored yaw
    #     0.0             # ignored yaw rate
    # )

# --------------------------------------------------------------------------------------
# MOTOR MODE CONTROLS
# --------------------------------------------------------------------------------------
def determine_motor_mode(shared_data):
    global MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT

    #On récupère les info
    gate_visible = shared_data.get('gate_visible', False)
    gate_x = shared_data.get('gate_x', 0.0)
    gate_y = shared_data.get('gate_y', 0.0)
    gate_size = shared_data.get('gate_size', 0.0)  # Surface relative du trou (0.0 à 1.0)

    # --- PARAMÈTRES ---
    ZONE_MORTE = 0.08       # Marge de tolérance au centre
    GAZ_STATIONNAIRE = 0.265 # Puissance pour maintenir l'altitude (à ajuster selon ton drone)
    KP_MONTER = 0.27        # Correction forte pour remonter rapidement
    KP_DESCENDRE = 0.12     # Correction TRÈS DOUCE pour éviter de tomber
    SAFETY_CEILING = 0.55   #Plafond de sécurité (ex: max X%)
    SAFETY_FLOOR = 0.12     #Plancher de sécurité (ex: min X%)

    # 1. Mode Standby / Recherche : si aucune porte vue
    if not gate_visible:
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = GAZ_STATIONNAIRE
        return "MODE: STANDBY / RECHERCHE 🔍"

    # --- LOGIQUE DE CONTRÔLE VERTICAL ---
    if gate_y < -ZONE_MORTE:
        # La porte est en HAUT de l'image -> Le drone est TROP BAS -> Il doit MONTER
        correction = KP_MONTER * abs(gate_y)
        puissance = GAZ_STATIONNAIRE + correction

        # Plafond de sécurité
        puissance = min(SAFETY_CEILING, puissance)
        
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = puissance
        MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = puissance
        return f"MODE: MONTER ⬆️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f}"

    elif gate_y > ZONE_MORTE:
        # La porte est en BAS de l'image -> Le drone est TROP HAUT -> Il doit DESCENDRE
        correction = KP_DESCENDRE * abs(gate_y)
        puissance = GAZ_STATIONNAIRE - correction

        # Plancher de sécurité
        puissance = max(SAFETY_FLOOR, puissance)
        
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = puissance
        MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = puissance
        return f"MODE: DESCENDRE ⬇️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f}"

    else:
        # La porte est centrée verticalement -> Maintien d'altitude / Avancer
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = GAZ_STATIONNAIRE
        MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = GAZ_STATIONNAIRE
        return f"FRANCHISSEMENT :: gaz stationnaire : {GAZ_STATIONNAIRE:.2f}  gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f}"




    # # --- FRANCHISSEMENT (BLIND DASH) ---
    # # Si le trou occupe plus de 18% de l'image, la porte est tout près :
    # # On fonce tout droit sans corriger X/Y car le cadre va sortir du champ de vision.
    # if gate_size and gate_size > 0.18:
    #     MOTOR_FRONT_LEFT = 0.28
    #     MOTOR_FRONT_RIGHT = 0.28
    #     MOTOR_BACK_LEFT = 0.28
    #     MOTOR_BACK_RIGHT = 0.28
    #     return f"FRANCHISSEMENT :: gate_x : {gate_x} / gate_y : {gate_y} / gate_size : {gate_size} "

    # ZONE_MORTE = 0.06  # Légèrement réduite pour une meilleure précision

    # # -------------------------------------------------------------
    # # PRIORITÉ 1 : MONTER / DESCENDRE (Axe Y)
    # # # Axe Y : +1.0 (Bas de l'image) , -1.0 (Haut de l'image)
    # # -------------------------------------------------------------
    # if gate_y < ZONE_MORTE:
    #     MOTOR_FRONT_LEFT = 0.35
    #     MOTOR_FRONT_RIGHT = 0.35
    #     MOTOR_BACK_LEFT = 0.35
    #     MOTOR_BACK_RIGHT = 0.35
    #     return "MODE: MONTER ⬆️"

    # elif gate_y > ZONE_MORTE:
    #     MOTOR_FRONT_LEFT = 0.22
    #     MOTOR_FRONT_RIGHT = 0.22
    #     MOTOR_BACK_LEFT = 0.22
    #     MOTOR_BACK_RIGHT = 0.22
    #     return "MODE: DESCENDRE ⬇️"

    # -------------------------------------------------------------
    # PRIORITÉ 2 : GAUCHE / DROITE (Axe X)
    # gate_x > 0 signifie que la porte est à DROITE -> PIVOT DROITE
    # -------------------------------------------------------------
    # if gate_x > ZONE_MORTE:
    #     # Différentiel pour pivoter à droite (moteurs gauches plus forts)
    #     MOTOR_FRONT_LEFT = 0.32
    #     MOTOR_FRONT_RIGHT = 0.24
    #     MOTOR_BACK_LEFT = 0.32
    #     MOTOR_BACK_RIGHT = 0.24
    #     return "MODE: DROITE ➡️"

    # elif gate_x < -ZONE_MORTE:
    #     # Différentiel pour pivoter à gauche (moteurs droits plus forts)
    #     MOTOR_FRONT_LEFT = 0.24
    #     MOTOR_FRONT_RIGHT = 0.32
    #     MOTOR_BACK_LEFT = 0.24
    #     MOTOR_BACK_RIGHT = 0.32
    #     return "MODE: GAUCHE ⬅️"

    # # -------------------------------------------------------------
    # # ALIGNÉ : AVANCE VERS LA PORTE
    # # Vitesse d'avance ajustée dynamiquement selon la distance
    # # -------------------------------------------------------------
    # if gate_size and gate_size < 0.05:
    #     # Porte loin : on avance un peu plus fort
    #     base_speed = 0.35
    # else:
    #     # Porte proche : approche prudente
    #     base_speed = 0.28

    # MOTOR_FRONT_LEFT = base_speed
    # MOTOR_FRONT_RIGHT = base_speed
    # MOTOR_BACK_LEFT = base_speed
    # MOTOR_BACK_RIGHT = base_speed

    # return "MODE: AVANCE CENTRÉE 🚪"
# def determine_motor_mode(shared_data):
#     global MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT

#     # 1. Mode par défaut : Si pas de porte, on avance lentement
#     if not shared_data.get('gate_visible', False):
#         MOTOR_FRONT_LEFT = 0.28
#         MOTOR_FRONT_RIGHT = 0.28
#         MOTOR_BACK_LEFT = 0.28
#         MOTOR_BACK_RIGHT = 0.28
#     #TODO : trouver les bons para pour garder le drone droit tout en allant vers l'avant
#         return "MODE: STAND BY"#TODO : desactiver log quand la course pas encore lancé

#     gate_x = shared_data.get('gate_x', 0.0)
#     gate_y = shared_data.get('gate_y', 0.0)

#     ZONE_MORTE = 0.1

#     # -------------------------------------------------------------
#     # PRIORITÉ 1 : MONTER / DESCENDRE (Axe Y)
#     # -------------------------------------------------------------

#     if gate_y > ZONE_MORTE:
#         MOTOR_FRONT_LEFT = 0.65
#         MOTOR_FRONT_RIGHT = 0.65
#         MOTOR_BACK_LEFT = 0.65
#         MOTOR_BACK_RIGHT = 0.65
#         return "MODE: MONTER ⬆️"

    
#     elif gate_y < -ZONE_MORTE:
#         MOTOR_FRONT_LEFT = 0.25
#         MOTOR_FRONT_RIGHT = 0.25
#         MOTOR_BACK_LEFT = 0.25
#         MOTOR_BACK_RIGHT = 0.25
    
#         return "MODE: DESCENDRE ⬇️"
    
    
        
#     # # -------------------------------------------------------------
#     # # PRIORITÉ 2 : GAUCHE / DROITE (Axe X)
#     # # Si gate_x > ZONE_MORTE, la porte est à DROITE -> Tourner à droite
#     # # -------------------------------------------------------------

#     if gate_x > ZONE_MORTE:
#         # DROITE (Yaw à droite) : Moteurs Gauches poussent plus, Moteurs Droits poussent moins
#         MOTOR_FRONT_LEFT = 0.30
#         MOTOR_FRONT_RIGHT = 0.25
#         MOTOR_BACK_LEFT = 0.30
#         MOTOR_BACK_RIGHT = 0.25

#         return "MODE: DROITE ➡️"
        
#     elif gate_x < -ZONE_MORTE:
#         # GAUCHE (Yaw à gauche) : Moteurs Droits poussent plus, Moteurs Gauches poussent moins
#         MOTOR_FRONT_LEFT = 0.25
#         MOTOR_FRONT_RIGHT = 0.30
#         MOTOR_BACK_LEFT = 0.25
#         MOTOR_BACK_RIGHT = 0.30

#         return "MODE: GAUCHE ⬅️"

#     # MOTOR_FRONT_LEFT = 0.2799999
#     # MOTOR_FRONT_RIGHT = 0.2799999
#     # MOTOR_BACK_LEFT = 0.28
#     # MOTOR_BACK_RIGHT = 0.28
#     MOTOR_FRONT_LEFT = 0.28
#     MOTOR_FRONT_RIGHT = 0.28
#     MOTOR_BACK_LEFT = 0.28
#     MOTOR_BACK_RIGHT = 0.28

#     return "MODE: AVANCE LENTE (vers porte) 🚪"


        # MOTOR_FRONT_LEFT = 0
        # MOTOR_FRONT_RIGHT = 0
        # MOTOR_BACK_LEFT = 0
        # MOTOR_BACK_RIGHT = 0
        # return "MODE: STATIONNEMENT"


# def decide_drone_movement(shared_data):
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


class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.system_boot_ms = system_boot_ms

        self.frame_counter = 0

        


    def update(self):
        self.frame_counter += 1

        # 1. Récupération de l'état de la course depuis shared_data
        race_started = self.data.get('race_started', False)

        if not race_started:
            # Compte à rebours/attente
            # logger.info("[LOG] Attente du départ... Moteurs coupés (0%)")
            pass
        else:
            # ROLL_RATE, PITCH_RATE, YAW_RATE, THRUST = decide_drone_movement(self.data)
            # à remettre


            # send automated targets to sim flight controller
            # update_attitude_flight_control(self.sim_conn, self.system_boot_ms)
            # alternatively one of
            # update_position_flight_control(self.sim_conn, self.system_boot_ms)
            update_motor_control(self.sim_conn, self.system_boot_ms)
            
            info_control = determine_motor_mode(self.data)
            # mode_actuel = decide_drone_movement(self.data)
            if info_control != "MODE: STANDBY / RECHERCHE 🔍" and self.frame_counter % 50 == 0:
                logger.info(f"{info_control}")
                self.frame_counter = 0
            # logger.info(f"[LOG] Moteurs -> AV_G: {MOTOR_FRONT_LEFT:.4f} | AV_D: {MOTOR_FRONT_RIGHT:.4f} | AR_G: {MOTOR_BACK_LEFT:.4f} | AR_D: {MOTOR_BACK_RIGHT:.4f}")

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
