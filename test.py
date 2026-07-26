def determine_motor_mode(shared_data):
    global MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT
    global ROLL_RATE

    #On récupère les info
    gate_visible = shared_data.get('gate_visible', False)
    gate_x = shared_data.get('gate_x', 0.0)
    gate_y = shared_data.get('gate_y', 0.0)
    gate_size = shared_data.get('gate_size', 0.0)  # Surface relative du trou (0.0 à 1.0)
    imu_gyro = shared_data.get('imu_gyro', (0.0, 0.0, 0.0))  # (roll_rate, pitch_rate, yaw_rate)
    current_roll_rate = imu_gyro[0]

    # --- PARAMÈTRES ---
    ZONE_MORTE = 0.1       # Marge de tolérance au centre
    GAZ_STATIONNAIRE = 0.28 # Puissance pour maintenir l'altitude (à ajuster selon ton drone)
    KP_MONTER = 0.30        # Correction pour remonter rapidement
    KP_DESCENDRE = 0.12     # Correction pour éviter de tomber
    SAFETY_CEILING = 0.45   # Plafond de sécurité (ex: max X%)
    SAFETY_FLOOR = 0.20     # Plancher de sécurité (ex: min X%)
    KP_DIRECTION = 0.03     # Correction pour direction

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

        if gate_x > ZONE_MORTE:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance + KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance - KP_DIRECTION
            return f"MODE: MONTER/DROITE ⬆️➡️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"
        elif gate_x < -ZONE_MORTE:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance - KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance + KP_DIRECTION
            return f"MODE: MONTER/GAUCHE ⬆️⬅️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"

        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = puissance
        MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = puissance
        return f"MODE: MONTER ⬆️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"

    elif gate_y > ZONE_MORTE:
        # La porte est en BAS de l'image -> Le drone est TROP HAUT -> Il doit DESCENDRE
        correction = KP_DESCENDRE * abs(gate_y)
        puissance = GAZ_STATIONNAIRE - correction

        # Plancher de sécurité
        puissance = max(SAFETY_FLOOR, puissance)

        if gate_x > ZONE_MORTE:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance + KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance - KP_DIRECTION
            return f"MODE: DESCENDRE/DROITE ⬇️➡️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"
        elif gate_x < -ZONE_MORTE:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance - KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance + KP_DIRECTION
            return f"MODE: DESCENDRE/GAUCHE ⬇️⬅️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"

        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = puissance
        MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = puissance
        return f"MODE: DESCENDRE ⬇️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"

    else:
        # La porte est centrée verticalement -> Maintien d'altitude / Avancer
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = GAZ_STATIONNAIRE
        MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = GAZ_STATIONNAIRE
        return f"FRANCHISSEMENT :: gaz stationnaire : {GAZ_STATIONNAIRE:.2f}  gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / gate_size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f}"

# Variables globales pour le filtre passe-bas (lissage)
FILTERED_GATE_X = 0.0
FILTERED_GATE_Y = 0.0

def determine_motor_mode(shared_data):
    global MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT
    global ROLL_RATE, PITCH_RATE, YAW_RATE, THRUST
    global FILTERED_GATE_X, FILTERED_GATE_Y

    # --- RECUPERATION ET LISSAGE (FILTRE PASSE-BAS) ---
    gate_visible = shared_data.get('gate_visible', False)
    raw_x = shared_data.get('gate_x', 0.0)
    raw_y = shared_data.get('gate_y', 0.0)
    gate_size = shared_data.get('gate_size', 0.0)
    imu_gyro = shared_data.get('imu_gyro', (0.0, 0.0, 0.0))  # (roll_rate, pitch_rate, yaw_rate)
    current_roll_rate = imu_gyro[0]
    current_yaw_rate  = imu_gyro[2]



    # 1. Mode Standby / Recherche : si aucune porte vue
    if not gate_visible:
        FILTERED_GATE_X, FILTERED_GATE_Y = 0.0, 0.0
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = 0.28
        # Maintien / Stabilisation instantanée
        ROLL_RATE, PITCH_RATE, YAW_RATE, THRUST = stabilize_drone(shared_data)
        return "MODE: STANDBY / RECHERCHE 🔍"

    # Lissage sur la vision pour supprimer le bruit à 250 Hz (alpha = 0.15)
    ALPHA = 0.15
    FILTERED_GATE_X = (ALPHA * raw_x) + ((1 - ALPHA) * FILTERED_GATE_X)
    FILTERED_GATE_Y = (ALPHA * raw_y) + ((1 - ALPHA) * FILTERED_GATE_Y)

    gate_x = FILTERED_GATE_X
    gate_y = FILTERED_GATE_Y

    # --- PARAMÈTRES RÉÉQUILIBRÉS ---
    ZONE_MORTE_Y = 0.10     # Tolérance verticale
    ZONE_MORTE_X = 0.15     # Tolérance horizontale
    GAZ_STATIONNAIRE = 0.27 

    # Gains P adoucis pour éliminer le yoyo
    KP_MONTER = 0.15        # Diminué (était 0.30) pour éviter les bonds
    KP_DESCENDRE = 0.12     
    KP_DIRECTION = 0.01

    SAFETY_CEILING = 0.38   # Plafond rabaissé à 38% (était 0.45)
    SAFETY_FLOOR = 0.24     # Plancher de sécurité (20%)

    # -------------------------------------------------------------
    # LOGIQUE DE CONTRÔLE
    # -------------------------------------------------------------

    # A. Drone TROP BAS -> MONTER
    if gate_y < -ZONE_MORTE_Y:
        puissance = min(SAFETY_CEILING, GAZ_STATIONNAIRE + (KP_MONTER * abs(gate_y)))

        if gate_x > ZONE_MORTE_X:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance + KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance - KP_DIRECTION
            return f"MODE: MONTER/DROITE ⬆️➡️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"
        elif gate_x < -ZONE_MORTE_X:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance - KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance + KP_DIRECTION
            return f"MODE: MONTER/GAUCHE ⬆️⬅️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"

        # Maintien / Stabilisation instantanée
        ROLL_RATE, PITCH_RATE, YAW_RATE, THRUST = stabilize_drone(shared_data)
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = puissance
        return f"MODE: MONTER ⬆️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"

    # B. Drone TROP HAUT -> DESCENDRE
    elif gate_y > ZONE_MORTE_Y:
        puissance = max(SAFETY_FLOOR, GAZ_STATIONNAIRE - (KP_DESCENDRE * abs(gate_y)))

        if gate_x > ZONE_MORTE_X:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance + KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance - KP_DIRECTION
            return f"MODE: DESCENDRE/DROITE ⬇️➡️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"
        elif gate_x < -ZONE_MORTE_X:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = puissance - KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = puissance + KP_DIRECTION
            return f"MODE: DESCENDRE/GAUCHE ⬇️⬅️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"

        # Maintien / Stabilisation instantanée
        ROLL_RATE, PITCH_RATE, YAW_RATE, THRUST = stabilize_drone(shared_data)
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = puissance
        return f"MODE: DESCENDRE ⬇️ :: puissance : {puissance:.2f} :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"

    # C. Drone aligné en Y -> Ajustement X OU Franchissement
    else:
        # Si aligné verticalement mais PAS horizontalement
        if gate_x > ZONE_MORTE_X:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = GAZ_STATIONNAIRE + KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = GAZ_STATIONNAIRE - KP_DIRECTION
            return f"MODE: AJUSTEMENT DROITE ➡️ :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"
        elif gate_x < -ZONE_MORTE_X:
            MOTOR_FRONT_LEFT = MOTOR_BACK_LEFT = GAZ_STATIONNAIRE - KP_DIRECTION
            MOTOR_FRONT_RIGHT = MOTOR_BACK_RIGHT = GAZ_STATIONNAIRE + KP_DIRECTION
            return f"MODE: AJUSTEMENT GAUCHE ⬅️ :: gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"

        # Aligné en X ET en Y -> VRAI FRANCHISSEMENT
        # Maintien / Stabilisation instantanée
        ROLL_RATE, PITCH_RATE, YAW_RATE, THRUST = stabilize_drone(shared_data)
        MOTOR_FRONT_LEFT = MOTOR_FRONT_RIGHT = MOTOR_BACK_LEFT = MOTOR_BACK_RIGHT = GAZ_STATIONNAIRE
        return f"FRANCHISSEMENT :: gaz : {GAZ_STATIONNAIRE:.2f} | gate_x : {gate_x:.2f} / gate_y : {gate_y:.2f} / size : {gate_size:.2f} // roll_rate : {current_roll_rate:.2f} / yaw_rate : {current_yaw_rate:.2f}"

def stabilize_drone(data):
    """
    Stabilise le drone à plat (Roll/Pitch/Yaw à 0) 
    avec une poussée stationnaire.
    """
    imu_gyro = data.get('imu_gyro', (0.0, 0.0, 0.0))  # (roll_rate, pitch_rate, yaw_rate)
    
    current_roll_rate  = imu_gyro[0]
    current_pitch_rate = imu_gyro[1]
    current_yaw_rate   = imu_gyro[2]

    # Gain de freinage pour annuler l'inertie
    K_D = 0.20

    # Consignes nulles + freinage actif basé sur le gyro
    final_roll_rate  = 0.0 - (K_D * current_roll_rate)
    final_pitch_rate = 0.0 - (K_D * current_pitch_rate)
    final_yaw_rate   = 0.0 - (K_D * current_yaw_rate)
    
    # Gaz pour maintenir le vol stationnaire
    final_thrust = 0.55

    return final_roll_rate, final_pitch_rate, final_yaw_rate, final_thrust   