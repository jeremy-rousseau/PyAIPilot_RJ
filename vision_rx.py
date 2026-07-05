import socket
import struct
import threading

import cv2
import numpy as np

##############
import logging
logger = logging.getLogger("DronePilot")
##############

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 5600


class VisionRX:

    def __init__(self, data):
        self.data = data
        self.thread = threading.Thread(
            target=self._vision_loop,
            daemon=True
            #Pour éviter problème, lors de la fin du process
            # daemon=False
        )
        self.is_running = True
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _vision_loop(self):
        header_format = "<IHHIIQ"
        header_sz = struct.calcsize(header_format)
        frames = {}  # frame_id -> received associated frame data

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT))
        print("Listening for camera frames...")

        while self.is_running:
            packet, addr = sock.recvfrom(65536)  # max UDP size

            header = packet[:header_sz]
            payload = packet[header_sz:]

            # frame_id - identifier for this vision frame
            # chunk_id - identifier for this chunk packet of data of this frame
            # total_chunks - total number of chunk packets that make up this frame
            # jpeg_size - full size of jpeg data
            # payload_size - size of this packet
            # sim_time_ns - frame's epoch timestamp in ns on the server
            frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns = struct.unpack(header_format, header)

            if frame_id not in frames:
                frames[frame_id] = {
                    "chunks": {},
                    "total": total_chunks,
                    "size": jpeg_size,
                    "time": sim_time_ns
                }

            frames[frame_id]["chunks"][chunk_id] = payload

            # Check if frame is complete
            if len(frames[frame_id]["chunks"]) == total_chunks:
                jpeg_bytes = bytearray()

                frame_complete = True
                for i in range(total_chunks):
                    if i not in frames[frame_id]["chunks"]:
                        print('Missing packet %s in frame %s' % (i, frame_id,))
                        frame_complete = False
                        continue
                    jpeg_bytes.extend(frames[frame_id]["chunks"][i])

                if not frame_complete:
                    del frames[frame_id]
                    continue

                img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if image is not None:
                    self.process_frame(frame_id, image)
                else:
                    print(f"Failed to decode frame: {frame_id}")

                del frames[frame_id]


    def process_frame(self, frame_id, img):
        # image is your FPV camera frame in JPEG format
        try:
            #Variable de l'image
            img_height, img_width, _ = img.shape
            img_center_x = img_width / 2
            img_center_y = img_height / 2

            #Conversion en HSV pour la détection de couleur
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            #Définition de la plage d'orange (Teinte entre 5 et 25)
            lower_color = np.array([5, 50, 50])
            upper_color = np.array([25, 255, 255])
            
            #Création du masque et nettoyage du bruit
            mask = cv2.inRange(hsv, lower_color, upper_color)
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # Enlève les petits points
            
            #Recherche des contours
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

            gate_found = False

            if hierarchy is not None:
                for i, cnt in enumerate(contours):
                    #On cherche un contour qui a un "Enfant" (le trou de la porte)
                    #hierarchy[0][i][2] != -1 signifie qu'il y a un enfant
                    if hierarchy[0][i][2] != -1:
                        #area = cv2.contourArea(cnt)
                        #if area > 2000: 

                        child_idx = hierarchy[0][i][2]
                        child_area = cv2.contourArea(contours[child_idx])
                            
                        if child_area > 500: 
                            xi, yi, wi, hi = cv2.boundingRect(contours[child_idx])
                            child_center_x = xi + wi//2
                            child_center_y = yi + hi//2

                            # Calcul des écarts normalisés (-1.0 à 1.0)
                            target_center_x = (child_center_x - img_center_x) / img_center_x
                            target_center_y = (child_center_y - img_center_y) / img_center_y
                            target_size_ratio = child_area / (img_width * img_height)

                            # Transmission des données
                            self.data['gate_visible'] = True
                            self.data['gate_x'] = target_center_x
                            self.data['gate_y'] = target_center_y
                            self.data['gate_size'] = target_size_ratio

                            logger.info(f"[VISION] Porte trouvée ! X={target_center_x:.2f} | Y={target_center_y:.2f} | Ratio={target_size_ratio:.3f}")

                            gate_found = True
                            break # On prend la première porte valide trouvée
                        
            if not gate_found:
                self.data['gate_visible'] = False
                self.data['gate_x'] = None
                self.data['gate_y'] = None
                self.data['gate_size'] = None

                logger.debug("[VISION] Aucune porte orange visible.")
            
        except Exception as e:
                print(f'Error processing frame: {str(e)}')    

        return
        