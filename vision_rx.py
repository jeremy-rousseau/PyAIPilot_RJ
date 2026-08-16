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
            daemon=False
        )
        self.is_running = True
        self.thread.start()

        # Configuration pour l'enregistrement MP4
        self.video_writer = None
        self.filename = "vol_drone.mp4"
        self.fps = 30.0  




    def get_thread_for_join(self):
        self.is_running = False
        cv2.destroyAllWindows()
        if self.video_writer is not None:
            self.video_writer.release()
            print(f"[VISION] Enregistrement vidéo sauvegardé dans : {self.filename}")   
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
        try:
            height, width = img.shape[:2]
            cx, cy = width // 2, height // 2

            # Initialisation unique du VideoWriter à la première image reçue
            if self.video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(
                    self.filename, 
                    fourcc, 
                    self.fps, 
                    (width, height)
                )

            # 1. Conversion en HSV pour la couleur de base
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Tolérance sur l'orange
            lower_orange = np.array([2, 100, 80])
            upper_orange = np.array([28, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Nettoyage morphologique
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 2. Extraction des contours arborescents
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            gate_found = False
            gate_x, gate_y = 0.0, 0.0
            target_size_ratio = 0.0

            if contours and hierarchy is not None:
                hierarchy = hierarchy[0]  # Aplatissement de la matrice

                for i, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area < 800:  # Filtrage des faux positifs trop petits
                        continue

                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h       

                    if 0.8 <= aspect_ratio <= 1.2:
                        child_idx = hierarchy[i][2]
                        has_child = child_idx != -1

                        if has_child:
                            gate_found = True
                            
                            # Calcul de la surface du trou (enfant) pour estimer la distance
                            child_contour = contours[child_idx]
                            child_area = cv2.contourArea(child_contour)
                            
                            # Calcul de la surface relative de la porte par rapport à l'image
                            target_size_ratio = child_area / (width * height)

                            # Calcul du centre de la porte dans l'image
                            gate_pixel_x = x + (w // 2)
                            gate_pixel_y = y + (h // 2)
                            
                            # NORMALISATION DES COORDONNÉES (-1.0 à +1.0)
                            # Axe X : -1.0 (Gauche), +1.0 (Droite)
                            gate_x = (gate_pixel_x - cx) / (width / 2)
                            
                            # Axe Y : +1.0 (Haut), -1.0 (Bas)
                            gate_y = (gate_pixel_y - cy) / (height / 2)

                            race_started = self.data.get('race_started', False)
                            
                            # Dessin sur l'image
                            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            cv2.circle(img, (gate_pixel_x, gate_pixel_y), 6, (0, 0, 255), -1)
                            break  # Porte principale identifiée

            # Dessin du viseur central (Blanc)
            cv2.line(img, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 2)
            cv2.line(img, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 2)

            # Mise à jour de la mémoire partagée
            if gate_found:
                self.data['gate_visible'] = True
                self.data['gate_x'] = gate_x
                self.data['gate_y'] = gate_y
                self.data['gate_size'] = target_size_ratio
            else:
                self.data['gate_visible'] = False
                self.data['gate_x'] = None
                self.data['gate_y'] = None
                self.data['gate_size'] = None
                logger.debug("[VISION 🎥] Aucune porte orange visible.")

            # Affichage et sauvegarde vidéo
            cv2.imshow("Vision du drone", img)
            # cv2.imshow("Masque Orange", mask)
            
            if self.video_writer is not None:
                self.video_writer.write(img)
                
            cv2.waitKey(1)

        except Exception as e:
            print(f'Error processing frame: {str(e)}')

        return
