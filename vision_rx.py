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

            #########################################
            img_height, img_width, _ = img.shape

            # Initialisation unique du VideoWriter à la première image reçue
            if self.video_writer is None:
                # Utilisation du codec 'mp4v' pour générer un fichier .mp4
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(
                    self.filename, 
                    fourcc, 
                    self.fps, 
                    (img_width, img_height)
                )
            ##########################################





            # 1. Conversion en HSV pour la couleur de base
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Large tolérance sur l'orange pour accepter le mélange avec le bleu transparent
            lower_orange = np.array([2, 100, 80])
            upper_orange = np.array([28, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Nettoyage morphologique pour boucher les trous faits par les inscriptions blanches
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 2. Extraction des contours avec structure ARBORESCENTE (Parent -> Enfant)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            gate_found = False
            gate_x, gate_y = 0.0, 0.0

            if contours and hierarchy is not None:
                hierarchy = hierarchy[0] # On aplatit la matrice d'arborescenc

                for i, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area < 800:  # Trop petit pour être la porte principale TODO quand on s'approche de la porte
                        continue

                    # Vérification géométrique : est-ce que c'est à peu près carré ?
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h       

                        # Un carré parfait a un ratio de 1.0. On accepte entre 0.8 et 1.2 avec la perspective
                    if 0.8 <= aspect_ratio <= 1.2:

                        # --- LE FILTRE MAGIQUE : La porte a-t-elle un trou ? ---
                        # hierarchy[i][2] donne l'index du premier contour ENFANT à l'intérieur
                        # Si la valeur n'est pas -1, cela veut dire qu'il y a une forme géométrique dedans (le trou !)
                        has_child = hierarchy[i][2] != -1

                        if has_child:
                            # On a trouvé notre structure : un carré orange avec un trou dedans !
                            gate_found = True
                            
                            # Calcul du centre exact de la structure
                            gate_pixel_x = x + (w // 2)
                            gate_pixel_y = y + (h // 2)
                            
                            # Normalisation pour ton contrôleur (-1.0 à +1.0)
                            gate_x = (gate_pixel_x - cx) / (width / 2)
                            gate_y = (gate_pixel_y - cy) / (height / 2)

                            logger.info(f"[VISION] Porte trouvée ! X={gate_x:.2f} | Y={gate_y:.2f}")#| Ratio={target_size_ratio:.3f}
                            
                            # Dessin du rectangle englobant principal
                            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            cv2.circle(img, (gate_pixel_x, gate_pixel_y), 6, (0, 0, 255), -1)
                            break # On a notre porte, on arrête la recherche 
            
            # --- AFFICHAGE DU VISEUR CENTRAL ---
            cv2.line(img, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 2)
            cv2.line(img, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 2)

            # target_size_ratio = child_area / (img_width * img_height)

            # Transmission des données dans la mémoire partagée
            if gate_found:
                self.data['gate_visible'] = gate_found
                self.data['gate_x'] = gate_x
                self.data['gate_y'] = gate_y
                # self.data['gate_size'] = target_size_ratio
            else :
                self.data['gate_visible'] = False
                self.data['gate_x'] = None
                self.data['gate_y'] = None
                # self.data['gate_size'] = None
                logger.debug("[VISION] Aucune porte orange visible.")

            #Affiche l'image brute avec les rectangles
            cv2.imshow("Vision du drone", img)
            #Affiche ce que le drone "voit" en binaire
            cv2.imshow("Masque Orange", mask)
            ################
            self.video_writer.write(img)
            ################
            cv2.waitKey(1)

        except Exception as e:
                print(f'Error processing frame: {str(e)}')
 
        return

    ######
        # image is your FPV camera frame in JPEG format
        # try:
        #     #Variable de l'image
        #     img_height, img_width, _ = img.shape
        #     img_center_x = img_width / 2
        #     img_center_y = img_height / 2

        #     #Conversion en HSV pour la détection de couleur
        #     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
        #     #Définition de la plage d'orange (Teinte entre 5 et 25)
        #     lower_color = np.array([5, 50, 50])
        #     upper_color = np.array([25, 255, 255])
            
        #     #Création du masque et nettoyage du bruit
        #     mask = cv2.inRange(hsv, lower_color, upper_color)
        #     kernel = np.ones((5,5), np.uint8)
        #     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # Enlève les petits points
            
        #     #Recherche des contours
        #     contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        #     gate_found = False

        #     if hierarchy is not None:
        #         for i, cnt in enumerate(contours):
        #             #On cherche un contour qui a un "Enfant" (le trou de la porte)
        #             #hierarchy[0][i][2] != -1 signifie qu'il y a un enfant
        #             if hierarchy[0][i][2] != -1:
        #                 #Cadre extérieur de la porte
        #                 x, y, w, h = cv2.boundingRect(cnt)

        #                 child_idx = hierarchy[0][i][2]
        #                 child_area = cv2.contourArea(contours[child_idx])
                            
        #                 if child_area > 500: 
        #                     xi, yi, wi, hi = cv2.boundingRect(contours[child_idx])
        #                     child_center_x = xi + wi//2
        #                     child_center_y = yi + hi//2

        #                     # Calcul des écarts normalisés (-1.0 à 1.0)
        #                     target_center_x = (child_center_x - img_center_x) / img_center_x
        #                     target_center_y = (child_center_y - img_center_y) / img_center_y
        #                     target_size_ratio = child_area / (img_width * img_height)

        #                     # Transmission des données
        #                     self.data['gate_visible'] = True
        #                     self.data['gate_x'] = target_center_x
        #                     self.data['gate_y'] = target_center_y
        #                     self.data['gate_size'] = target_size_ratio

        #                     logger.info(f"[VISION] Porte trouvée ! X={target_center_x:.2f} | Y={target_center_y:.2f} | Ratio={target_size_ratio:.3f}")

        #                     gate_found = True

        #                     ######
        #                     #Pour l'affichage :
        #                         #En vert : Le cadre extérieur
        #                     cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        #                     cv2.putText(img, "PORTE DETECTEE", (x, y-10), 
        #                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        #                     #En rouge : Le trou intérieur (pour vérifier)
        #                     xi, yi, wi, hi = cv2.boundingRect(contours[child_idx])
        #                     cv2.rectangle(img, (xi, yi), (xi + wi, yi + hi), (0, 0, 255), 2)
        #                     cv2.putText(img, f"Trou de la porte - Area: {int(child_area)}", (xi, yi-10),
        #                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        #                     #Le centre (en bleu)
        #                     cv2.circle(img, (xi + wi//2, yi + hi//2), 7, (255, 0, 0), -1)

                            #On stocke l'image modifiée dans le dictionnaire partagé pour le thread principal
                            # self.data['last_img'] = img
                            #####


                            # break # On prend la première porte valide trouvée
            
        #     #Affiche l'image brute avec les rectangles
        #     cv2.imshow("Vision du drone", img)
        #     #Affiche ce que le drone "voit" en binaire
        #     cv2.imshow("Mask Orange", mask)
            
        #     cv2.waitKey(1)

        #     if not gate_found:
        #         self.data['gate_visible'] = False
        #         self.data['gate_x'] = None
        #         self.data['gate_y'] = None
        #         self.data['gate_size'] = None

        #         logger.debug("[VISION] Aucune porte orange visible.")
            
        # except Exception as e:
        #         print(f'Error processing frame: {str(e)}')    

        # return
    
    
        