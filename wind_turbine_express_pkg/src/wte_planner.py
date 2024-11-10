#!/usr/bin/env python3

# Copyright 2024 Wind Turbine Express.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import cv2
import rclpy
import numpy as np
import os
import copy
from rclpy.node import Node
from wind_turbine_express_interfaces.msg import Thruster
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, UInt32
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.msg import ParamVec

class WTENavigationNode(Node):

    def __init__(self):
        #TEMPLATE
        super().__init__('wte_navigation_node_py')

        self.navigation = self.create_publisher(Thruster, '/aquabot/navigation/point', 10)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10)
        self.checkup_suscriber = self.create_subscription(String, '/vrx/windturbinesinspection/windturbine_checkup', self.wind_turbine_checkup_callback, 10)
        self.pinger_subscriber = self.create_subscription(ParamVec, '/aquabot/sensors/acoustics/receiver/range_bearing', self.pinger_callback, 10)
        self.current_phase_subscriber = self.create_subscription(UInt32, '/vrx/windturbinesinspection/current_phase', self.current_phase_callback, 10)
        

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.nav_point_callback)
        
        self.aquabot_coordinate = []
        self.wind_turbines_coordinates = []
        
        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632
        
        # vnuezvburb
        self.id_eolienne_checked = 0
        self.state_eolienne_checked = ""

        # gangenagna
        self.current_task = 0

        #Pinger critical windturbine 
        self.critical_wind_turbine_coordinates_calculated = False
        
        #IMU
        self.yaw = 0
        # Declare les variables ici
        #END TEMPLATE
        self.get_logger().info("Begin initialisation")

        # Précharger carte des distance euclidiennes : 

        nom_package = "wind_turbine_express_pkg"
        chemin_relatif_image_dist = "images/dist_transform.jpg"
        chemin_complet_image_dist = os.path.join(get_package_share_directory(nom_package), chemin_relatif_image_dist)

        self.carte_pixel = self.precharger_carte_distance(chemin_complet_image_dist)
        if self.carte_pixel is not None:
            self.taille_carte_dist_transform = self.carte_pixel.shape
        else:
            self.get_logger().error("ECHEC CHARGEMENT CARTE DISTANCE !")


        chemin_relatif_graph_rocher = "assets/graph_rocher.txt"
        chemin_complet_graph_rocher = os.path.join(get_package_share_directory(nom_package), chemin_relatif_graph_rocher)

        #Charger le graphe des rochers 
        self.G = self.recup_graphe(chemin_complet_graph_rocher)


        #PARAMETRE NAVIGATION
        
        # Parametre eoliennes.
        self.coordonnees_eoliennes = [] #Mise a jour dans ais callback
        self.l_graphe_eolienne = [[],[]] #Mis a jour lors de ais callback
        self.eoliennes_data_initialisee = False
        self.i_eolienne = -1
        d, self.ordre_visite = 0,(0,1,2)
        self.eolienne_vu = [False, False, False]

        self.eolienne_scanne = [False, False, False]

        self.pos_aquabot = (0,0) # Mis a jour dans gps_callback

        # liste du chemin en cours
        self.liste_chemin_en_cours = []

        #liste des points en train d'être parcouru autour d'une eolienne
        self.l_chemin_en_cours_autour_eolienne = []

        #Liste des rochers deja atteint
        self.rocher_deja_atteint = []
        self.p_eolien_deja_atteint = []

        #Critical windturbine : 
        self.critical_windturbine_coordinates = (0,0)
        self.critical_windturbine_idx = -1
        
        self.distance_point_passage_eolienne = 14
        
        self.tolerance_target_dist = 15
        
        self.get_logger().info("End of initialisationt")


    #FUNCTION TEMPLATE
    def coordinates_from_point(self, lat1, long1, lat2, long2):
        
        R = 6366354 # earth radius at latitude 48.04630
        C = 40075017 # earth meridional circumference

        long_angle = long1 - long2 # angle between the two longitudes in deg
        lat_angle = lat1 - lat2 # angle between the two latitudes in deg

        x = (long_angle/360)*C*(2/3) # distance between the two latitudes in m
        y = (lat_angle/360)*(2*np.pi*R) # distance between the two longitudes in m

        return x,y

    def nav_point_callback(self):
        """
        Publie la prochaine position a atteindre par le bateau
        """
        # self.get_logger().info("NAV CALLBAKCK CALLED")
        # self.get_logger().info(f"chemin eolienne en cours : {self.l_chemin_en_cours_autour_eolienne}")
        # self.get_logger().info(f"chemin rocher en cours : {self.liste_chemin_en_cours}")
        self.get_logger().info(f"PHASE ACTUELLE : {self.current_task}")
        self.get_logger().info(f"EOLIENNES SCANNEES : {self.eolienne_scanne}")

        if self.critical_wind_turbine_coordinates_calculated:
            self.get_logger().info(f"Critical Windturbine coordinates : {self.critical_windturbine_coordinates}")
            self.get_logger().info(f"Windturbine coordinates : {self.coordonnees_eoliennes}")

        if self.eoliennes_data_initialisee:
            next_pos, indice_eolienne_si_prochain_point_est_eolienne = self.next_point()



        else:
            self.get_logger().info("Données par encore initialisée")
            next_pos = (0,0)
        self.get_logger().info("next_pos published : ")
        self.get_logger().info(f"x : {next_pos[0]} y : {next_pos[1]}")


        prochaine_coord_bateau = next_pos

        Point_objectif_bateau = [prochaine_coord_bateau[0],prochaine_coord_bateau[1]] 
        nav_msg = Thruster() 
        nav_msg.x = float(Point_objectif_bateau[0])
        nav_msg.y = float(Point_objectif_bateau[1])
        self.navigation.publish(nav_msg)

    def ais_callback(self, msg):
        """Met a jour les coordonnées des éoliennes
        La première fois que cela publie quelque chose de non vide
        """
        self.eolienne_1_latitude = msg.poses[0].position.x
        self.eolienne_1_longitude = msg.poses[0].position.y

        self.eolienne_2_latitude = msg.poses[1].position.x
        self.eolienne_2_longitude = msg.poses[1].position.y

        self.eolienne_3_latitude = msg.poses[2].position.x
        self.eolienne_3_longitude = msg.poses[2].position.y
        
        eolienne_A_coordinate = self.coordinates_from_point(self.eolienne_1_latitude, self.eolienne_1_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_B_coordinate = self.coordinates_from_point(self.eolienne_2_latitude, self.eolienne_2_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_C_coordinate = self.coordinates_from_point(self.eolienne_3_latitude, self.eolienne_3_longitude, self.origine_latitude, self.origine_longitude)
        
        self.wind_turbines_coordinates = [eolienne_A_coordinate, eolienne_B_coordinate, eolienne_C_coordinate] # position des éoliennes
        if not self.eoliennes_data_initialisee: # On suppose que les coordonnées des éoliennes ne vont pas changer
            self.coordonnees_eoliennes = self.wind_turbines_coordinates.copy() # On met a jour les coordonnées des éoliennes
            self.l_graphe_eolienne = copy.deepcopy(self.genere_graphe_eolienne(self.coordonnees_eoliennes)) # On met a jour les graphes des éoliennes
            d,self.ordre_visite = self.calcul_ordre_parcours_eolienne()
            self.eoliennes_data_initialisee = True
            self.get_logger().info("POS_EOLIENNE_INITIALISEE : ")
            print(self.l_graphe_eolienne)
    
    def gps_callback(self, msg):
        """Met a jour la position GPS du bateau"""
        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        self.aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude) # position de l'aquabot

        self.pos_aquabot = self.aquabot_coordinate # On met a jour la position de l'aquabot
        #self.get_logger().info(f"pos bateau : x:{self.aquabot_coordinate[0]} y:{self.aquabot_coordinate[1]}")
        
        # On vérifie si il a atteint le point suivant si il en a a atteindre : 
        if len(self.l_chemin_en_cours_autour_eolienne)>0:
            i_pos_autour_eolienne,i_eolienne = self.l_chemin_en_cours_autour_eolienne[0]

            position_cible = self.l_graphe_eolienne[i_eolienne][0][i_pos_autour_eolienne]
            #self.get_logger().info(f"pos_cible : {position_cible}")
            #self.get_logger().info(f"pos_aquabot : {self.pos_aquabot}")
            #self.get_logger().info(f"dist_pos_cible : {self.dist(position_cible,self.pos_aquabot)}")
            if self.dist(position_cible,self.pos_aquabot) < self.tolerance_target_dist:
                self.point_atteint(i_eolienne)
        
        elif len(self.liste_chemin_en_cours)>0:
            i_rocher,i_eolienne = self.liste_chemin_en_cours[0]
            position_cible = self.G[0][i_rocher]
            if self.dist(position_cible,self.pos_aquabot) < self.tolerance_target_dist:
                self.point_atteint(-1)

    def wind_turbine_checkup_callback(self, msg):
        """
        Reçoit le contenu du QR code des eolienne quand un QR code est scanné
        """
        msg_dico = eval(msg.data)
        #self.get_logger().info(f"Mesasge recu : {msg_dico}")
        self.id_eolienne_checked = msg_dico["id"]
        self.state_eolienne_checked = msg_dico["state"]
        self.eolienne_scanne[self.id_eolienne_checked] = True
        self.get_logger().info(f"Data qr code received : id:{self.id_eolienne_checked} state_eolienne : {self.state_eolienne_checked}")

    def pinger_callback(self, msg):
        """
        Calcul once the coordinates of the critical wind turbine from the pinger data when task phase update to 2
        """
        if self.current_task == 2 and self.critical_wind_turbine_coordinates_calculated == False:
            for param in msg.params:
                if param.name == "bearing":
                    pinger_bearing = param.value.double_value
                if param.name == "range":
                    pinger_range = param.value.double_value
            critical_wind_turbine_theta = self.yaw + pinger_bearing
            self.critical_wind_turbine_x = pinger_range*np.cos(critical_wind_turbine_theta)
            self.critical_wind_turbine_y = pinger_range*np.sin(critical_wind_turbine_theta)
            if critical_wind_turbine_theta < 0:
                self.critical_wind_turbine_y = -self.critical_wind_turbine_y
            if np.abs(critical_wind_turbine_theta) > np.pi:
                self.critical_wind_turbine_x = -self.critical_wind_turbine_x
            self.critical_wind_turbine_coordinates_calculated = True
            self.critical_windturbine_coordinates = (self.critical_wind_turbine_x,self.critical_wind_turbine_y)

            self.get_logger().info(f"Wind_Turbine_Coordinates received : {self.critical_windturbine_coordinates}")        

            self.initialise_phase_2()

        else:
            return

        self.get_logger().info(f'pinger : bearing: {pinger_bearing}, range: {pinger_range}')
        self.get_logger().info(f'critical_wind_turbine_x: {self.critical_wind_turbine_x}, critical_wind_turbine_y: {self.critical_wind_turbine_y}')

    def imu_callback(self, msg):
        """
        Receives aquabot yaw
        """
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

    def current_phase_callback(self, msg):
        """
        Get the current task number
        """
        self.current_task = msg.data
        self.get_logger().info(f'current_task: {self.current_task}')

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw \n
        Quaternion = [x, y, z, w]
        """

        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw
    #Mets tes fonctions ici (dans la classe WTENavigationNode) en enlevant tout le graphique

    #END FUNCTION TEMPLATE

    #Mes fonctions sans le graphique
    def initialise_phase_2(self):
        windturbine_target = self.critical_windturbine_coordinates
        wind_turbine_target_idx = self.indice_plus_proche(self.coordonnees_eoliennes, windturbine_target)
        self.eolienne_vu[wind_turbine_target_idx] = False
        self.critical_windturbine_idx = wind_turbine_target_idx


    def precharger_carte_distance(self,nom_fichier):
        """
        Charge limage des distance dans la ram.
        """
        image = cv2.imread(nom_fichier)
        if image is not None:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            return None

    def mul_vect(self, v, lambd):
        """
        Multiplie 1 vecteur de taille 2 par un scalaire
        """
        return (v[0] * lambd, v[1] * lambd)

    def add_vect(self, v, v2):
        """
        Ajoute 2 vecteurs de taille 2
        """
        return (v[0] + v2[0], v[1] + v2[1])

    def get_distance_caillou(self):
        """
            Renvoie la vitesse recommandé (entre 0 et 1) en fonction de l'image des distance eulerienne
        """
        x_carte = self.taille_carte_dist_transform[0] * (self.pos_aquabot[0] + 300) / 600
        y_carte = self.taille_carte_dist_transform[1] * (self.pos_aquabot[1] + 300) / 600
        x, y = math.floor(x_carte), math.floor(y_carte)
        dec_x = x_carte - x
        dec_y = y_carte - y

        if x_carte >= self.taille_carte_dist_transform[0] - 1 or y_carte >= self.taille_carte_dist_transform[1] - 1 or x_carte < 0 or y_carte < 0:
            return 1

        px_entier = self.carte_pixel[x, y]
        px_droit = self.carte_pixel[x + 1, y]
        px_haut = self.carte_pixel[x, y + 1]
        px_coin = self.carte_pixel[x + 1, y + 1]

        px_e_partie = px_entier * (1 - dec_x) * (1 - dec_y)
        px_d_partie = px_droit * dec_x * (1 - dec_y)
        px_h_partie = px_haut * (1 - dec_x) * dec_y
        px_c_partie = px_coin * dec_x * dec_y

        total = px_e_partie + px_d_partie + px_c_partie + px_h_partie
        moyenne = total

        return moyenne

    def recup_graphe(self,nom_f):
        """
        Récupère le graphe stocké dans le fichier en argument et le renvoie
        """
        G = [[], []]
        try:
            f = open(nom_f, "r")
        except:
            f = None

        if f is not None:
            texte = f.read()
        else:
            texte = "0"
            self.get_logger().error("ECHEC CHARGEMENT GRAPHE !")

        if f is not None:
            f.close()
    
        texte_ligne = texte.split("\n")
        # print(texte_ligne)
        n = int(texte_ligne[0])
        i = 1
        # Coord des sommets
        while i <= n:
            l = texte_ligne[i].split(",")

            G[0].append((float(l[0]), float(l[1])))
            G[1].append([])
            i += 1
        # Tout les voisinages
        while i < len(texte_ligne):
            l = texte_ligne[i].split("->")

            s_origine = int(l[0])
            v_arrive = int(l[1])
            G[1][s_origine].append(v_arrive)

            i += 1

        return G

    def norme_vecteur(self,x):
        """
        Norme du vecteur x de dimension 2
        """
        return math.sqrt(x[0] ** 2 + x[1] ** 2)

    def dist(self,a, b):
        """
        Distance euclidienne entre 2 coordonnées
        """
        if type(a) == int or type(b) == int:
            self.get_logger().error("A OR B IS NOT THE GOOD TYPE FOR THE DIST FUNCTION")
            self.get_logger().error(f"A : {a} .B : {b}")
            return 999999999
        if len(a) < 2 or len(b) < 2:
            self.get_logger().error("A OR B IS NOT THE GOOD length FOR THE DIST FUNCTION")
            self.get_logger().error(f"A : {a} .B : {b}")
            return 999999999
        
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def se_croise(self, s1, s2):
        """
        Détermine si 2 segment se croisent
        """

        # une des droite est verticale
        if abs(s1[1][0] - s1[0][0]) < 0.000001 or abs(s2[1][0] - s2[0][0]) < 0.000001:
            if abs(s1[1][0] - s1[0][0]) < 0.000001 and abs(s2[1][0] - s2[0][0]) < 0.00001:
                # Les deux droites sont verticales
                return abs(s1[0][0] - s2[0][0]) < 0.000001 and not (
                        min(s1[0][1], s1[1][1] > max(s2[0][1], s2[1][1])) or max(s1[0][1],
                                                                                s1[1][1] < min(s2[0][1], s2[1][1])))
            else:
                # Si l'une des droite est verticale, on "bascule" le plan est on se retrouve avec des droite horizontale
                s1 = ((s1[0][1], s1[0][0]), (s1[1][1], s1[1][0]))
                s2 = ((s2[0][1], s2[0][0]), (s2[1][1], s2[1][0]))

        c1 = (s1[1][1] - s1[0][1]) / (s1[1][0] - s1[0][0])

        c2 = (s2[1][1] - s2[0][1]) / (s2[1][0] - s2[0][0])

        d1 = s1[0][1] - s1[0][0] * c1
        d2 = s2[0][1] - s2[0][0] * c2

        # Les droties on le meme coefficient directeur
        if (c1 - c2) == 0:
            return abs(d1 - d2) < 0.00001 and not (
                    max(s1[0][0], s1[1][0]) < min(s2[0][0], s2[1][0]) or min(s1[0][0], s1[1][0]) > max(s2[0][0],
                                                                                                    s2[1][0]))
        point_dintersection = (d2 - d1) / (c1 - c2)
        # print(point_dintersection)
        if min(s1[0][0], s1[1][0]) <= point_dintersection <= max(s1[0][0], s1[1][0]) and min(s2[0][0], s2[1][
            0]) <= point_dintersection <= max(s2[0][0], s2[1][0]):
            return True
        else:
            return False
    
    def composante_connexe(self, G, s):
        """
        Trouve la composante connexe d'un graphe G en partant du sommet s (a laide de multipe parcours en profondeur)
        """
        c_connexe = []
        n = len(G[0])
        s_vu = [False for _ in range(n)]

        def parcours_profondeur(G, s):
            if s_vu[s]:
                return
            s_vu[s] = True
            c_connexe.append(s)
            for v in G[1][s]:
                parcours_profondeur(G, v)

        parcours_profondeur(G, s)
        return c_connexe

    def trouver_chemin(self, G, depart, arrivee):
        """
        Renvoie une liste de sommet du graphe allant de depart a arrivee (minimise le nombre de sommet)
        """
        file = [depart]
        d = {depart: depart}
        s_vu =  [False for i in range( len(G[0]))]
        while len(file) != 0:
            s = file.pop(0)
            if not s_vu[s]:
                s_vu[s] = True
                
                if s == arrivee:
                    break
                for i in range(len(G[1][s])):
                    v = G[1][s][i]
                    if v not in d:
                        d[v] = s
                    file.append(v)
        
        if len(file) == 0 and s != arrivee: # Si la file de noeud a regarder est vide et que le noeud qu'on regarde n'est pas le noeud d'arrivée
            # Il n'existe pas de chemin
            self.get_logger().error(f"appel de trouver_chemin résultant en une absence de chemin ! depart : {depart};arrivee:{arrivee}")
            return []

        c = []
        s_actuel = arrivee
        while d[s_actuel] != s_actuel:
            c.append(s_actuel)
            s_actuel = d[s_actuel]
        c.append(depart)
        c.reverse()
        return c
    
    def coord_autour(self,coord):
        """
        Renvoie les coordonnées autour d'un point (utilisé lors de la création des graphes des éoliennes)
        """
        l_off = []
        d = self.distance_point_passage_eolienne
        
        l_off.append((coord[0] + d,coord[1]))
        l_off.append((coord[0] + d * math.sqrt(2) / 2, coord[1] + d * math.sqrt(2) / 2))
        l_off.append((coord[0] ,coord[1] + d))
        l_off.append((coord[0] - d * math.sqrt(2) / 2, coord[1] + d * math.sqrt(2) / 2))
        l_off.append((coord[0] - d, coord[1]))
        l_off.append((coord[0] - d * math.sqrt(2) / 2, coord[1] - d * math.sqrt(2) / 2))
        l_off.append((coord[0], coord[1] - d))
        l_off.append((coord[0] + d * math.sqrt(2) / 2, coord[1] - d * math.sqrt(2) / 2))
        
        return l_off

    def spawn_eolienne(self,G,pos_eolienne):
        """
        Ajoute l'éolienne a la position pos_eolienne au graphe G par effet de bord
        """
        pos_eolienne = (pos_eolienne[0], pos_eolienne[1])
        V = 0
        E = 1
        c_off_eolienne = self.coord_autour(pos_eolienne)

        for i in range(len(c_off_eolienne)):
            coord_decalee = c_off_eolienne[i]

            G[V].append(coord_decalee)
            G[E].append([])

        for i in range(len(c_off_eolienne)):
            n = len(G[V])
            coord_decalee = c_off_eolienne[i]

            plus_proche = -1
            d1 = 999999999
            deux_proche = -1
            d2 = 999999999
            for j in range(n):
                if i == j:
                    continue
                c2 = G[V][j]

                d = self.dist(coord_decalee, c2)
                if d < max(d1, d2):

                    if d < d1:
                        tempd1 = d1
                        tempi1 = plus_proche
                        plus_proche = j
                        d1 = d
                        if tempd1 < d2:
                            d2 = tempd1
                            deux_proche = tempi1

                    elif d < d2:
                        d2 = d
                        deux_proche = j

            G[E][i].append(plus_proche)
            G[E][i].append(deux_proche)

    def genere_graphe_eolienne(self,l_eolienne):
        """
        Renvoie une liste qui contient a chaque indice i, le graphe généré par la position de l'éolienne dans l_eolienne a l'indice i
        """
        l = []
        for i in range(len(l_eolienne)):
            graphe = [[],[]]
            self.spawn_eolienne(graphe,l_eolienne[i])
            l.append(graphe)
        return l

    def calcul_cout_ordre(self,liste_ordre):
        p0 = self.pos_aquabot
        p1 = self.coordonnees_eoliennes[liste_ordre[0]]
        d = 0
        for i in range(len(liste_ordre)):
            d += self.dist(p0,p1)
            p0 = p1
            if i != len(liste_ordre) - 1:
                p1 = self.coordonnees_eoliennes[liste_ordre[i+1]]
            else:
                p1 = self.pos_aquabot

        return d

    def calcul_ordre_parcours_eolienne(self):
        """
        Renvoie la liste des indice dans lesquel parcourir les éoliennes ainsi que la distance total (distance,ordre)
        """
        ordre_possible = [(0,1,2),(0,2,1),(1,0,2),(1,2,0)]
        cout_min = self.calcul_cout_ordre(ordre_possible[0])
        meilleur_ordre = ordre_possible[0]
        for ord in ordre_possible:
            cout = self.calcul_cout_ordre(ord)
            if cout < cout_min:
                cout_min = cout
                meilleur_ordre = ord

        return cout_min,meilleur_ordre
    
    #AJOUTER UNE FINCTION QUI UNE FOIS LE QR CODE distance_point_passage_eolienne
    #Vide ke chemin eolienne pour aller vers la prochaine
    
    def point_atteint(self,i_eolienne):
        """
        Quand on a atteint le point précédent donné par la fonction, appeler cette fonction marque le point comme étant atteint et met a jour les variables
        """
        if len(self.l_chemin_en_cours_autour_eolienne)>0:
            i_point,i_eolien = self.l_chemin_en_cours_autour_eolienne.pop(0)
            p_atteint = self.l_graphe_eolienne[i_eolien][0][i_point]
            self.p_eolien_deja_atteint.append(p_atteint)
        elif len(self.liste_chemin_en_cours) > 0:
            i_point,i_eolien = self.liste_chemin_en_cours.pop(0)
            p_atteint = self.G[0][i_point]
            # On a atteint un rocher, on l'ajoute dans les rochers déjà atteint
            self.rocher_deja_atteint.append(i_point)
        else:
            p_atteint = self.pos_aquabot
        
        if i_eolienne != -1:
            p_e = self.coordonnees_eoliennes[i_eolienne]
            #print("EOLIENNE ATTEINTE")
            self.eolienne_vu[i_eolienne] = True
            # on remet a 0 les rochers atteint.
            self.rocher_deja_atteint.clear()

    def next_point(self):
        """
        Renvoie la prochaine position à atteindre depuis la position self.pos_aquabot
        Prend en compte les éoliennes déjà atteinte
        """
        #Si on est proche de l'eolienne qu'on veut atteindre, on y clear le chemin en cours pour aller au else
        
        if self.dist(self.pos_aquabot,self.coordonnees_eoliennes[self.i_eolienne]) < 1.6*self.distance_point_passage_eolienne+self.tolerance_target_dist and len(self.l_chemin_en_cours_autour_eolienne) == 0:
            self.liste_chemin_en_cours.clear()

        #Si on est en train de faire le tour d'une eolienne, on continue
        if len(self.l_chemin_en_cours_autour_eolienne) >0:
            i_point, i_eolien = self.l_chemin_en_cours_autour_eolienne[0]
            return self.l_graphe_eolienne[i_eolien][0][i_point],i_eolien
        #Si on est en train de faire le tour d'un rocher, on continue
        elif len(self.liste_chemin_en_cours) != 0:
            i_point, i_eolien = self.liste_chemin_en_cours[0]
            return self.G[0][i_point],-1
        #Sinon on recalcule le chemin.
        else:
            next_pos,indice_eolienne_si_eolienne = self.prochain_points_etape1()
            return next_pos,indice_eolienne_si_eolienne

    def prochain_points_etape1(self):
        """
        Détermine le prochain point a atteindre en fonction de la progression,
        Crée un chemin si des rochers ou des éoliennes sont a proximité
        Met a jour les variables de next_point pour qu'un tel chemin soit retenu
        Renvoie la prochaine position et l'indice de l'éolienne rencontrée (si le prochain point a atteindre est une éolienne)
        """

        i_actuel = 0

        while i_actuel < len(self.eolienne_vu) and self.eolienne_vu[self.ordre_visite[i_actuel]]:
            i_actuel += 1
    
        """
            if i_actuel == len(self.eolienne_vu):
            self.get_logger().fatal("FINI")
            self.liste_chemin_en_cours.clear()
            return (0,0),-1
        """
        if i_actuel == len(self.eolienne_vu):
            self.get_logger().info("EOLIENNE TOUTE VISITEE, PASSAGE PHASE 2")
            self.get_logger().info(f"PHASE ACTUELLE : {self.current_task}")

        if self.current_task == 1 and i_actuel <= 2:
            i_eolienne = self.ordre_visite[i_actuel]
        else:
            self.get_logger().info("TOUTE LES EOLIENNES ONT ETE VISITEE")
            if self.critical_wind_turbine_coordinates_calculated:
                i_eolienne = self.critical_windturbine_idx
            else:
                self.get_logger().error("PHASE 2 ENCLENCHEE ET EOLIENNE PAS CALCULEE")
                return (0,0),-1

        
        #Position de l'objectif d'apres
        if i_actuel < 2:
            pos_suivante = self.coordonnees_eoliennes[self.ordre_visite[(i_actuel + 1)]]
        else:
            pos_suivante = (0,0)
        #self.i_eolienne est l'indice de l'éolienne que dont l'on cherche a faire le tour.
        self.i_eolienne = i_eolienne
        
        #C'est la position de l'éolienne que l'on cherche a atteindre
        eolinne_cible_pos = self.coordonnees_eoliennes[i_eolienne]

        self.get_logger().info(f"Etat des éolienne vues : {self.eolienne_vu}")

        # Si la distance avec l'eolienne cible est faible -> on met en file d'attente les point qui passent autour de l'éolienne.
        # On pourrait ajouter d'autre point pour fiare en sorte que le bateau s'éloigne de l'éolienne au point le plus proche de la suivante.
        if self.dist(eolinne_cible_pos, self.pos_aquabot) < self.distance_point_passage_eolienne + self.tolerance_target_dist:
            self.rocher_deja_atteint.clear()
            self.liste_chemin_en_cours.clear()

            self.get_logger().info("CONSTRUCTION DU CHEMIN AUTOUR DE LEOLIENNE EN EXPLORATION")
            
            graphe_de_eolienne_cible = self.l_graphe_eolienne[i_eolienne]

            #Chemin a parcourir pour faire le tour de léolienne.
            chemin_autour_eolienne_sans_indice = copy.deepcopy(self.calcul_exploration_eolienne(i_eolienne,self.pos_aquabot,False))
            
            #On ajoute quelque points pour qu'il fasse bien le tour de l'éolienne.
            chemin_autour_eolienne_sans_indice = chemin_autour_eolienne_sans_indice + chemin_autour_eolienne_sans_indice[0:2]

            # On ajoute des points pour qu'il s'écarte de l'éolienen lorsque il est le plus proche de l'autre

            idx_dernier_du_chemin = chemin_autour_eolienne_sans_indice[-1]

            idx_proche_eolienne_suivante = self.indice_plus_proche(graphe_de_eolienne_cible[0],pos_suivante)

            l = []
            i = idx_dernier_du_chemin
            n = len(graphe_de_eolienne_cible[0])
            while i != idx_proche_eolienne_suivante:
                l.append(i)
                i = (i + 1) % n
            
            self.get_logger().info(f"SUIT BORD GRAPHE : DEPART {idx_dernier_du_chemin}; NEXT {idx_proche_eolienne_suivante}; EVITE {chemin_autour_eolienne_sans_indice[-2]}")
            #Il ajoute rien haha hoho

            self.get_logger().info(f"ajoute par suit_bord_graphe : {l}")
            self.get_logger().info(f"CHEMIN AUTOUR EOLIENNE : {chemin_autour_eolienne_sans_indice}")
            for elt in l:
                chemin_autour_eolienne_sans_indice.append(elt)

            self.l_chemin_en_cours_autour_eolienne = [(indice,self.i_eolienne) for indice in (chemin_autour_eolienne_sans_indice)]
            print(self.l_chemin_en_cours_autour_eolienne)
            graphe_de_eolienne_cible =  self.l_graphe_eolienne[self.i_eolienne][0]
            return graphe_de_eolienne_cible[chemin_autour_eolienne_sans_indice[0]],i_eolienne

        
        # Regarder la vitesse desire en fonctio nde la carte des distances

        d_caillou = self.get_distance_caillou()

        # trouver un rayon de surveillance das lequel on veut attraper tout les rochers
        r = 100
        if d_caillou < 125:
            r = 100 #Rayon autour duquel on regarde les caillous :3

        # Regarder la proximité des rochers autour du rayon

        V = self.G[0]
        n = len(V)
        caillou_a_portee = []
        for i in range(n):
            if self.dist(self.pos_aquabot, V[i]) < r and i not in self.rocher_deja_atteint: #On ignore les rochers déjà atteint
                caillou_a_portee.append(i)

        eoliennes_a_portee = []
        for i in range(len(self.coordonnees_eoliennes)):
            if self.dist(self.pos_aquabot,self.coordonnees_eoliennes[i]) < r and i != i_eolienne:
                eoliennes_a_portee.append(i)
        

        RANGE = r
        vitesse_actuelle = RANGE

        self.get_logger().info(f"Eolienne visée : ({self.coordonnees_eoliennes[i_eolienne][0]},{self.coordonnees_eoliennes[i_eolienne][1]})")
        #C'est le vecteur qui de l'aquabot à l'éolienne
        vect_eolienne = (self.coordonnees_eoliennes[i_eolienne][0] - self.pos_aquabot[0], self.coordonnees_eoliennes[i_eolienne][1] - self.pos_aquabot[1])

        n_vect = self.norme_vecteur(vect_eolienne)
        vect_eolienne = self.mul_vect(vect_eolienne,1/n_vect)
        
        vect_eolienne = self.mul_vect(vect_eolienne, min(vitesse_actuelle,self.dist(eolinne_cible_pos, self.pos_aquabot)))

        # Regarder si le vecteur "traverse la ligne des points" ou "va vers l'intérieur des rochers"

        points_qui_coupent = []
        s_direction = (self.pos_aquabot, self.add_vect(self.pos_aquabot, vect_eolienne))
        for i in range(len(caillou_a_portee)):
            caillou = caillou_a_portee[i]
            v_caillou = self.G[1][caillou]
            for j in range(len(v_caillou)):
                voisin = v_caillou[j]
                segment = (self.G[0][caillou], self.G[0][voisin])
                if self.se_croise(segment, s_direction):
                    if voisin not in self.rocher_deja_atteint: #On ignore les rochers que l'on a déjà atteint.
                        points_qui_coupent.append(voisin)

        
        graphe_position_autour_eolienne_cible = self.l_graphe_eolienne[i_eolienne][0]
        chemin_autour_eolienne = copy.deepcopy(self.calcul_exploration_eolienne(i_eolienne,self.pos_aquabot,False))
        pos_du_premier_point = graphe_position_autour_eolienne_cible[chemin_autour_eolienne[2]]

        # Si aucun rocher ne s'interposent continuer en mode on s'en fiche, c'est à dire que il n'y a aucun rocher qui se coupent
        if len(points_qui_coupent) == 0:
            return pos_du_premier_point,i_eolienne


        # On trouve le point le plus proche des rochers trouvé
        coord_point_qui_coupent = [self.G[0][i] for i in points_qui_coupent]
        indice_rocher_plus_proche_bateau = points_qui_coupent[self.indice_plus_proche(coord_point_qui_coupent,self.pos_aquabot)]
        coord_rocher_plus_proche_bateau = self.G[0][indice_rocher_plus_proche_bateau]
        self.get_logger().info(f"coord rocher plus proche bateau : {coord_rocher_plus_proche_bateau}")

        indice_rocher_connecte_au_caillou = self.composante_connexe(self.G, indice_rocher_plus_proche_bateau)
        
        self.get_logger().info(f"Rocher connecté au caillou : {indice_rocher_connecte_au_caillou}")
        
        coord_connecte_au_caillou = [self.G[0][i] for i in indice_rocher_connecte_au_caillou]
        # On met pos du premier poit a atteindre
        indice_plus_proche_eolienne = self.indice_plus_proche(coord_connecte_au_caillou,pos_du_premier_point)
        # Au lieu de regarder les plus prochzs, il faut rrgarder lesquelles on peut passser pour contourner les rochers

        chemin_dindices_rocher_a_parcourir = self.trouver_chemin(self.G, indice_rocher_plus_proche_bateau, indice_rocher_connecte_au_caillou[indice_plus_proche_eolienne])
        # Le chemin existe, car, indice_rocher_plus_proche_bateau et indice_plus_proche_eolienne sont dans une meme composante connexe de G

        for idx in chemin_dindices_rocher_a_parcourir:
            self.liste_chemin_en_cours.append((idx,-1))

        return coord_rocher_plus_proche_bateau,-1


    def indice_plus_proche(self,l,p):
        """
        Renvoie l'indice de la liste de coordonnée qui minimise la distance avec p
        """
        mini = 9999999
        i_proche = -1
        for i in range(len(l)):
            if self.dist(l[i],p) < mini:
                mini = self.dist(l[i],p)
                i_proche = i
        return i_proche
    
    def calcul_exploration_eolienne(self, i_eolienne,pos_bateau,saute_point):
        g_eolienne = self.l_graphe_eolienne[i_eolienne]
        n_point = len(g_eolienne)
        point_proche = self.indice_plus_proche(g_eolienne[0],pos_bateau)
        if saute_point:
            l_sommet = self.suit_bord_graphe(g_eolienne,(point_proche + 2) % n_point)
        else:
            l_sommet = self.suit_bord_graphe(g_eolienne,point_proche)
        return l_sommet

    def suit_bord_graphe(self,G,depart):
        V = G[0]
        chemin = []
        n = len(V)
        i = depart
        
        for _ in range(n):
            chemin.append(i)
            i = (i+1)%n
        
        return chemin

def main(args=None):
    rclpy.init(args=args)
    wte_navigation_node = WTENavigationNode()
    rclpy.spin(wte_navigation_node)
    wte_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()