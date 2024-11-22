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
import rclpy
import numpy as np
import os
import copy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from wind_turbine_express_interfaces.msg import Thruster
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, UInt32
from ament_index_python.packages import get_package_share_directory
import tool_planner as tp

class WTENavigationNode(Node):

    def __init__(self):
        super().__init__('wte_planner_node_py')

        self.get_logger().info("WTE_planner : Début d'initialisation.")

        self.reentrant_group = ReentrantCallbackGroup()

        #Publisher et Subscriber
        self.navigation = self.create_publisher(Thruster, '/aquabot/navigation/point', 10, callback_group=self.reentrant_group)
        self.chat_pub = self.create_publisher(String, '/aquabot/chat', 5, callback_group=self.reentrant_group)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10, callback_group=self.reentrant_group)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10, callback_group=self.reentrant_group)
        self.checkup_subscriber = self.create_subscription(String, '/vrx/windturbinesinspection/windturbine_checkup', self.wind_turbine_checkup_callback, 10, callback_group=self.reentrant_group)
        self.current_phase_subscriber = self.create_subscription(UInt32, '/vrx/windturbinesinspection/current_phase', self.current_phase_callback, 10, callback_group=self.reentrant_group)
        self.critical_wind_turbine_subscriber = self.create_subscription(Thruster, '/aquabot/critical_wind_turbine_coordinates', self.critical_wind_turbine_callback, 10, callback_group=self.reentrant_group)

        # Publisher
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.nav_point_callback)
        
        self.aquabot_coordinate = []
        self.wind_turbines_coordinates = []
        
        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632
        
        # Checkup subscriber
        self.id_eolienne_checked = 0
        self.state_eolienne_checked = ""

        # Pinger critical windturbine 
        self.critical_wind_turbine_coordinates_calculated = False

        # Current phase subscriber
        self.current_task = 0 # Etape actuel du Challenge

        # Précharger graphe des rochers : 
        nom_package = "wind_turbine_express_pkg"
        chemin_relatif_graph_rocher = "assets/graph_rocher.txt"
        chemin_complet_graph_rocher = os.path.join(get_package_share_directory(nom_package), chemin_relatif_graph_rocher) # Chemin vers le fichier contenant le graphe.

        # GRAPHE DES ROCHERS (point autour des rochers, permettant de les contourner.)
        self.G = self.recup_graphe(chemin_complet_graph_rocher) # Charger le graphe des rochers 
        self.liste_chemin_en_cours = [] # liste des indices des rochers en train d'être parcouru.
        self.rocher_deja_atteint = [] # Liste des indices des rochers déjà atteint. Collisions avec ignoré.

        # PARAMETRE NAVIGATION
        self.PHASE_STABILISATION_ENCLENCHEE = False # Signal le début de la phase de stabilisation et empêche la publication de nouveau point.
        self.pos_aquabot = (0,0) # Mis a jour dans gps_callback Position de l'aquabot.
        self.rayon_detection_rocher = 100 # Rayon de detection a partir duquel l'quabot détecte la présence d'obstacles
        # Parametre eoliennes. Initialisées correctement lors de ais callback
        self.coordonnees_eoliennes = [] # Coordonnées des éoliennes
        self.n_eolienne = 0 # Nombre d'éolienne
        self.l_graphe_eolienne = [[],[]] # Utilisée lorsqu'on parcours une éolienne dans la phase 1
        self.l_graphe_eolienne_critique = [[],[]] # Utilisée lorsqu'on parcours une éolienne dans un état critique.
        self.i_eolienne = -1 # Eolienne en cours de visite actuelle. 
        self.ordre_visite = (0,1,2) # Ordre de visite non initialisées
        self.eolienne_vu = [] # Eolienne pas encore explorées initalement
        self.eolienne_scanne = [] # Eolienne pas encore scannées initialement
        self.eoliennes_data_initialisee = False # Données des éolienne non initialisée de base
        self.positions_eolienne_scanne = [] # Stocke les positions des éoliennes scannées
        self.l_chemin_en_cours_autour_eolienne = [] #liste des points en train d'être parcouru autour d'une eolienne

        # Critical windturbine : (Mis a jour lors du critical_wind_turbine_callback)
        self.critical_windturbine_coordinates = (0,0) # Position de léolienne dans un état critique
        self.critical_windturbine_idx = -1 # Indice de l'éolienne dans un état critique
        self.critical_wind_turbine_coordinates_calculated = False # Indique si les varialbe concernant l'éolienne en état critique on été initialisée.
        
        # Paramètres de la navigation.

        self.distance_point_passage_eolienne_CRITIQUE = 15 # Distance des points de passage autour de l'éolienne en état critique.
        self.distance_point_passage_eolienne = 14 # Distance des points de passage autour des éolienne lors de la phase 1
        
        self.tolerance_target_dist = 15 # Distance de tolérance à partir duquel on considère que les points sont atteints.
        
        self.get_logger().info("WTE_Planner : initialisation terminée.")

    # FONCTION DE CALLBACK.
    
    def coordinates_from_point(self, lat1, long1, lat2, long2):
        
        R = 6366354 # earth radius at latitude 48.04630
        C = 40075017 # earth meridional circumference

        long_angle = long1 - long2 # angle between the two longitudes in deg
        lat_angle = lat1 - lat2 # angle between the two latitudes in deg

        x = (long_angle/360)*C*(2/3) # distance between the two latitudes in m
        y = (lat_angle/360)*(2*np.pi*R) # distance between the two longitudes in m

        return x,y

    def publier_coord_bateau(self,pos,msg):
        self.get_logger().info(f"[{msg}] : ({round(pos[0],2)},{round(pos[1],2)})")
        nav_msg = Thruster() 
        nav_msg.x = float(pos[0])
        nav_msg.y = float(pos[1])
        self.navigation.publish(nav_msg)  

    def coord_critique_qr_code(self):
        point_a_atteindre = self.moyenne_liste_coord(self.positions_eolienne_scanne[self.critical_windturbine_idx])
        # vect_eolienne_point = self.add_vect(point_a_atteindre,self.mul_vect(self.coordonnees_eoliennes[self.critical_windturbine_idx],-1))
        # vect_eolienne_point = self.mul_vect(vect_eolienne_point,self.distance_point_passage_eolienne_CRITIQUE/self.norme_vecteur(vect_eolienne_point))
        # On ne scale pas le point a atteindre pour éviter des bugs de maths mais ca fonctionne normalement
        return point_a_atteindre

    def nav_point_callback(self):
        """
        Publie la prochaine position a atteindre par le bateau
        """

        if self.PHASE_STABILISATION_ENCLENCHEE: # Si la phase de stabilisation a été enclenché : On renvoie la position a atteindre
            Point_objectif_bateau = self.coord_critique_qr_code()
            self.publier_coord_bateau(Point_objectif_bateau,"STABILISATION EN COURS")     
            self.get_logger().info(f"pos_aquabot : {self.pos_aquabot}")
            self.get_logger().info(f"distance entre aquabot et qr_code : {self.dist(self.pos_aquabot,Point_objectif_bateau)}")

        elif self.current_task >= 2 and self.critical_wind_turbine_coordinates_calculated: # On doit se diriger vers l'éolienne en détresse et si on est proche activer la phase de stabilisation et publier le point a atteindre
            Point_objectif_bateau = self.coord_critique_qr_code()
            if self.dist(Point_objectif_bateau,self.pos_aquabot) < self.tolerance_target_dist: # On est proche du point a atteindre et on a pas encore stabilisé.
                self.get_logger().info("ENCLENCHER STABILISATION ! ")
                self.PHASE_STABILISATION_ENCLENCHEE = True
                msg = String()
                msg.data = "OMG J'AI ATTEINT UNE SUPERBE EOLIENNE ! Elle est dans un état critique, il faut la réparer !"
                self.chat_pub.publish(msg)
                self.publier_coord_bateau(Point_objectif_bateau,"STABILISATION ENCLENCHEE A LINSTANT")  
            
            else: # On est pas assez proche et on continue de se diriger vers l'éolienne
                Point_objectif_bateau, indice_eolienne_si_prochain_point_est_eolienne = self.next_point()
                self.publier_coord_bateau(Point_objectif_bateau,"GO_TO_STABILISATION")
                
        else: # On est pas a la phase 2 ou + donc on s'en fiche
            if self.eoliennes_data_initialisee:
                Point_objectif_bateau, indice_eolienne_si_prochain_point_est_eolienne = self.next_point()
            else:
                # self.get_logger().info("Données par encore initialisée")
                self.get_logger().info("Données eoliennes non-initialisée")
                Point_objectif_bateau = (0,0)

            self.publier_coord_bateau(Point_objectif_bateau,"NORMAL PUB")
        

    def ais_callback(self, msg):
        """Met a jour les coordonnées des éoliennes\n
        La première fois que cela publie quelque chose de non vide
        """

        n_eolienne = len(msg.poses)
        eolienne_coord = []
        for i in range(n_eolienne):
            eolienne_1_latitude = msg.poses[i].position.x
            eolienne_1_longitude = msg.poses[i].position.y
            eolienne_coordinates = self.coordinates_from_point(eolienne_1_latitude, eolienne_1_longitude, self.origine_latitude, self.origine_longitude)
            eolienne_coord.append(eolienne_coordinates)

        self.wind_turbines_coordinates = [c for c in eolienne_coord] # position des éoliennes


        if not self.eoliennes_data_initialisee: # On suppose que les coordonnées des éoliennes ne vont pas changer
            self.coordonnees_eoliennes = self.wind_turbines_coordinates.copy() # On met a jour les coordonnées des éoliennes
            self.n_eolienne = len(self.coordonnees_eoliennes)
            self.l_graphe_eolienne = copy.deepcopy(self.genere_graphe_eolienne(self.coordonnees_eoliennes,False)) # On met a jour les graphes des éoliennes
            self.l_graphe_eolienne_critique = copy.deepcopy(self.genere_graphe_eolienne(self.coordonnees_eoliennes,True)) # On met a jour les graphes des éoliennes critique (les distance sont a self.eolienne__critique)
            self.ordre_visite = self.calcul_ordre_parcours_eolienne()
            self.eolienne_vu = [False] * self.n_eolienne
            self.eolienne_scanne = [False] * self.n_eolienne
            self.positions_eolienne_scanne = []
            for _ in range(self.n_eolienne):
                self.positions_eolienne_scanne.append([])
            self.eoliennes_data_initialisee = True
            self.get_logger().info("POS_EOLIENNE_INITIALISEE : ")
            self.get_logger().info(f"graphe eolienne : {self.l_graphe_eolienne}")
            self.get_logger().info(f"graphe eolienne CRITIQUE : {self.l_graphe_eolienne_critique}")
            self.get_logger().info(f" Nombre d'éolienne : {self.n_eolienne}")
    
    def gps_callback(self, msg):
        """Met a jour la position GPS du bateau"""
        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        self.aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude) # position de l'aquabot

        self.pos_aquabot = self.aquabot_coordinate # On met a jour la position de l'aquabot
        
        # On vérifie si il a atteint le point suivant si il en a a atteindre : 
        if len(self.l_chemin_en_cours_autour_eolienne)>0:
            i_pos_autour_eolienne,i_eolienne = self.l_chemin_en_cours_autour_eolienne[0]


            if self.current_task < 2:
                position_cible = self.l_graphe_eolienne[i_eolienne][0][i_pos_autour_eolienne]
            else:
                position_cible = self.l_graphe_eolienne_critique[i_eolienne][0][i_pos_autour_eolienne]
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
        self.positions_eolienne_scanne[self.id_eolienne_checked].append(self.pos_aquabot)
        # self.get_logger().info(f"Data qr code received : id:{self.id_eolienne_checked} state_eolienne : {self.state_eolienne_checked}")

    def current_phase_callback(self, msg):
        """
        Get the current task number
        """
        self.current_task = msg.data
        #if self.current_task == 3:
        #    self.tolerance_target_dist = 5
        # self.get_logger().info(f'current_task: {self.current_task}')

    def critical_wind_turbine_callback(self, msg):
        self.critical_windturbine_coordinates = (msg.x,msg.y)
        self.initialise_phase_2()
    
    # FONCTIONS DE CALCUL DU CHEMIN.

    def initialise_phase_2(self):
        """Initialise toute les variable relative au début de la phase 2."""
        windturbine_target = self.critical_windturbine_coordinates # Coordonnées calculée approximativement de la position de l'éolienne critique.
        wind_turbine_target_idx = self.indice_plus_proche(self.coordonnees_eoliennes, windturbine_target) # On devine l'indice de l'éolienne en état critique ne prenant l'éolienne la plus proche
        self.eolienne_vu[wind_turbine_target_idx] = False # On met l'éolienne comme n'étant pas vu, pour forcer le système à aller vers elle.
        self.critical_windturbine_idx = wind_turbine_target_idx # On met comme cible de destination l'éolienne en état critique
        self.critical_wind_turbine_coordinates_calculated = True # On met qu'on a calculé tout ce qu'il fallait pour que le système démarre la phase 2

    def moyenne_liste_coord(self,l_coord):
        n = len(l_coord)
        coord_somme = (0,0)
        for i in range(len(l_coord)):
            c = l_coord[i]
            coord_somme = self.add_vect(c,coord_somme)
        
        if n == 0:
            self.get_logger().error("MOYENNE : liste vide !")
            return (0,0)
        else:
            return self.mul_vect(coord_somme,1/n)

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
            # self.get_logger().info("Graphe chargé avec succès.")
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
    
    def coord_autour(self,coord, est_critique):
        """
        Renvoie les coordonnées autour d'un point (utilisé lors de la création des graphes des éoliennes)
        """
        l_off = []
        if est_critique:
            d = self.distance_point_passage_eolienne_CRITIQUE
        else:
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

    def spawn_eolienne(self,G,pos_eolienne, est_critique):
        """
        Ajoute l'éolienne a la position pos_eolienne au graphe G par effet de bord
        """
        pos_eolienne = (pos_eolienne[0], pos_eolienne[1])
        V = 0
        E = 1
        
        #On génère les coordonnées autour de l'éolienne.
        c_off_eolienne = self.coord_autour(pos_eolienne, est_critique)

        for i in range(len(c_off_eolienne)):
            coord_decalee = c_off_eolienne[i]
            # On regarde la distance entre la coordonnee et le rocher le plus proche du point
            # Si le point est plus proche du rocher que de leolienne On fait la moyenne entre la pos du point et celle de leolienne et du point precedent avec bcp de point sur leolienne et le rocher
            i_rocher = self.indice_plus_proche(self.G[0],coord_decalee)
            p_rocher_plus_proche = self.G[0][i_rocher]
            if self.dist(p_rocher_plus_proche,coord_decalee) < self.distance_point_passage_eolienne:
                coord_decalee = self.add_vect(self.add_vect(self.mul_vect(pos_eolienne, 1 ), self.mul_vect(p_rocher_plus_proche, 2 )), coord_decalee)
                coord_decalee = self.mul_vect(coord_decalee, 1/4)

            # On ajoute les coordonnées des points au graphe
            G[V].append(coord_decalee)
            G[E].append([])

        for i in range(len(c_off_eolienne)):
            coord_decalee = c_off_eolienne[i]
            #On trie les points autour en fonction de la distance au point qu'on regarde actuellement
            d_au_eolienne = [(self.dist(coord_decalee,c_off_eolienne[k]),k) for k in range(len(c_off_eolienne))]
            d_au_eolienne.sort()
            #Le point autour del 'éolienne est reliée aux deux points les plus proches d'eux.
            G[E][i].append(d_au_eolienne[1][1])
            G[E][i].append(d_au_eolienne[2][1])

    def genere_graphe_eolienne(self,l_eolienne,est_critique):
        """
        Renvoie une liste qui contient a chaque indice i, le graphe généré par la position de l'éolienne dans l_eolienne a l'indice i
        """
        l = []
        for i in range(len(l_eolienne)):
            graphe = [[],[]]
            self.spawn_eolienne(graphe,l_eolienne[i],est_critique)
            l.append(graphe)
        return l

    def calcul_cout_ordre(self,liste_ordre):
        p0 = self.pos_aquabot
        p1 = self.coordonnees_eoliennes[liste_ordre[0]]
        d = 0
        for i in range(self.n_eolienne):
            d += self.dist(p0,p1)
            p0 = p1
            if i != self.n_eolienne - 1:
                p1 = self.coordonnees_eoliennes[liste_ordre[i+1]]
            else:
                p1 = p0

        return d

    def calcul_ordre_parcours_eolienne(self):
        """
        Renvoie la liste des indice dans lesquel parcourir les éoliennes ainsi que la distance total (distance,ordre)\n
        En utilisant un algorithme de selection génétique.
        """

        meilleur_ordre, d = tp.run_combat_genetic(self.coordonnees_eoliennes)

        return meilleur_ordre
    
    def point_atteint(self,i_eolienne):
        """
        Quand on a atteint le point précédent donné par la fonction, appeler cette fonction marque le point comme étant atteint et met a jour les variables
        """
        if len(self.l_chemin_en_cours_autour_eolienne)>0:
            i_point,i_eolien = self.l_chemin_en_cours_autour_eolienne.pop(0)
           
        if len(self.liste_chemin_en_cours) > 0:
            i_point,i_eolien = self.liste_chemin_en_cours.pop(0)
            # On a atteint un rocher, on l'ajoute dans les rochers déjà atteint
            self.rocher_deja_atteint.append(i_point)
        
        if i_eolienne != -1:
            # On marque l'éolienne atteinte comme exploré
            self.eolienne_vu[i_eolienne] = True
            # on remet a 0 les rochers atteint.
            self.rocher_deja_atteint.clear()

    def coord_point_passage_eolienne(self,i_eolienne,i_point):
        """Donne la coordonné du ième point de passage autour d'une éolienne"""
        if self.current_task < 2:
            return self.l_graphe_eolienne[i_eolienne][0][i_point]
        else:
            return self.l_graphe_eolienne_critique[i_eolienne][0][i_point]


    def next_point(self):
        """
        Renvoie la prochaine position à atteindre depuis la position self.pos_aquabot\n
        Prend en compte les éoliennes déjà atteinte
        """
        #Si on est proche de l'eolienne qu'on veut atteindre, on y clear le chemin en cours pour aller au else
        if self.dist(self.pos_aquabot,self.coordonnees_eoliennes[self.i_eolienne]) < 1.6*self.distance_point_passage_eolienne+self.tolerance_target_dist and len(self.l_chemin_en_cours_autour_eolienne) == 0:
            self.liste_chemin_en_cours.clear()

        # Si on est a la phase 3 et qu'on est proche de l'éolienne qu'on veut atteindre.
        if self.current_task == 3 and self.dist(self.aquabot_coordinate,self.coordonnees_eoliennes[self.i_eolienne]) < 1.6*self.distance_point_passage_eolienne+self.tolerance_target_dist:
            # On fait le tour de l'éolienne jusqu'à atteindre le point scanné.
            pass

        # Si on est en train de faire le tour d'une eolienne, on continue
        if len(self.l_chemin_en_cours_autour_eolienne) > 0:
            i_point, i_eolien = self.l_chemin_en_cours_autour_eolienne[0]
            return self.coord_point_passage_eolienne(i_eolien,i_point),i_eolien
        
        # Si on est en train de faire le tour d'un rocher, on continue
        elif len(self.liste_chemin_en_cours) != 0:
            i_point, i_eolien = self.liste_chemin_en_cours[0]
            return self.G[0][i_point],-1
        
        #Sinon on recalcule le chemin.
        else:
            next_pos,indice_eolienne_si_eolienne = self.prochain_points_etape1()
            return next_pos,indice_eolienne_si_eolienne

    def prochain_points_etape1(self):
        """
        Détermine le prochain point a atteindre en fonction de la progression,\n
        Crée un chemin si des rochers ou des éoliennes sont a proximité\n
        Met a jour les variables de next_point pour qu'un tel chemin soit retenu\n
        Renvoie la prochaine position et l'indice de l'éolienne rencontrée (si le prochain point a atteindre est une éolienne)
        """

        i_actuel = 0

        while i_actuel < self.n_eolienne and self.eolienne_vu[self.ordre_visite[i_actuel]] and self.eolienne_scanne[self.ordre_visite[i_actuel]]:
            i_actuel += 1
    

        if self.current_task <= 1 and i_actuel < self.n_eolienne:
            i_eolienne = self.ordre_visite[i_actuel]
        else:
            # self.get_logger().info("TOUTE LES EOLIENNES ONT ETE VISITEE")
            if self.critical_wind_turbine_coordinates_calculated:
                i_eolienne = self.critical_windturbine_idx
            else:
                self.get_logger().error("PHASE 2 ENCLENCHEE ET EOLIENNE PAS CALCULEE")
                return (0,0),-1

        
        #Position de l'objectif d'apres
        if i_actuel < self.n_eolienne - 1:
            pos_suivante = self.coordonnees_eoliennes[self.ordre_visite[(i_actuel + 1)]]
        else:
            pos_suivante = (0,0)
        #self.i_eolienne est l'indice de l'éolienne que dont l'on cherche a faire le tour.
        self.i_eolienne = i_eolienne
        
        #C'est la position de l'éolienne que l'on cherche a atteindre
        eolinne_cible_pos = self.coordonnees_eoliennes[i_eolienne]

        # self.get_logger().info(f"Etat des éolienne vues : {self.eolienne_vu}")

        # Si la distance avec l'eolienne cible est faible -> on met en file d'attente les point qui passent autour de l'éolienne.
        # On pourrait ajouter d'autre point pour fiare en sorte que le bateau s'éloigne de l'éolienne au point le plus proche de la suivante.
        if self.dist(eolinne_cible_pos, self.pos_aquabot) < self.distance_point_passage_eolienne + self.tolerance_target_dist:
            self.rocher_deja_atteint.clear()
            self.liste_chemin_en_cours.clear()

            # self.get_logger().info("CONSTRUCTION DU CHEMIN AUTOUR DE LEOLIENNE EN EXPLORATION")
            
            if self.current_task < 2:
                graphe_de_eolienne_cible = self.l_graphe_eolienne[i_eolienne]
            else:
                graphe_de_eolienne_cible = self.l_graphe_eolienne_critique[i_eolienne]

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
            
            # self.get_logger().info(f"SUIT BORD GRAPHE : DEPART {idx_dernier_du_chemin}; NEXT {idx_proche_eolienne_suivante}; EVITE {chemin_autour_eolienne_sans_indice[-2]}")
            #Il ajoute rien haha hoho

            # self.get_logger().info(f"ajoute par suit_bord_graphe : {l}")
            # self.get_logger().info(f"CHEMIN AUTOUR EOLIENNE : {chemin_autour_eolienne_sans_indice}")
            for elt in l:
                chemin_autour_eolienne_sans_indice.append(elt)

            self.l_chemin_en_cours_autour_eolienne = [(indice,self.i_eolienne) for indice in (chemin_autour_eolienne_sans_indice)]
            # print(self.l_chemin_en_cours_autour_eolienne)
            if self.current_task < 2:
                graphe_de_eolienne_cible = self.l_graphe_eolienne[self.i_eolienne][0]
            else:
                graphe_de_eolienne_cible = self.l_graphe_eolienne_critique[self.i_eolienne][0]
            
            return graphe_de_eolienne_cible[chemin_autour_eolienne_sans_indice[0]],i_eolienne

        
        # Regarder la proximité des rochers autour du rayon

        V = self.G[0]
        n = len(V)
        caillou_a_portee = []
        for i in range(n):
            if self.dist(self.pos_aquabot, V[i]) < self.rayon_detection_rocher and i not in self.rocher_deja_atteint: #On ignore les rochers déjà atteint
                caillou_a_portee.append(i)

        eoliennes_a_portee = []
        for i in range(self.n_eolienne):
            if self.dist(self.pos_aquabot,self.coordonnees_eoliennes[i]) < 4*self.rayon_detection_rocher and i != i_eolienne:
                eoliennes_a_portee.append(i)
        
        vitesse_actuelle = self.rayon_detection_rocher

        # self.get_logger().info(f"Eolienne visée : ({self.coordonnees_eoliennes[i_eolienne][0]},{self.coordonnees_eoliennes[i_eolienne][1]})")
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

        # Regarder si le vecteur traverse la ligne des points autour d'une éolienne

        point_eolienne_coupe = []
        for i in range(len(eoliennes_a_portee)):
            i_eolienne_a_portee = eoliennes_a_portee[i]
            graphe_eolienne_a_portee = self.l_graphe_eolienne[i_eolienne_a_portee]
            for i in range(len(graphe_eolienne_a_portee[0])):
                pointA = i
                liste_voisin_point = graphe_eolienne_a_portee[1][pointA]
                for j in range(len(liste_voisin_point)):
                    voisin = liste_voisin_point[j]
                    segment = (graphe_eolienne_a_portee[0][pointA], graphe_eolienne_a_portee[0][voisin])
                    if self.se_croise(segment, s_direction):
                        p_voisin = graphe_eolienne_a_portee[0][voisin]
                        if self.dist(self.pos_aquabot,p_voisin) > self.tolerance_target_dist: #On ignore les points trop proche du bateau
                            point_eolienne_coupe.append(voisin)

        # Graphe de l'éolienne ciblée
        if self.current_task < 2:
            graphe_position_autour_eolienne_cible = self.l_graphe_eolienne[i_eolienne][0]
        else:
            graphe_position_autour_eolienne_cible = self.l_graphe_eolienne_critique[i_eolienne][0]
        
        #Chemin autour de l'éolienne
        chemin_autour_eolienne = copy.deepcopy(self.calcul_exploration_eolienne(i_eolienne,self.pos_aquabot,False))
        pos_du_premier_point = graphe_position_autour_eolienne_cible[chemin_autour_eolienne[2]]
            
        # Si aucun rocher ne s'interposent continuer en mode on s'en fiche, c'est à dire que il n'y a aucun rocher qui se coupent
        if len(points_qui_coupent) == 0 and len(point_eolienne_coupe) == 0:
            return pos_du_premier_point,i_eolienne

        #Evalue si il y a des éolienne avec lesquelle on peut rentrer
        if len(point_eolienne_coupe) > 0:
            self.get_logger().info("Collision avec eolienne possible")

            if len(points_qui_coupent)> 0:
                self.get_logger().info("Collision avec rocher possible")
                i_rocher_plus_proche_local = self.indice_plus_proche([self.G[0][i] for i in points_qui_coupent], self.pos_aquabot)
                i_rocher_plus_proche = points_qui_coupent[i_rocher_plus_proche_local]
                pos_rocher = self.G[0][i_rocher_plus_proche]
            
        
            indice_local_eolienne_plus_proche = self.indice_plus_proche([self.coordonnees_eoliennes[k] for k in eoliennes_a_portee], self.pos_aquabot)
            i_eolienne_plus_proche = eoliennes_a_portee[indice_local_eolienne_plus_proche]
            i_point_eolienne_plus_proche_local = self.indice_plus_proche([self.l_graphe_eolienne[i_eolienne_plus_proche][0][i] for i in point_eolienne_coupe], self.pos_aquabot)
            pos_point_passage_eolienne = self.l_graphe_eolienne[i_eolienne_plus_proche][0][i_point_eolienne_plus_proche_local]

            if len(points_qui_coupent) == 0 or (self.dist(pos_point_passage_eolienne,self.pos_aquabot) < self.dist(pos_rocher,self.pos_aquabot)):
                # C'est l'éolienne dont il faut s'occuper
                #On a une eolienne en plein milieu du chemin
                self.get_logger().info("Eolienne en plein milieu du chemin !!")
                # Trouve un chemin autour de l'éolienne pour l'éviter et aller vers le prochain point a atteindre
                i_min_eolienne_local = self.indice_plus_proche([self.l_graphe_eolienne[i_eolienne_plus_proche][0][i] for i in point_eolienne_coupe], self.pos_aquabot)
                i_plus_proche_bateau = point_eolienne_coupe[i_min_eolienne_local]
                i_max_eolienne_local = self.indice_plus_proche([self.l_graphe_eolienne[i_eolienne_plus_proche][0][i] for i in point_eolienne_coupe], self.coordonnees_eoliennes[i_eolienne_plus_proche])
                i_plus_proche_eolienne = point_eolienne_coupe[i_max_eolienne_local]

                chemin = []
                for i in range(8):
                    chemin.append((i_plus_proche_bateau + i) % 8)

                chemin = self.trouver_chemin(self.l_graphe_eolienne[i_eolienne_plus_proche], i_plus_proche_bateau, i_plus_proche_eolienne)
                
                chemin_opti = self.optimise_chemin(self.l_graphe_eolienne[i_eolienne_plus_proche],[i for i in range(8)], chemin, self.pos_aquabot, pos_du_premier_point)
                pos_opti = self.l_graphe_eolienne[i_eolienne_plus_proche][0][chemin_opti[0]]
                return pos_opti,i_eolienne_plus_proche

        # Il reste des rocher a casser
        
        # On trouve le point le plus proche des rochers trouvé
        coord_point_qui_coupent = [self.G[0][i] for i in points_qui_coupent]
        indice_rocher_plus_proche_bateau = points_qui_coupent[self.indice_plus_proche(coord_point_qui_coupent,self.pos_aquabot)]
        coord_rocher_plus_proche_bateau = self.G[0][indice_rocher_plus_proche_bateau]
        # self.get_logger().info(f"coord rocher plus proche bateau : {coord_rocher_plus_proche_bateau}")

        if len(coord_point_qui_coupent) > 0:
            self.get_logger().info("Collision avec rocher possible")

        indice_rocher_connecte_au_caillou = self.composante_connexe(self.G, indice_rocher_plus_proche_bateau)
        
        # self.get_logger().info(f"Rocher connecté au caillou : {indice_rocher_connecte_au_caillou}")
        
        coord_connecte_au_caillou = [self.G[0][i] for i in indice_rocher_connecte_au_caillou]
        # On met pos du premier poit a atteindre
        indice_plus_proche_eolienne = self.indice_plus_proche(coord_connecte_au_caillou,pos_du_premier_point)
        # Au lieu de regarder les plus prochzs, il faut rrgarder lesquelles on peut passser pour contourner les rochers

        chemin_dindices_a_parcourir = self.trouver_chemin(self.G, indice_rocher_plus_proche_bateau, indice_rocher_connecte_au_caillou[indice_plus_proche_eolienne])
        # Le chemin existe, car, indice_rocher_plus_proche_bateau et indice_plus_proche_eolienne sont dans une meme composante connexe de G


        #Chemin qui ne colle pas aux rochers
        chemin_optimise_dindices_a_parcourir = self.optimise_chemin(self.G, indice_rocher_connecte_au_caillou, chemin_dindices_a_parcourir, self.pos_aquabot, pos_du_premier_point)

        # self.get_logger().info(f"CHEMIN CHOISI : {chemin_dindices_a_parcourir}")
        # self.get_logger().info(f"CHEMIN OPTIMISE : {chemin_optimise_dindices_a_parcourir}")

        #Utilisation du chemin optimal 
        chemin_dindices_a_parcourir = chemin_optimise_dindices_a_parcourir

        for idx in chemin_dindices_a_parcourir:
            self.liste_chemin_en_cours.append((idx,-1))

        return coord_rocher_plus_proche_bateau,-1

    def optimise_chemin(self, graphe, composante_connexe, chemin_des_indices_parcouru_original,coord_depart, coord_arrive):
        """
        Optimise le chemin emprunté par le drone
        """

        chemin_des_indices_parcouru = copy.deepcopy(chemin_des_indices_parcouru_original)
        
        # On parcours les indices parcouru
        indice_le_plus_loin = 0
        for i in range(len(chemin_des_indices_parcouru)):
            idx_rocher = chemin_des_indices_parcouru[i]
            p_rocher = graphe[0][idx_rocher]
            self.se_croise
            segment_bateau_rocher = (coord_depart, self.add_vect(coord_depart, self.add_vect(self.mul_vect(coord_depart,-1),p_rocher)))

            # On regarde si on peut pas aller de l'aquabot a un des points du chemin, sans passer au travers du rocher
            indice_coupable = True
            for k in range(len(composante_connexe)):
                caillou = composante_connexe[k]
                v_caillou = graphe[1][caillou]
                for j in range(len(v_caillou)):
                    voisin = v_caillou[j]
                    if caillou != idx_rocher and voisin != idx_rocher:
                        
                        segment = (graphe[0][caillou], graphe[0][voisin])

                        if self.se_croise(segment, segment_bateau_rocher):
                            indice_coupable = False

            # Si on peut, on raccourcis le chemin en enlevant les noeux avant
            if indice_coupable:
                indice_le_plus_loin = i
        
        for k in range(indice_le_plus_loin):
            id_pop = chemin_des_indices_parcouru.pop(0)
            self.rocher_deja_atteint.append(id_pop)

        # On fait la meme mais depuis l'éolienne.

        # On parcours les indices parcouru
        indice_le_plus_loin = 0
        for i in range(len(chemin_des_indices_parcouru)):
            idx_rocher = chemin_des_indices_parcouru[i]
            p_rocher = graphe[0][idx_rocher]
            self.se_croise
            segment_bateau_rocher = (coord_arrive, self.add_vect(coord_arrive, self.add_vect(self.mul_vect(coord_arrive,-1),p_rocher)))

            # On regarde si on peut pas aller de l'aquabot a un des points du chemin, sans passer au travers du rocher
            indice_coupable = True
            for k in range(len(composante_connexe)):
                caillou = composante_connexe[k]
                v_caillou = graphe[1][caillou]
                for j in range(len(v_caillou)):
                    voisin = v_caillou[j]
                    if caillou != idx_rocher and voisin != idx_rocher:
                        
                        segment = (graphe[0][caillou], graphe[0][voisin])

                        if self.se_croise(segment, segment_bateau_rocher):
                            indice_coupable = False

            # Si on peut, on raccourcis le chemin en enlevant les noeux avant
            if indice_coupable:
                indice_le_plus_loin = i
                break
        
        n = len(chemin_des_indices_parcouru)
        for k in range(indice_le_plus_loin + 1,n):
            id_pop = chemin_des_indices_parcouru.pop(indice_le_plus_loin + 1)
            self.rocher_deja_atteint.append(id_pop)

        return chemin_des_indices_parcouru


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
        if self.current_task < 2:
            g_eolienne = self.l_graphe_eolienne[i_eolienne]
        else:
            g_eolienne = self.l_graphe_eolienne_critique[i_eolienne]
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

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(wte_navigation_node)

    try:
        executor.spin()
    finally:
        wte_navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()