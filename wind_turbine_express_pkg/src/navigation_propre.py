#!/usr/bin/env python3
import random
import math
import cv2
import rclpy
import numpy as np
import os
import copy
from rclpy.node import Node
from wind_turbine_express_interfaces.msg import Thruster
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory

class WTENavigationNode(Node):

    def __init__(self):
        #TEMPLATE
        super().__init__('wte_navigation_node_py')

        self.navigation = self.create_publisher(Thruster, '/aquabot/navigation/point', 10)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.nav_point_callback)
        
        self.aquabot_coordinate = []
        self.wind_turbines_coordinates = []
        
        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632

        # Declare les variables ici
        #END TEMPLATE
        self.get_logger().info("Begin initialisation")

        # Précharger carte des distance euclidiennes : 

        nom_package = "wind_turbine_express_pkg"
        chemin_relatif_image_dist = "images/dist_transform.jpg"
        chemin_complet_image_dist = os.path.join(get_package_share_directory(nom_package), chemin_relatif_image_dist)

        carte_pixel = self.precharger_carte_distance(chemin_complet_image_dist)
        if carte_pixel is not None:
            self.taille_carte_dist_transform = carte_pixel.shape
        else:
            self.get_logger().error("ECHEC CHARGEMENT CARTE DISTANCE !")


        chemin_relatif_graph_rocher = "assets/graph_rocher.txt"
        chemin_complet_graph_rocher = os.path.join(get_package_share_directory(nom_package), chemin_relatif_graph_rocher)

        #Charger le graphe des rochers 
        self.G = self.recup_graphe(chemin_complet_graph_rocher)


        #PARAMETRE NAVIGATION
        coordonees_eoliennes = [(-137.07841337181242, 145.127446451792), (-105.59533937321473, -41.79734152154377),
                    (139.4011694854088, 85.02789988405654)]

        coordonees_eoliennes = [(-190.07841337181242, 0.127446451792), (-105.59533937321473, -41.79734152154377),
                    (139.4011694854088, 85.02789988405654)]

        self.coordonnees_eoliennes = [] #Mise a jour dans ais callback

        self.pos = (0,0) # Mis a jour dans gps_callback

        self.l_graphe_eolienne = [[],[]] #Mis a jour lors de ais callback

        d, self.ordre_visite = self.calcul_ordre_parcours_eolienne(coordonees_eoliennes)
        # print(ordre_visite)
        self.eolienne_visite = [False, False, False]

        # liste du chemin en cours
        self.liste_chemin_en_cours = []

        #liste du chemin des éolienne parcouru en cours
        self.liste_eolienne_chemin = []

        #Liste des rochers deja atteint
        self.rocher_deja_atteint = []
        self.p_eolien_deja_atteint = []

        #Behavior ! (target nearby windturbine or go to the next one !)
        self.IDLE = 0 # never used
        self.ROUND_AROUND_WINDTURBINE = 1
        self.GO_NEXT_WINDTURBINE = 2
        self.PHASE_ONE_INITIALISATION = -1
        self.PHASE_ONE_COMPLETE = -2

        self.behavior = self.PHASE_ONE_INITIALISATION

        #Eolienne tournage autour
        self.l_sommet = []
        self.g_eolienne = []
        self.qr_code_scaned = False
        self.qr_code_pos = 5
        self.i_objectif = 0
        self.i_proche_next = -1
        self.i_eolienne = -1
        self.next_indice_around_windturbine = -1
        self.plus_proche_next_atteint = False
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
        self.get_logger().info("NAV CALLBAKCK CALLED")

        next_pos = self.get_next_move()
        self.get_logger().info("next_pos : ")
        self.get_logger().info(f"x : {next_pos[0]} y : {next_pos[1]}")


        prochaine_coord_bateau = next_pos

        Point_objectif_bateau = [prochaine_coord_bateau[0],prochaine_coord_bateau[1]] #Doit pouvoir recup le next point jsp oû
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
        if len(self.coordonnees_eoliennes) == 0: # On suppose que les coordonnées des éoliennes ne vont pas changer
            self.coordonnees_eoliennes = self.wind_turbines_coordinates.copy() # On met a jour les coordonnées des éoliennes
            self.l_graphe_eolienne = copy.deepcopy(self.genere_graphe_eolienne(self.coordonnees_eoliennes)) # On met a jour les graphes des éoliennes
            self.behavior = self.GO_NEXT_WINDTURBINE
    def gps_callback(self, msg):
        """Met a jour la position GPS du bateau"""
        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        self.aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude) # position de l'aquabot

        self.pos = self.aquabot_coordinate # On met a jour la position de l'aquabot

    #Mets tes fonctions ici (dans la classe WTENavigationNode) en enlevant tout le graphique

    #END FUNCTION TEMPLATE

    #Mes fonctions sans le graphique
    
    def get_next_move(self):
        
        self.get_logger().info("Behavior : " + str(self.behavior))
        self.update_behavior()
        if self.behavior == self.GO_NEXT_WINDTURBINE:
            # ON VA VERS LA PROCHAINE EOLIENNE
            if self.i_eolienne != -1:
                next_pos = self.coordonnees_eoliennes[self.i_eolienne]
            elif len(self.liste_chemin_en_cours) > 0:
                next_pos = self.liste_chemin_en_cours[0]
            else:
                if self.i_eolienne >= 0 and self.i_eolienne < len(self.coordonnees_eoliennes):
                    next_pos = self.coordonnees_eoliennes[self.ordre_visite[self.i_eolienne]]
                else:
                    next_pos = (self.origine_longitude,self.origine_latitude)

            if self.i_eolienne != -1 and self.dist(self.pos,self.coordonnees_eoliennes[self.i_eolienne]) < 15:
                #EOLIENNE ATTEINTE ! 
                
                self.behavior = self.ROUND_AROUND_WINDTURBINE
                #INITIALISATION PARCOURS EOLIENNE

                self.next_indice_around_windturbine = (self.i_eolienne + 1) if self.i_eolienne < 2 else -1

                self.l_sommet = self.calcul_exploration_eolienne(self.i_eolienne,self.pos)
                self.g_eolienne = self.l_graphe_eolienne[self.i_eolienne]
                self.qr_code_scaned = False 
                self.qr_code_pos = 5 # ON NE CONNAIT PAS LA DONNEE, A OBTENIR A LA VOLEE
                self.i_objectif = 0
                self.i_proche_next = -1

                if self.next_indice_around_windturbine != -1:
                    self.indice_point_eolienne_next = self.indice_plus_proche(self.l_graphe_eolienne[self.next_indice_around_windturbine][0], self.pos)
                    self.i_proche_next = self.indice_plus_proche(self.g_eolienne[0],self.l_graphe_eolienne[self.next_indice_around_windturbine][0][self.indice_point_eolienne_next])


                #APPEL RECURSIF DE NEXT_MOVE
                return self.get_next_move()
            

            elif self.dist(self.pos,next_pos) < 5:
                # On a atteint le prochain poin de passage 
                self.point_atteint(self.pos)
                return self.get_next_move()
            
            else:
                # On se dirige vers l'éolienne
                v_pn = self.add_vect(next_pos, self.mul_vect(self.pos, -1))
                n_v = self.norme_vecteur(v_pn) #Norme vecteur >=1
                v_pn = self.mul_vect(v_pn, 1 / n_v)
                v_pn = self.mul_vect(v_pn,10)
                next_pos = self.add_vect(self.pos,v_pn)

                if self.dist(next_pos,self.pos) < 10: # N'arrive jamais ?
                    return next_pos
                else:
                    return next_pos
                    #return self.add_vect(self.pos,v_pn)

        elif self.behavior == self.ROUND_AROUND_WINDTURBINE:
            
            
            # ON EST PROCHE DE LEOLIENNE, ON TOURNE AUTOUR ET ON SEN VA APRES
            if not self.qr_code_scaned or not self.plus_proche_next_atteint:
                self.behavior = self.GO_NEXT_WINDTURBINE
                return self.get_next_move()

            p_objectif = self.g_eolienne[0][self.l_sommet[self.i_objectif]]
            if self.dist(self.pos,p_objectif) < 1:
                #Point atteint !
                
                
                #Si on est sur le point le plus proche de l'éolienne suivante
                if self.i_objectif == self.i_proche_next or self.i_proche_next == -1:
                    self.plus_proche_next_atteint = True
                else:
                    self.plus_proche_next_atteint = False

                #Si on a scanné le QR code
                if self.i_objectif == self.qr_code_pos:
                    #print("QR CODE SCANNED")
                    self.qr_code_scaned = True
                    if self.i_proche_next == -1:
                        self.plus_proche_next_atteint = True

                #On va au point suivant
                self.i_objectif = (self.i_objectif + 1) % len(self.l_sommet)
                return p_objectif
        return -1,-1

        


    #FONCTION DE GESTION DU BEHAVIOR(comportement)

    def update_behavior(self):
        if self.behavior == self.PHASE_ONE_INITIALISATION:
            self.update_behavior_phase_one_initialisation()
        elif self.behavior == self.GO_NEXT_WINDTURBINE:
            self.update_behavior_go_next_windturbine()
        elif self.behavior == self.ROUND_AROUND_WINDTURBINE:
            self.update_behavior_round_around()
        elif self.behavior == self.PHASE_ONE_COMPLETE:
            self.update_behavior_phase_one_completed()
        else:
            pass
    
    def update_behavior_phase_one_initialisation(self):
        if len(self.g_eolienne) > 0 and len(self.G[0])>0:
            self.behavior = self.GO_NEXT_WINDTURBINE

    def update_behavior_go_next_windturbine(self):
        pass


    def precharger_carte_distance(self,nom_fichier):
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
        x_carte = self.taille_carte_dist_transform[0] * (self.pos[0] + 300) / 600
        y_carte = self.taille_carte_dist_transform[1] * (self.pos[1] + 300) / 600
        x, y = math.floor(x_carte), math.floor(y_carte)
        dec_x = x_carte - x
        dec_y = y_carte - y

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
    
    def map_to_world_coord(self, map_size, world_size, map_coord):
        """
        Converti la liste de coordonnées de la taille de la map au monde
        """
        world_coord = []
        for c in map_coord:
            x, y = c[0], c[1]
            world_coord.append(((x / map_size[0]) * world_size[0] - 300, -((y / map_size[1]) * world_size[1] - 300)))
        return world_coord

    def word_to_map_coord(self, map_size, world_size, map_coord):
        """
        Converti la liste de coordonnées de la taille du monde a la map
        """
        world_coord = []
        for c in map_coord:
            x, y = c[0], c[1]
            world_coord.append((math.floor(((x + 300) / (world_size[0])) * map_size[0]),
                                math.floor(((-y + 300) / (world_size[1])) * map_size[1])))
        return world_coord

    def norme_vecteur(self,x):
        """
        Norme du vecteur x de dimension 2
        """
        return math.sqrt(x[0] ** 2 + x[1] ** 2)

    def dist(self,a, b):
        """
        Distance euclidienne entre 2 coordonnées
        """
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
        while len(file) != 0:
            s = file.pop(0)
            if s == arrivee:
                break
            for i in range(len(G[1][s])):
                v = G[1][s][i]
                if v not in d:
                    d[v] = s
                file.append(v)

        c = []
        s_actuel = arrivee
        while d[s_actuel] != s_actuel:
            c.append(s_actuel)
            s_actuel = d[s_actuel]
        c.append(depart)
        c.reverse()
        return c
    
    def chemin_a_travers_graphe(self,G,pos_depart,pos_arrivee):
        """
        Prend en argumen 2 position dans les coordonnées du monde 
        Renvoie la suite de sommet qui passe a travers le graphe en minimisant le nombre de sommet parcouru
        Le premier sommet du chemin renvoyé est le plus proche de pos_depart et le dernier sommet du chemin renvoyé est le plus proche de pos_arrivee
        """
        i_depart = -1
        d_depart = 999999
        i_arrivee = -1
        d_arrivee = 99999

        n = len(G[0])
        for i in range(n):
            if self.dist(pos_depart, G[0][i]) < d_depart:
                d_depart = self.dist(pos_depart, G[0][i])
                i_depart = i

            if self.dist(pos_arrivee, G[0][i]) < d_arrivee:
                d_arrivee = self.dist(pos_arrivee, G[0][i])
                i_arrivee = i

        if i_depart == i_arrivee:
            return []
        else:
            return self.trouver_chemin(G,i_depart,i_arrivee)

    def coord_autour(self,coord):
        """
        Renvoie les coordonnées autour d'un point (utilisé lors de la création des graphes des éoliennes)
        """
        l_off = []
        d = 10
        l_off.append((coord[0] + d,coord[1]))
        l_off.append((coord[0] ,coord[1] + d))
        l_off.append((coord[0], coord[1] - d))
        l_off.append((coord[0] - d, coord[1]))
        l_off.append((coord[0] + d * math.sqrt(2) / 2, coord[1] + d * math.sqrt(2) / 2))
        l_off.append((coord[0] + d * math.sqrt(2) / 2, coord[1] - d * math.sqrt(2) / 2))
        l_off.append((coord[0] - d * math.sqrt(2) / 2, coord[1] + d * math.sqrt(2) / 2))
        l_off.append((coord[0] - d * math.sqrt(2) / 2, coord[1] - d * math.sqrt(2) / 2))
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

    def calcul_ordre_parcours_eolienne(self,l_eolienne):
        """
        Renvoie la liste des indice dans lesquel parcourir les éoliennes ainsi que la distance total (distance,ordre)
        """
        return -1,(0,1,2)

    def point_atteint(self,p_atteint):
        """
        Quand on a atteint le point précédent donné par la fonction, appeler cette fonction marque le point comme étant atteint et met a jour les variables
        """
        if len(self.liste_eolienne_chemin)>0:
            i_point,i_eolien = self.liste_eolienne_chemin.pop(0)
            p_atteint = self.l_graphe_eolienne[i_eolien][0][i_point]
            self.p_eolien_deja_atteint.append(p_atteint)
        elif len(self.liste_chemin_en_cours) > 0:
            id_rocher = self.liste_chemin_en_cours.pop(0)
            p_atteint = self.G[0][id_rocher]
            # On a atteint un rocher, on l'ajoute dans les rochers déjà atteint
            self.rocher_deja_atteint.append(id_rocher)
        for i_e in range(len(self.coordonnees_eoliennes)):
            p_e = self.coordonnees_eoliennes[i_e]
            if self.dist(p_atteint,p_e) < 0.001:
                #print("EOLIENNE ATTEINTE")
                self.eolienne_vu[i_e] = True
                # on remet a 0 les rochers atteint.
                self.rocher_deja_atteint.clear()

    def next_point(self):
        """
        Renvoie la prochaine position à atteindre depuis la position self.pos
        Prend en compte les éoliennes déjà atteinte
        Renvoie (-9999999, -999999) quand toutes les éoliennes ont été atteinte
        """
        if len(self.liste_eolienne_chemin) >0:
            i_point, i_eolien = self.liste_eolienne_chemin[0]
            return self.l_graphe_eolienne[i_eolien][0][i_point],i_eolien
        if len(self.liste_chemin_en_cours) != 0:
            return self.G[0][self.liste_chemin_en_cours[0]],-1
        else:
            # print("chemin en cours vide, recherche d'un nouveau point ")
            next_pos,i_eolienne = self.prochain_points_etape1(self.coordonnees_eoliennes, self.eolienne_vu, self.G, (-9999999, -999999))
            return next_pos,i_eolienne

    def prochain_points_etape1(self,l_eolienne, eolienne_vu, G_caillou, vect_vitesse):
        """
        Détermine le prochain point a atteindre en fonction de la progression,
        Crée un chemin si des rochers ou des éoliennes sont a proximité
        Met a jour les variables de next_point pour qu'un tel chemin soit retenu
        Renvoie la prochaine position et l'indice de l'éolienne rencontrée (si le prochain point a atteindre est une éolienne)
        """

        i_actuel = 0
        while i_actuel < len(eolienne_vu) and eolienne_vu[self.ordre_visite[i_actuel]]:
            i_actuel += 1

        if i_actuel == len(eolienne_vu):
            print("FINI")
            return (999, 999),-1

        i_eolienne = self.ordre_visite[i_actuel]

        # print(l_eolienne)
        vect_eolienne = [l_eolienne[i_eolienne][0] - self.pos[0], l_eolienne[i_eolienne][1] - self.pos[1]]
        # print(vect_eolienne)

        eolinne_cible_pos = l_eolienne[i_eolienne]

        n_vect = self.norme_vecteur(vect_eolienne)
        vect_eolienne[0] /= n_vect
        vect_eolienne[1] /= n_vect
        # Regarder la vitesse desire en fonctio nde la carte des distances

        d_caillou = self.get_distance_caillou(self.pos)

        # trouver un rayon de surveillance das lequel on veut attraper tout les rochers
        r = 0
        if d_caillou < 125:
            r = 30
        # print(r)

        # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [self.pos])[0], (255, 255, 0, 255))

        # Regarder la proximité des rochers autour du rayon

        V = self.G[0]
        n = len(V)
        caillou_a_portee = []
        for i in range(n):
            if self.dist(self.pos, V[i]) < r and i not in self.rocher_deja_atteint: #On ignore les rochers déjà atteint
                caillou_a_portee.append(i)
                # print(V[i])
                # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [V[i]])[0], (255, 0, 0, 255))

        #On regarde aussi les éoliennes qu'on ne veux pas atteindre a proximité
        eoliennes_a_portee = []
        for i in range(len(l_eolienne)):
            if self.dist(self.pos,l_eolienne[i]) < r and i != i_eolienne:
                eoliennes_a_portee.append(i)



        RANGE = 30
        vitesse_actuelle = RANGE * (255 - self.get_distance_caillou(self.pos)) / 255

        # print(vitesse_actuelle)

        vect_eolienne = self.mul_vect(vect_eolienne, vitesse_actuelle)

        # draw_line(self.pos, add_vect(vect_eolienne, self.pos), (255, 0, 255, 255))

        # Regarder si le vecteur "traverse la ligne des points" ou "va vers l'intérieur des rochers"

        points_qui_coupent = []
        s_direction = (self.pos, self.add_vect(self.pos, vect_eolienne))
        for i in range(len(caillou_a_portee)):
            caillou = caillou_a_portee[i]
            v_caillou = self.G[1][caillou]
            for j in range(len(v_caillou)):
                voisin = v_caillou[j]
                segment = (self.G[0][caillou], self.G[0][voisin])
                if self.se_croise(segment, s_direction):
                    if voisin not in self.rocher_deja_atteint:
                        points_qui_coupent.append(voisin)

        # Afficher les points qui on un sgment qui fait que le cecteur du bateau se coupe avec
        # for p in points_qui_coupent:
        #    point = G[0][p]
        # print(point)
        # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [point])[0], (255, 255,255, 255))

        # Si non continuer en mode on s'en fiche
        if len(points_qui_coupent) == 0:
            return eolinne_cible_pos,i_eolienne

        # On trouve le point le plus proche des rochers trouvé
        plus_proche = -1
        d_proche = 99999999
        for i in range(len(points_qui_coupent)):
            p_rocher = self.G[0][points_qui_coupent[i]]
            if d_proche > self.dist(self.pos, p_rocher):
                d_proche = self.dist(self.pos, p_rocher)
                plus_proche = points_qui_coupent[i]

        # Point du graphe le plus proche du bateau
        plus_proche_bateau = plus_proche
        # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][plus_proche]])[0], (125, 125, 125, 255))

        # print("FIN TESt")

        # On dis qu'on y va et on set_up une liste de point a parcourir ?

        point_connecte_au_caillou = self.composante_connexe(self.G, plus_proche_bateau)
        plus_proche = -1
        d_proche = 99999999
        for i in range(len(point_connecte_au_caillou)):
            p_rocher = self.G[0][point_connecte_au_caillou[i]]
            if d_proche > self.dist(eolinne_cible_pos, p_rocher):
                d_proche = self.dist(eolinne_cible_pos, p_rocher)
                plus_proche = point_connecte_au_caillou[i]

        plus_proche_eolienne = plus_proche

        c = self.trouver_chemin(self.G, plus_proche_bateau, plus_proche_eolienne)


        for p in c:
            self.liste_chemin_en_cours.append(p)
            # AFficher le chemin
        #        image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][p]])[0],(255, 255, 255, 255))

        return self.G[0][c[0]],-1
        # Si oui calculer un point plus proche en fonction de l'interieur des rochers/
        # pour que le prochain pour soir un peu en dehors du rochers et que ce soit pas sur le graphe sinon cest la cata

        # image_originale.show()

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
    
    def calcul_exploration_eolienne(self, i_eolienne,pos_bateau):
        g_eolienne = self.l_graphe_eolienne[i_eolienne]
        point_proche = self.indice_plus_proche(g_eolienne[0],pos_bateau)
        l_sommet = self.composante_connexe(g_eolienne,point_proche)
        return l_sommet


def main(args=None):
    rclpy.init(args=args)
    wte_navigation_node = WTENavigationNode()
    rclpy.spin(wte_navigation_node)
    wte_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
