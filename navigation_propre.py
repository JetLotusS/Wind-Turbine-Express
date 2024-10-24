import math
import cv2


def precharger_carte_distance(nom_fichier):
    image = cv2.imread(nom_fichier)
    if image is None:
        print("IMAGE DES DISTANCE PAS PUS CAHRGER AUSCOUR")
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


carte_pixel = precharger_carte_distance("dist_transform.jpg")
taille = carte_pixel.shape


def mul_vect(v, lambd):
    return (v[0] * lambd, v[1] * lambd)


def add_vect(v, v2):
    return (v[0] + v2[0], v[1] + v2[1])


def get_distance_caillou(pos):
    x_carte = taille[0] * (pos[0] + 300) / 600
    y_carte = taille[1] * (pos[1] + 300) / 600
    x, y = math.floor(x_carte), math.floor(y_carte)
    dec_x = x_carte - x
    dec_y = y_carte - y

    px_entier = carte_pixel[x, y]
    px_droit = carte_pixel[x + 1, y]
    px_haut = carte_pixel[x, y + 1]
    px_coin = carte_pixel[x + 1, y + 1]

    px_e_partie = px_entier * (1 - dec_x) * (1 - dec_y)
    px_d_partie = px_droit * dec_x * (1 - dec_y)
    px_h_partie = px_haut * (1 - dec_x) * dec_y
    px_c_partie = px_coin * dec_x * dec_y

    total = px_e_partie + px_d_partie + px_c_partie + px_h_partie
    moyenne = total

    return moyenne


def get_vitesse(pos):
    pass


def recup_graphe(nom_f):
    G = [[], []]

    f = open(nom_f, "r")
    texte = f.read()
    f.close()

    # print(texte)

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


G = recup_graphe("graph_rocher.txt")


def map_to_world_coord(map_size, world_size, map_coord):
    world_coord = []
    for c in map_coord:
        x, y = c[0], c[1]
        world_coord.append(((x / map_size[0]) * world_size[0] - 300, -((y / map_size[1]) * world_size[1] - 300)))
    return world_coord


def word_to_map_coord(map_size, world_size, map_coord):
    world_coord = []
    for c in map_coord:
        x, y = c[0], c[1]
        world_coord.append((math.floor(((x + 300) / (world_size[0])) * map_size[0]),
                            math.floor(((-y + 300) / (world_size[1])) * map_size[1])))
    return world_coord


"""
def afficher_graph(G, image):
    # Créer un objet ImageDraw
    draw = ImageDraw.Draw(image)
    print("V : ")
    print(G[0])
    print("E : ")
    print(G[1])
    for i in range(len(G[0])):
        for j in range(len(G[1][i])):
            c1m = G[0][i]
            #print(G[1][i][j])
            c2m = G[0][G[1][i][j]]
            #print(c1m)
            #print(c2m)

            c1 = word_to_map_coord(image.size, (600, 600), [c1m])[0]
            c2 = word_to_map_coord(image.size, (600, 600), [c2m])[0]
            #print(c1)
            #print(c2)

            start_point = c1  # Point de départ de la ligne (x1, y1)
            end_point = c2  # Point de fin de la ligne (x2, y2)
            line_color = (255, 0, 255)  # Couleur de la ligne (rouge)
            line_width = 1  # Largeur de la ligne
            draw.line([start_point, end_point], fill=line_color, width=line_width)

    for i in range(len(G[0])):
        c1 = word_to_map_coord(image.size, (600, 600), [G[0][i]])[0]
        image.putpixel(c1, (0, 255, 0, 255))

    image.show()"""


# image_originale = Image.open("map_300.png")

# afficher_graph(G, image_originale)


def norme_vecteur(x):
    return math.sqrt(x[0] ** 2 + x[0] ** 2)


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# Trouve l'odre le pluss court pour visiter les éoliennes
def plus_court_chemin(l, pos):
    s_vu = [False for _ in range(len(l))]

    def mini_chemin(l, s):
        if s_vu[s]:
            return (dist(pos, l[s]), [s])
        else:

            seul_pas_vu = True
            for i in range(len(l)):
                if i != s and not s_vu[i]:
                    seul_pas_vu = False

            if seul_pas_vu:
                return (0, [s])

            s_vu[s] = True
            mini = 999999999
            best_parcours = []
            for k in range(len(l)):
                if not s_vu[k]:
                    somme = dist(l[s], l[k])
                    sortie_rec = mini_chemin(l, k)
                    d_rec = sortie_rec[0]
                    s_precedent = sortie_rec[1]
                    somme += d_rec
                    if somme < mini:
                        mini = somme
                        best_parcours = s_precedent
                    # print("bucle")
                    # print([s] + s_precedent)
                    # print(somme)
                    # print()
            s_vu[s] = False
            return (mini, [s] + best_parcours)

    mini = 999999999
    best_parcours = []
    for i in range(len(l)):
        v, p = mini_chemin(l, i)
        if v < mini:
            best_parcours = p
            mini = v
    return (mini, best_parcours)


# Détermine si 2 segments se croisent
def se_croise(s1, s2):
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
    # print(c1)
    c2 = (s2[1][1] - s2[0][1]) / (s2[1][0] - s2[0][0])
    # print(c2)
    d1 = s1[0][1] - s1[0][0] * c1
    d2 = s2[0][1] - s2[0][0] * c2
    # print(d1)
    # print(d2)
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


def composante_connexe(G, s):
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


def trouver_chemin(G, depart, arrivee):
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


# print("TEST COMPOSANT CONNEXE")
# s_test = 0
# print(G[0][s_test])
# c_connexe = composante_connexe(G,s_test)
# for i in c_connexe:
#    image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][i]])[0], (0, 0, 255, 255))
# print("FIn TEST COMPOSANTE CONNEXE")
# image_originale.show()

pos = (0, 0)

# print(se_croise(((0,0),(150,150)),((0,5),(0.000000001,-15))))



#PARAMETRE NAVIGATION
l_eolienne = [(-137.07841337181242, 145.127446451792), (-105.59533937321473, -41.79734152154377),
              (139.4011694854088, 85.02789988405654)]

d, ordre_visite = plus_court_chemin(l_eolienne, pos)
# print(ordre_visite)
eolienne_vu = [False, False, False]

# liste du chemin en cours
liste_chemin_en_cours = []


def point_atteint():
    if len(liste_chemin_en_cours) > 0:
        liste_chemin_en_cours.pop(0)


def next_point(pos):
    if len(liste_chemin_en_cours) != 0:
        return G[0][liste_chemin_en_cours[0]]
    else:
        next_pos = prochain_points_etape1(l_eolienne, eolienne_vu, G, pos, (-9999999, -999999))
        return next_pos


# vect vitesse est la vitesse du bateau genre où il va aller a la prochaine fram mais probablement pas improtant
def prochain_points_etape1(l_eolienne, eolienne_vu, G_caillou, pos, vect_vitesse):
    i_actuel = 0
    while i_actuel < len(eolienne_vu) and eolienne_vu[ordre_visite[i_actuel]]:
        i_actuel += 1

    if i_actuel == len(eolienne_vu):
        print("FINI")
        return (999, 999)

    i_eolienne = ordre_visite[i_actuel]

    # print(l_eolienne)
    vect_eolienne = [l_eolienne[i_eolienne][0] - pos[0], l_eolienne[i_eolienne][1] - pos[1]]
    # print(vect_eolienne)

    # def draw_line(p1, p2, col):
    #    # Dessine ligne vecteur
    #    draw = draw = ImageDraw.Draw(image_originale)
    #    start_point = word_to_map_coord(image_originale.size, (600, 600), [p1])[
    #        0]  # Point de départ de la ligne (x1, y1)
    #    end_point = word_to_map_coord(image_originale.size, (600, 600), [p2])[0]  # Point de fin de la ligne (x2, y2)
    #    line_color = col  # Couleur de la ligne (rouge)
    #    line_width = 1  # Largeur de la ligne
    #    draw.line([start_point, end_point], fill=line_color, width=line_width)

    # draw_line(pos, l_eolienne[i_eolienne], (255, 0, 0, 255))

    eolinne_cible_pos = l_eolienne[i_eolienne]

    n_vect = norme_vecteur(vect_eolienne)
    vect_eolienne[0] /= n_vect
    vect_eolienne[1] /= n_vect
    # print("Vect Eolienne :")
    # print(vect_eolienne)
    # print(norme_vecteur(vect_eolienne))
    # print("-----------")
    # Regarder la vitesse desire en fonctio nde la carte des distances

    d_caillou = get_distance_caillou(pos)

    # trouver un rayon de surveillance das lequel on veut attraper tout les rochers
    r = 0
    if d_caillou < 125:
        r = 30
    # print(r)

    # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [pos])[0], (255, 255, 0, 255))

    # Regarder la proximité des rochers autour du rayon

    V = G[0]
    n = len(V)
    caillou_a_portee = []
    for i in range(n):
        if dist(pos, V[i]) < r:
            caillou_a_portee.append(i)
            # print(V[i])
            # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [V[i]])[0], (255, 0, 0, 255))

    # Affiche la carte et les points dans le rayon autour du  bato
    vrai = False
    if vrai:
        # draw = ImageDraw.Draw(image_originale)
        # centre = word_to_map_coord(image_originale.size, (600, 600), [pos])[0]
        rayon = 820 * r / 600
        # Définir les coordonnées du cercle (bounding box)
        # x0, y0 = centre[0] - rayon, centre[1] - rayon
        # x1, y1 = centre[0] + rayon, centre[1] + rayon

        couleur = (255, 0, 0)
        epaisseur = 1
        # Dessiner le cercle (ellipse avec ratio 1:1 est un cercle)
        # draw.ellipse([x0, y0, x1, y1], outline=couleur, width=epaisseur)

        # print(caillou_a_portee)

    # Retenir les points qui sont a la portée du vecteur + un peu plus loin
    # vitesse_actuelle = norme_vecteur(vect_vitesse)

    RANGE = 30
    vitesse_actuelle = RANGE * (255 - get_distance_caillou(pos)) / 255

    # print(vitesse_actuelle)

    vect_eolienne = mul_vect(vect_eolienne, vitesse_actuelle)

    # draw_line(pos, add_vect(vect_eolienne, pos), (255, 0, 255, 255))

    # Regarder si le vecteur "traverse la ligne des points" ou "va vers l'intérieur des rochers"

    points_qui_coupent = []
    s_direction = (pos, add_vect(pos, vect_eolienne))
    for i in range(len(caillou_a_portee)):
        caillou = caillou_a_portee[i]
        v_caillou = G[1][caillou]
        for j in range(len(v_caillou)):
            voisin = v_caillou[j]
            segment = (G[0][caillou], G[0][voisin])
            if se_croise(segment, s_direction):
                points_qui_coupent.append(voisin)
    # print("points qui se coupent")
    # print(points_qui_coupent)

    # Afficher les points qui on un sgment qui fait que le cecteur du bateau se coupe avec
    # for p in points_qui_coupent:
    #    point = G[0][p]
    # print(point)
    # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [point])[0], (255, 255,255, 255))

    # Si non continuer en mode on s'en fiche
    if len(points_qui_coupent) == 0:
        return add_vect(pos, vect_eolienne)

    # On trouve le point le plus proche des rochers trouvé
    plus_proche = -1
    d_proche = 99999999
    for i in range(len(points_qui_coupent)):
        p_rocher = G[0][points_qui_coupent[i]]
        if d_proche > dist(pos, p_rocher):
            d_proche = dist(pos, p_rocher)
            plus_proche = points_qui_coupent[i]

    # Point du graphe le plus proche du bateau
    plus_proche_bateau = plus_proche
    # image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][plus_proche]])[0], (125, 125, 125, 255))

    # print("FIN TESt")

    # On dis qu'on y va et on set_up une liste de point a parcourir ?

    point_connecte_au_caillou = composante_connexe(G, plus_proche_bateau)
    plus_proche = -1
    d_proche = 99999999
    for i in range(len(point_connecte_au_caillou)):
        p_rocher = G[0][point_connecte_au_caillou[i]]
        if d_proche > dist(eolinne_cible_pos, p_rocher):
            d_proche = dist(eolinne_cible_pos, p_rocher)
            plus_proche = point_connecte_au_caillou[i]

    plus_proche_eolienne = plus_proche

    c = trouver_chemin(G, plus_proche_bateau, plus_proche_eolienne)

    """print("depart : ")
    print(plus_proche_bateau)
    image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][plus_proche_bateau]])[0],
                             (125, 255, 125, 255))
    print("arrive : ")
    
    image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][plus_proche_eolienne]])[0],(255, 125, 125, 255))
    print(plus_proche_eolienne)
    print("chemin")
    print(c)
    print()
    """
    for p in c:
        liste_chemin_en_cours.append(p)
        # AFficher le chemin
    #        image_originale.putpixel(word_to_map_coord(image_originale.size, (600, 600), [G[0][p]])[0],(255, 255, 255, 255))

    return G[0][c[0]]
    # Si oui calculer un point plus proche en fonction de l'interieur des rochers/
    # pour que le prochain pour soir un peu en dehors du rochers et que ce soit pas sur le graphe sinon cest la cata

    # image_originale.show()

pos = (0,0)
point_atteint()
print(next_point(pos))
#afficher_graph(G, image_originale)
