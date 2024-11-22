from random import randint
import math


def wide_perm(iteration, n):
    """Décide de l'écart de la permutation qui va être effectué\n 
    c'est a dire de l'écart entre i et j pour échanger le contenu des case i et j dans la permutation"""
    racine = math.ceil(math.sqrt(n))
    courbe_d = math.ceil(math.sqrt(n))
    courbe_offset = math.ceil(math.sqrt(math.sqrt(n)))
    return max(1, min(courbe_d, -iteration // racine + courbe_offset + courbe_d))


def n_perm(iteration, n):
    """Décide du nombre de permutation effectuée"""
    racine = math.ceil(math.sqrt(n))
    return racine


def mutate(fourmis, iteration, n_points):
    """Mute le chemin d'une fourmis en permuttant des indices de son chemin"""
    new_f = fourmis.copy()
    n = len(fourmis)
    for i in range(n_perm(iteration, n_points)):
        i_echange = randint(0, n - 1)
        taille_echange = max(wide_perm(iteration, n_points) - i, 1)
        temp = new_f[(i_echange + taille_echange) % n]
        new_f[(i_echange + taille_echange) % n] = new_f[i_echange]
        new_f[i_echange] = temp
    return new_f


def distance(a, b):
    """Calcule la distance entre 2 points a et b"""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def calcul_cout(fourmis, points):
    """Calcule le cout d'un chemin effectuée par une fourmis"""
    n = len(fourmis)
    cout = distance(points[0],(0,0))
    for i in range(n):
        i0 = fourmis[i]
        i1 = fourmis[(i + 1) % n]
        cout += distance(points[i0], points[i1])
    return cout


def calcul_liste_cout(l_combattants, points):
    """Renvoie la liste des cout de la liste des combattant en arguments"""
    l_cout = []
    for i in range(len(l_combattants)):
        l_cout.append((calcul_cout(l_combattants[i], points), i))
    return l_cout


def keep_first(l_combatant):
    """Trie la liste en fonction du cout des fourmis, pour avoir celle qui on le plus faible cout en premier ! """
    l_combatant.sort()


def mutate_first(l_combatant, l_cout, iteration, n_fourmis_retenu, n_enfant, n_points):
    """Fait muter les premieres fourmis"""
    l = []
    for i in range(n_fourmis_retenu):
        fourmis_actuel = l_combatant[l_cout[i][1]]
        l.append(fourmis_actuel)
        for j in range(n_enfant):
            l.append(mutate(fourmis_actuel, iteration, n_points))
    return l


def glouton(touts_les_points):
    """Fais un parcours glouton de recherche de chemin le plus court entre les points.\n
    Les fourmis ainsi initialisée ont un bien meilleur départ qu'une permutation aléatoire."""
    n = len(touts_les_points)

    depart = randint(0, n - 1)

    vu = [False for i in range(n)]
    vu[depart] = True
    chemin = [depart]

    for i in range(n - 1):
        s_actuel = chemin[-1]
        d_mini = 99999999999
        meilleur_sommet = depart
        for j in range(n):
            if not vu[j] and distance(touts_les_points[s_actuel], touts_les_points[j]) < d_mini:
                d_mini = distance(touts_les_points[s_actuel], touts_les_points[j])
                meilleur_sommet = j

        vu[meilleur_sommet] = True
        chemin.append(meilleur_sommet)
    return chemin


def run_combat_genetic(points):
    """Algorithme de selection génétique pour trouver un chemin le plus court possible parcourant tout les points."""
    iteration = 0

    n = len(points)
    n_fourmis_retenu = n
    racine = math.ceil(math.sqrt(n))
    n_enfant = racine
    n_iteration = 150

    liste_combattant = [glouton(points) for i in range(100)]

    cout_fourmis = calcul_liste_cout(liste_combattant, points)

    keep_first(cout_fourmis)

    while iteration < n_iteration:
        liste_combattant = mutate_first(liste_combattant, cout_fourmis, iteration, n_fourmis_retenu, n_enfant, n)

        cout_fourmis = calcul_liste_cout(liste_combattant, points)

        keep_first(cout_fourmis)

        iteration += 1

    keep_first(cout_fourmis)
    return liste_combattant[cout_fourmis[0][1]], calcul_cout(liste_combattant[cout_fourmis[0][1]], points)
