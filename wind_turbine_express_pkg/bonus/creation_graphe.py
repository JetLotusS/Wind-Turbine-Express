import math

from PIL import Image, ImageDraw

image_originale = Image.open("map_300.png")

def obtenir_point_jaune(img):
    larg, haut = img.size
    points_jaune = []
    #print(larg,haut)

    for y in range(haut):
        for x in range(larg):
            px = img.getpixel((x,y))

            if (px[0] + px[1])/8 > px[2] and (px[0] + px[1])/2 > 155:
                points_jaune.append((x,y))
                #img.putpixel((x,y),(0,255,0,255))
            else:
                pass
                img.putpixel((x,y),image_originale.getpixel((x,y)))

    #            image.putpixel((x, y), nouvelle_couleur)

    #print(points_jaune)
    #print(len(points_jaune))
    #img.show()
    return points_jaune

image = Image.open("map_300(5).png")#

# Utilisation de la fonction pour générer une image#

map_coord = obtenir_point_jaune(image)
print(map_coord)

larg,haut = image.size


def map_to_world_coord(map_size,world_size,map_coord):
    world_coord = []
    for c in map_coord:
        x,y = c[0],c[1]
        world_coord.append(((x/map_size[0]) * world_size[0] -300,-((y/map_size[1]) * world_size[1] -300)))
    return world_coord


def word_to_map_coord(map_size,world_size,map_coord):
    world_coord = []
    for c in map_coord:
        x,y = c[0],c[1]
        world_coord.append((math.floor(((x+300)/(world_size[0])) * map_size[0]),math.floor(((-y+300)/(world_size[1])) * map_size[1])))
    return world_coord


world_coord = map_to_world_coord((larg,haut),(600,600),map_coord)
print(world_coord)


def dist(a,b):
    return math.sqrt((a[0]-b[0])**2 + (b[1]-a[1])**2)


def init_graph(world_coord):
    n = len(world_coord)
    V = world_coord.copy()
    E = [[] for i in range(n)]

    for i in range(n):
        c = world_coord[i]
        plus_proche = -1
        d1 = 999999999
        deux_proche = -1
        d2 = 999999999
        for j in range(n):
            c2 = world_coord[j]
            if i == j:
                continue

            d = dist(c,c2)
            if d < max(d1,d2):

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

        if plus_proche not in E[i]:
            E[i].append(plus_proche)
        if deux_proche not in E[i]:
            E[i].append(deux_proche)

        if i not in E[plus_proche]:
            E[plus_proche].append(i)
        if i not in E[deux_proche]:
            E[deux_proche].append(i)

    return (V,E)

print("\nG : ")
G = init_graph(world_coord)
print("V : ")
print(G[0])
print("E : ")
print(G[1])


def afficher_graph(G,image):
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


            c1 = word_to_map_coord(image.size,(600,600),[c1m])[0]
            c2 = word_to_map_coord(image.size,(600,600),[c2m])[0]
            #print(c1)
            #print(c2)


            start_point = c1  # Point de départ de la ligne (x1, y1)
            end_point = c2  # Point de fin de la ligne (x2, y2)
            line_color = (255, 0, 255)  # Couleur de la ligne (rouge)
            line_width = 1  # Largeur de la ligne
            draw.line([start_point, end_point], fill=line_color, width=line_width)

    for i in range(len(G[0])):
        c1 = word_to_map_coord(image.size,(600,600),[G[0][i]])[0]
        image.putpixel(c1,(0,255,0,255))

    image.show()


def coord_autour(pos):
    l_off = []
    # offset_1 = (1,0)
    # offset_2 = (math.sqrt(2)/2,math.sqrt(2)/2)
    d = 10
    l_off.append((pos[0] + d,pos[1]))
    l_off.append((pos[0] ,pos[1] + d))
    l_off.append((pos[0], pos[1] - d))
    l_off.append((pos[0] - d, pos[1]))
    l_off.append((pos[0] + d * math.sqrt(2) / 2, pos[1] + d * math.sqrt(2) / 2))
    l_off.append((pos[0] + d * math.sqrt(2) / 2, pos[1] - d * math.sqrt(2) / 2))
    l_off.append((pos[0] - d * math.sqrt(2) / 2, pos[1] + d * math.sqrt(2) / 2))
    l_off.append((pos[0] - d * math.sqrt(2) / 2, pos[1] - d * math.sqrt(2) / 2))
    return l_off


def spawn_eolienne(G,pos_eolienne):
   # pt = (pos_eolienne[1],-pos_eolienne[0])
  #  pos_eolienne=pt
    pos_eolienne = (pos_eolienne[0], pos_eolienne[1])
    V = 0
    E = 1
    n = len(G[V])
    c_off_eolienne = coord_autour(pos_eolienne)
    for c in c_off_eolienne:
        G[V].append(c)
        G[E].append([])

        i_eolienne = len(G[V]) - 1
        c = c
        plus_proche = -1
        d1 = 999999999
        deux_proche = -1
        d2 = 999999999
        for j in range(n):
            if c == j:
                continue
            c2 = G[V][j]
            #print(c2)
            d = dist(c, c2)
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

        G[E][i_eolienne].append(plus_proche)


#eoliennes_pos = [(-137.07841337181242, 145.127446451792), (-105.59533937321473, -41.79734152154377), (139.4011694854088, 85.02789988405654)]
#for p in eoliennes_pos:
#    spawn_eolienne(G,p)

texte = ""
texte += str(len(G[0])) + "\n"
for c in G[0]:
    x = c[0]
    y = c[1]
    texte += str(x) + "," + str(y) + "\n"

for i in range(len(G[0])):
    for j in range(len(G[1][i])):
        texte += (str(i) + "->" + str(G[1][i][j]) + "\n")

print(texte)


file = open("graph_rocher.txt","w")

file.write(texte[0:len(texte) - 1])



file.close()

afficher_graph(G,image)
