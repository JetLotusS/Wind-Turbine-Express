

def generate_all_perm(n):
    chiffre_utilise = [False] * n

    def generate_perm(k):
        if k == 0:
            return []
        l = []
        chiffre_dispo = []
        for i in range(n):
            if not chiffre_utilise[i]:
                chiffre_dispo.append(i)
        
        for j in range(k):
            c_actuel = chiffre_dispo[j]
            chiffre_utilise[c_actuel] = True
            l_rec = generate_perm(k-1)
            if l_rec == []:
                l.append([c_actuel])
            for li in l_rec:
                l.append([c_actuel] + li)
            chiffre_utilise[c_actuel] = False

        return l
    
    return generate_perm(n)

# l = generate_all_perm(8)
# print(l)
# print(len(l))