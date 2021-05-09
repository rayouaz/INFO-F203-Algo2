#PROJET ALGORITHMIQUE 2
#Novembre 2018
#Martis William 000441157
#Rayan Ayouaz 000429095

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import copy
from random import randint

#np.random.seed(1)
random_tree = {
    1 : [2,3],
    2 : [4,5,6],
    3 : [7,8,9],
    4 : [],
    5 : [10,11],
    6 : [],
    7 : [],
    8 : [12],
    9 : [],
    10 : [],
    11 : [13,14],
    12 : [],
    13 : [],
    14 : [15],
    15 : []
}


_list1 = ["null",2,-5,-1,4,-1,-1,-1,-2,2,4,-5,1,-1,3,-1]
_list2 = ["null",-1,-3,-2,-4,-5,1,-3,-2,-3,-1,-4,1,-3,-1,-2]
_list3 = ["null"]+[randint(-15,15) for i in range (len(random_tree))]


def clean_node_delete(tree,node):
    #Destruction du sommet + tout les sommets dépendant dans le graph
    for i in tree[node]:
        tree.pop(i)
    tree.pop(node)
    for _node in tree:
        f_node = tree[_node]
        for j in range(len(tree[_node])):
            if tree[_node][j] == node:
                f_node = tree[_node][:j] + tree[_node][j+1:]
        tree[_node] = [i for i in f_node]


def max_subtree(tree,weights,node=1,prices=[0]):
    '''L'algorithme utilise la programmation dynamique
    et est basé sur le fait que l'arbre de poids maximum
    est la somme de tout les sous arbre de poids positif'''
    #Le poids à chaque niveau est ajouté à celui du père
    prices.append(prices[-1]+weights[node])
    #Parcours en profondeur
    childs = tree[node]
    for child in childs:
        max_subtree(tree,weights,child,prices)
    #On remonte ("virtuellement" sur une feuille)
    #Si le nouveau est plus petit que le premier
    #Ou que le sous-arbre unaire engendré est négatif
    # on supprimer le noeud
    if prices[-1] < prices[-2] or prices[-1] < 1:
        clean_node_delete(tree,node)
    else:
        #Virtuellement on fusionne deux noeuds
        prices[-2] = prices[-1]
    #Le poid est supprimé car soit fusionné soit supprimé
    prices.pop()
    return tree



def generate_hypergraph():
    ''' Creee une matrice de booleens M ou une entree M[i][j] vaut 1 si un sommet (ligne) i appartient a
        une hyperarete (colonne) j. La taille, ainsi que la distribution des
        booleens de la matrice est aleatoire dans un interval afin d'avoir une complexite un minimum
        interressante mais pas illisible.
    '''
    vertices = np.random.randint(4,16)            #nombre de sommets (lignes dans la matrice)
    hyper_edges = np.random.randint(4,11)    #nombre de hyperaretes (colonnes dans la matrice)

    distribution = (hyper_edges*vertices-1)/150 #plus la matrice est grande plus il y a de chance d'avoir des 0
    hypergraph = np.random.choice([0, 1], size=(vertices,hyper_edges),p = [distribution,1-distribution]) #cree la matrice avec la taille et distribution
    print(hypergraph)
    return hypergraph

def convert_to_dual(hypergraph):
    '''Retourne l'hypergraphe dual d'un hypergraphe.
        Permute les sommets et hyperaretes, revient a la transposee de la matrice '''
    return np.transpose(hypergraph)

def build_primal(hypergraph):
    '''Construit le graphe (networkx) primal d'un hypergraphe en parcourant les colonnes, puis les lignes et ajoute
        une arete entre 2 sommets si ils sont dans la meme hyperarete'''

    vertices = hypergraph.shape[0]
    hyper_edges = hypergraph.shape[1]
    primal = nx.Graph()
    primal.add_nodes_from(range(vertices))   #ajouter sommet au graphe networkx pour chaque ligne

    for j in range(hyper_edges):    #Pour chaque hyperarete
        vertice_indexes = []             #initialiser une liste d'indexes des sommets appartenant a l'hyperarete
        for i in range(vertices):       #Pour chaque sommet

            if hypergraph[i][j]:        #si il appartient a l'hyperarete
                for k in vertice_indexes:   #Pour chaque indice de sommet de l'hyperarete deja detecte
                    primal.add_edge(k,i)    #ajouter une arete entre les sommets sur le graphe networkx
                vertice_indexes.append(i) #ajouter le sommet a la listes d'indexes detectes

    return primal

def check_maxcliques(hypergraph):
    '''Renvoie True si toute clique maximale de taille deux ou plus dans le graphe est une hyper-arête'''
    maxcliques_check = True
    primal = build_primal(hypergraph)       #construire le graphe networkx
    cliques = list(nx.find_cliques(primal)) #trouver les cliques maximales et les mettre sous forme de liste
    hyper_edges = get_hyper_edges(hypergraph)   #trouver les indices des sommets dans les hyper aretes

    i = 0
    while i < len(cliques) and maxcliques_check:

        if len(cliques[i]) > 1 and sorted(cliques[i]) not in hyper_edges: #trie les cliques pour les comparer au hyperaretes
            maxcliques_check = False
        i += 1

    return maxcliques_check


def get_hyper_edges(hypergraph):
    '''Renvoie les indices des sommets dans les hyper-aretes sous forme de liste de listes'''
    hyper_edges = []

    for i in hypergraph.T:          #pour chaque colonne (lignes de la transposee)
        hyper_edge = np.nonzero(i)[0].tolist()   #trouver chaque indice de l'hyperarete dont l'element est non nul
        hyper_edges.append(sorted(hyper_edge))   #trier les indices et les ajouter a la liste                       #not append if empty or < 2?

    return hyper_edges

def test_hypertree(hypergraph):
    ''' Convertie l'hypergraphe en son hypergraphe dual, construit son graphe primal networkx, l'affiche avec matplotlib.
        Verifie si le graphe est cordal et que toute clique maximale de longueur 2 ou plus est une hyperarete.
        Renvoie True si les deux conditions sont respectees'''

    dual_hypergraph = convert_to_dual(hypergraph)       #convertir en hypergraphe dual
    primal = build_primal(dual_hypergraph)              #construire le graphe primal
    labels = {i: 'E' + str(i + 1) for i in range(hypergraph.shape[1])} #r'$E\nu$'
    nx.draw(primal, pos=nx.shell_layout(primal), labels=labels, alpha=0.6) #l'afficher avec des etiquettes pour chaque sommet

    chordal = nx.is_chordal(primal)                         #verifier si le graphe est cordal
    maxcliques_check = check_maxcliques(dual_hypergraph)    #verifier si les cliques max >=2 sont des hyperaretes
    isHyperTree = True if chordal and maxcliques_check else False
    print('isHyperTree:',isHyperTree)
    plt.show()

    return isHyperTree

#bonus
def find_PEO(graph):
    '''Trouve un ordonnancement d'elimination parfaite en temps lineaire selon la methode de recherche de cardinalite maximum
        (maximum cardinality search). Renvoie l'ordonnancement sous forme de liste de sommets'''
    PEO = []
    to_place = {node:0 for node in graph.nodes()}  #dictonnaire ayant pour clefs les sommets qui sont a placer dans
    #l'ordonnancement parfait et ont comme valeur le nombre de voisins deja places
    for i in range(len(to_place)):          #pour chaque sommet a placer
        pick = max(to_place, key=to_place.get)  #prendre celui qui a le plus de voisins deja places
        del to_place[pick]                      #le supprimer du dictionnaire de sommets a placer
        PEO.insert(0,pick)             #inserer au debut de l'ordonnancement (au lieu d'inverser l'ordre plus tard)

        for j in graph.neighbors(pick):   #incrementer la valeur de tous les sommets voisins qui sont encore a placer
            if j not in PEO:
                to_place[j] += 1

    return PEO

def cover_hypertree(hypergraph):
    '''Trouve une couverture exacte pour un hypertree en temps polynomial en trouvant un ensemble independant
    de poids total maximal (maximum weight independent set) ou les poids sont les cardinalites des hyperaretes
    Affiche la couverture sous forme de liste d'hyper-aretes si et renvoie True si une couverture existe, sinon False'''

    dual_hypergraph = convert_to_dual(hypergraph)  # convertir en hypergraphe dual
    primal = build_primal(dual_hypergraph)
    perfect_order = find_PEO(primal)

    L = []
    n = len(perfect_order)
    w = [len(np.nonzero(i)[0].tolist()) for i in hypergraph.T]  #liste des poids (cardinalite) des hyper-aretes
    marks = [False for i in range(n)]                           #liste des marquages des hyper-aretes

    for i in range(n):
        if w[perfect_order[i]] > 0:             #si le poids est superieur a 0
            marks[perfect_order[i]] = True              #marquer l'hyperarete
            for j in list(primal.neighbors(perfect_order[i])):
                w[j] = max(w[j] - w[perfect_order[i]], 0)

    for i in range(n - 1, -1, -1):
        if marks[perfect_order[i]]:                 #ajouter l'hyperarete a la liste si elle est marquee
            L.append(perfect_order[i])
            for j in primal.neighbors(perfect_order[i]):
                marks[j] = False

    w = [len(np.nonzero(i)[0].tolist()) for i in hypergraph.T]
    check = sum([w[i] for i in L])
    if check == hypergraph.shape[0]:         #si le poids total est egal a la cardinalite de l'ensemble des sommets
        print('Exact cover found with hyperedges:', [i+1 for i in L] )  #une couverture exacte est trouvee, l'afficher
        found = True
    else:
        found = False

    return found

# test = np.array([[1,0,0,0],     #exemple de l'enonce
#                  [1,1,0,0],
#                  [1,1,1,0],
#                  [0,0,0,1],
#                  [0,0,1,0],
#                  [0,0,1,0],
#                  [0,0,0,0]])
#
# test1 = np.array([[1,1,0,0],    #exemple 2 de l'enonce
#                  [1,0,0,0],
#                  [1,0,1,0],
#                  [0,0,0,1],
#                  [0,1,1,0],
#                  [0,0,1,0],
#                  [0,0,0,0]])
#
# test2 = np.array([[1,0,0,0],    #exemple de hypertree qui a une couverture exacte
#                  [1,1,0,0],
#                  [0,1,1,0],
#                  [0,0,0,1],
#                  [0,0,1,0],
#                  [0,0,1,0]])


def main():
    #EXO 1
    print ('Exercice 1: ')
    print ('\nEnoncé: ',_list1[1:])
    print(max_subtree(copy.deepcopy(random_tree),_list1))
    print ('\nEnsemble vide: ',_list2[1:])
    print(max_subtree(copy.deepcopy(random_tree),_list2))
    print ('\nRandom: ',_list3[1:])
    print(max_subtree(copy.deepcopy(random_tree),_list3))
    #EXO 2
    print ('Exercice 2:')

    for i in range(5):
        hypergraph = generate_hypergraph()
        test_hypertree(hypergraph)
        cover_hypertree(hypergraph)
        print()

if __name__ == '__main__':
    main()


