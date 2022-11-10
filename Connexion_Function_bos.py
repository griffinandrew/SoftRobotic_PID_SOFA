from os import getcwd, chdir, mkdir
from datetime import datetime
import csv
import time

""" Fonction connection à la simulation """
def CavityConnect(RootNode,module):
    stiffNode = RootNode.getChild('RigidFrames')
    position = stiffNode.getObject('DOFs')
    ind = -1
    pressure = []
    txt_chmbre = ""
    print('-------')
    print('Nodes that are connected : ')
    for i in range(1):
        for j in range(module.nb_cavity):
            ind = ind + 1
            node_name = "Bellow" + str(j+1) + str(i+1)            
            noeud = stiffNode.getChild(node_name)
            print(node_name)
            pressure.append(noeud.getObject('SPC'))
    print('-------')
    print(' ')
    return pressure, txt_chmbre

def OpenPrintFile(module,txt_chmbre,file_type,nom_fichier):
        path = getcwd()
        d_et_h = str(datetime.now())
        nf = path + '/record/' + nom_fichier + d_et_h[0:19] + file_type # '.csv' or '.txt'
        fichier = open(nf,'x')

        fichier.write('Caracteristiques des modules de la simulation : ')
        fichier.write('\n Hauteur du module (en mm) : , ' + str(module.h_module))
        fichier.write('\n Pression initiale : , ' + str(module.init_pressure_value))
        # self.fichier.write('Hauteur du module, en mm :', str(module.value_type))
        fichier.write('\n Module de Young des parties souples : , ' + str(module.YM_soft_part))
        fichier.write('\n Module de Young des parties rigides : , ' + str(module.YM_stiff_part))
        fichier.write('\n Coefficient de Poisson : , ' + str(module.coef_poi))
        fichier.write('\n Nombre de modules robotiques : ,'+ str(module.nb_module))
        fichier.write('\n Nombre de paires de cavites par module : , '+ str(module.nb_cavity))
        fichier.write('\n Nombre de poutres le long de l ensemble des modules : , '+ str(module.nb_poutre))
        fichier.write('\n Pression maximale : , '+ str(module.max_pression))
        fichier.write('\n Masse d un module (en kg) : , '+ str(module.masse_module))
        # if module.rigid_bool == 1 : # pertinent ?
        fichier.write('\n Application des parties rigides : , '+ str(module.rigid_bool))
        fichier.write('\n Modele 3D des modules : , '+ str(module.module_model))
        fichier.write('\n Modele 3D des chambres : , '+ str(module.chamber_model))
        fichier.write('\n')
        fichier.write('\n [Positions effecteur] , [Temps relatif] ' + txt_chmbre + '\n')

        return nf, fichier

def kPa_to_bar(tab_in):
    # Conversion  d'un tableau 1d de kPa en bar (titre relativement explicite)
    tab_out = [] # pour appliquer la même transformation à tout le tableau python
    for i in range(len(tab_in)):
        tab_out.append(tab_in[i]/100) # on divise par 100 pcq 1 bar = 100 kPa
    # print(tab_out)
    return tab_out

# def bar_to_kPa(tab_in): # not used and not tested
#     # Conversion  d'un tableau 1d de kPa en bar (titre relativement explicite)
#     tab_out = [] # pour appliquer la même transformation à tout le tableau python
#     for i in range(len(tab_in)):
#         tab_out.append(tab_in[i]*100) # on divise par 100 pcq 1 bar = 100 kPa
#     # print(tab_out)
#     return tab_out
