#!/usr/bin/env python;
# -*- coding: utf-8 -*-
from zipfile import ZIP_STORED
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import Sofa.constants.Key as Key
from spicy import *
from os import getcwd, chdir, mkdir
from datetime import datetime
import csv
import time
import Connexion_Function_bos as connect
import serial
import math
import copy
import pyfirmata
from pyfirmata import util, STRING_DATA
from csv import writer
from re import L


# pi = 3.14 # remplacé par math.pi

#board = pyfirmata.Arduino("COM7") 

# réunit toutes les fonctions python, permettant de :
# - controller le robot en pression/volume
# - imprimer les positions de l'effecteur dans le terminal
# - enregistrer les temps, pressions et positions correspondantes dans des fichiers txt ou csv

 ### - CONTROLLER - ###
class PressureController(Sofa.Core.Controller):

    def __init__(self,pressure_step,max_pressure,module,*args, **kwargs):

            Sofa.Core.Controller.__init__(self,args,kwargs)
            # self.node = args[0]
            self.RootNode = kwargs["RootNode"]
            self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.RootNode,module=module)
            self.flag = 0
            self.pressure_step = pressure_step
            self.max_pressure = max_pressure
            # self.nb_module = module.nb_module # que pour stiff_flop
            self.nb_cavity = module.nb_cavity


    def onKeypressedEvent(self,e):
        
            if e["key"] == Key.T: # switche d'un module à un autre pour l'actionnement
                if self.flag < self.nb_module - 1:
                    self.flag = self.flag + 1
                    print('Switch au mondule n° : ',self.flag+1)
                    # print(self.flag)
                else:
                    self.flag = 0
                    print('Switch au mondule n° : ',self.flag+1)
                    # print(self.flag)

            pressureValue = zeros(self.nb_cavity)

            index = self.flag*self.nb_cavity
            for i in range(self.nb_cavity):
                pressureValue[i] = self.pressure[index+i].value.value[0]

            if e["key"] == Key.D:
                pressureValue[0] += self.pressure_step
                # print('===========D')
                if pressureValue[0] > self.max_pressure:
                    pressureValue[0]= self.max_pressure
            if e["key"] == Key.C:
                pressureValue[0] -= self.pressure_step
                if pressureValue[0] < 0:
                    pressureValue[0] = 0

            if e["key"] == Key.F:
                # print('===========F')
                pressureValue[1] += self.pressure_step
                if pressureValue[1] > self.max_pressure:
                    pressureValue[1] = self.max_pressure
            if e["key"] == Key.V: # S déjà pris, je met F à la place
                pressureValue[1] -= self.pressure_step
                if pressureValue[1] < 0:
                    pressureValue[1] = 0

            if e["key"] == Key.G:
                # print('==========G')
                pressureValue[2] += self.pressure_step
                if pressureValue[2] > self.max_pressure:
                    pressureValue[2] = self.max_pressure
            if e["key"] == Key.B:
                pressureValue[2] -= self.pressure_step
                if pressureValue[2] < 0:
                    pressureValue[2] = 0
                    
            print('         ****       ')
            print('Control du mondule n° : ',self.flag+1)
            for i in range(self.nb_cavity): # remise valeurs au bon endroit
                self.pressure[index+i].value = [pressureValue[i]]
                print('Pression chambre ',i,' : ',pressureValue[i])
            print('         ****       ')

### - VIEWER - ###
class PositionViewer(Sofa.Core.Controller):
    def __init__(self,nb_poutre,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.node = args[0]
        self.stiffNode=self.node.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = nb_poutre

    def onKeypressedEvent(self,e):
        pos = self.position.position.value[self.nb_poutre-1][0:3]
        print('         ----       ')
        print('Position effecteur : ',pos)
        print('         ----       ')

 ### - TXT PRINTER - ###
class PositionPrinterTxt(Sofa.Core.Controller): # utile ????
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.node = args[0]
        self.stiffNode = self.node.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.node,module=module)
        self.nf, self.fichier_txt = connect.OpenPrintFile(module,txt_chmbre,'.txt','pos_stiff_record_')
        self.start = time.time()

    def onKeypressedEvent(self,e):
        # print(self.nf)
        # ATTENTION INCOMPLET => REGARDER CSV
        pos = self.position.position.value[self.nb_poutre-1][0:3]
        ind = 0
        # pres = []
        pres_txt = ""
        for i in range(self.nb_module):
            pres_txt = pres_txt + " - ["
            for j in range(self.nb_cavity):
                pres_txt = pres_txt + ' ' + str(self.pressure[ind].value.value[0]) # for controller
                # pres_txt = pres_txt + ' ' + str(self.pressure[ind].pressure.value) # for qp
                ind = ind + 1
            pres_txt = pres_txt + " ]"

        self.fichier_txt.write(str(pos) + pres_txt +'\n')
        self.fichier_txt.close()
        print("%%%% Positions Enregistrées en Txt %%%%")
        self.fichier_txt = open(self.nf,'a')


### - CSV PRINTER - ###
class PositionPrinterCsv(Sofa.Core.Controller):
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.node = args[0]
        self.stiffNode = self.node.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.node,module=module)
        self.nf, self.fichier_csv = connect.OpenPrintFile(module,txt_chmbre,'.csv','pos_stiff_record_')
        self.start = time.time()

    # def onAnimateBeginEvent(self, dt): 
    def onKeypressedEvent(self,e):
        # print(self.nf)
        pos = self.position.position.value[self.nb_poutre-1][0:3]
        ind = 0
        # pres = []
        print(str(time.time() - self.start)) 
        time_txt = ", [" + str(time.time() - self.start) + "]"
        pres_txt = ""
        for i in range(self.nb_module):
            pres_txt = pres_txt + ",["
            i0 = ind
            for j in range(self.nb_cavity):
                pres_txt = pres_txt + ' ' + str(self.pressure[ind].value.value[0]) # for controller 
                ind = ind + 1
            pres_txt = pres_txt + "]"
            ind = i0
            pres_txt = pres_txt + ",["
            for j in range(self.nb_cavity):
                pres_txt = pres_txt + ' ' + str(self.pressure[ind].cavityVolume.value)
                ind = ind + 1
            pres_txt = pres_txt + "]"

        self.fichier_csv.write(str(pos) + time_txt + pres_txt +'\n')
        self.fichier_csv.close()
        print("%%%% Positions Enregistrées en Csv %%%%")
        self.fichier_csv = open(self.nf,'a')

class GoalPrinterCsv(Sofa.Core.Controller):
    # pour enregistrer toutes les positions désirées succéssives dans un csv => pour pouvoir enregistrer les commandes faites à la souris 
        def __init__(self,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.node = args[0]
            txt_chmbre = ""
            self.nf, self.fichier_csv_goal = connect.OpenPrintFile(module,txt_chmbre,'.csv','goal_pos_record_')
            self.stiffNode = self.node.getChild('goal')
            self.position = self.stiffNode.getObject('goalMO')

        def onAnimateBeginEvent(self, dt): # choisir ce qui convient le mieux
        # def onMouseEvent(self,e): 
            pos = self.position.position.value
            self.fichier_csv_goal.write(str(pos) +'\n')
            self.fichier_csv_goal.close()
            print("%%%% Positions Goal Enregistrées en Csv %%%%")
            self.fichier_csv_goal = open(self.nf,'a')

class GoalKeyboardController(Sofa.Core.Controller):
    # pour controller la position de l'effecteur désiré avec le clavier
        def __init__(self,goal_pas,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.node = args[0]
            self.stiffNode = self.node.getChild('goal')
            self.position = self.stiffNode.getObject('goalMO')
            self.pas = goal_pas

        def onKeypressedEvent(self,e):

            d = copy(self.position.position.value)
            if e["key"] == Key.D:
                d[0][0] += self.pas  
            if e["key"] == Key.C:
                d[0][0] -= self.pas  

            if e["key"] == Key.F:
                d[0][1] += self.pas  
            if e["key"] == Key.V:
                d[0][1] -= self.pas  

            if e["key"] == Key.G:
                d[0][2] += self.pas  
            if e["key"] == Key.B:
                d[0][2] -= self.pas 

            self.position.position = [d[0]]


class ArduinoPressure(Sofa.Core.Controller):
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        # self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        # self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        # self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        ind = -1
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.RootNode,module=module)

        # self.board = pyfirmata.Arduino('/dev/ttyACM0') # pyfirmata connexion
        # self.led = self.board.get_pin('d:13:o')

        ### Stefan version
        self.SerialObj1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5) #port used by the arduino mega board 

    def onAnimateBeginEvent(self, dt): 
        pres_tab = [copy(self.pressure[0].pressure.value),copy(self.pressure[1].pressure.value),copy(self.pressure[2].pressure.value)]
        # print(pres_tab)

        bar_tab = connect.kPa_to_bar(pres_tab)

        S = "{:,.3f}".format(bar_tab[0]) + "," + "{:,.3f}".format(bar_tab[1]) + "," + "{:,.3f}".format(bar_tab[2]) + "\n"

        print(S)
        ByteStr = S.encode("utf-8")

        self.SerialObj1.write(ByteStr)
        # time.sleep(0.2) #usefull ?
        # print("Step: " + str(i) + ", Bytes sent: " + S)
        # print("  Bytes sent: " + S)

        # # pour le test avec la led
        # pres = pres_tab[0]
        # if pres > 50 :
        #     self.led.write(1)
        # else :
        #     self.led.write(0)


class CircleTrajectory(Sofa.Core.Controller):
# Génère un trajectoire circulaire à l'effecteur
        def __init__(self,rayon,nb_iter,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]
            self.stiffNode = self.RootNode.getChild('goal')
            self.position = self.stiffNode.getObject('goalMO')
            self.nb_iter_d = 30 # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
            self.nb_iter = nb_iter  # nb d'itération pour réaliser un cercle complet
            self.d_flag = 0 # flag, passe à 1 quand le placement sur le cercle est effectué
            self.iter = 1 # numérote les itérations
            self.r = rayon

        def onAnimateBeginEvent(self,e):

            d = copy.copy(self.position.position.value)

            if self.d_flag == 0 :
                d[0][2] = (self.iter/self.nb_iter_d)*self.r
                if self.iter >= self.nb_iter_d:
                    self.d_flag = 1
                    self.iter = 1
            else :
                # in_iter = self.iter - self.nb_iter_d
                d[0][2] = self.r*math.cos((self.iter/self.nb_iter)*2*math.pi);
                d[0][1] = self.r*math.sin((self.iter/self.nb_iter)*2*math.pi);


            self.position.position = [d[0]]

            self.iter += 1

class PressurePrinter(Sofa.Core.Controller):
        def __init__(self,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,*args,**kwargs)
            self.RootNode = kwargs["RootNode"]
            # self.step = step
            self.IterSimu = 0 # Counter for dt steps before stopping simulation
            self.ecart = 0 # ecart entre la simulation et la réalité, en mm
            self.pressure , txt_chmbre = connect.CavityConnect(RootNode = self.RootNode,module = module)
            # txt_chmbre = "" # pour ne pas avoir d'entête fausse
            # self.nf, self.fichier_csv = connect.OpenPrintFile(module,txt_chmbre,'.csv','pressure_record_')


        def onAnimateBeginEvent(self, dt): 
            pres_tab = [float(copy.copy(self.pressure[0].pressure.value)),float(copy.copy(self.pressure[1].pressure.value)),float(copy.copy(self.pressure[2].pressure.value))]
            print(pres_tab)
            # pres_csv = copy(Sim.tab[self.step][self.IterSimu][3])
            # self.fichier_csv.write(str(pres_tab)+str(pres_csv) +'\n')
            # self.fichier_csv.close()
            # print("%%%% Pressions Enregistrées en Csv %%%%")
            # self.fichier_csv = open(self.nf,'a')
            # self.IterSimu += 1

class GoalShift(Sofa.Core.Controller):
    # pour controller la position de l'effecteur désiré avec le clavier
        def __init__(self,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.node = args[0] # args[0] is just rootNode from the main scene
            self.stiffNode = self.node.getChild('goal') # StiffNode is a child of rootNode (goal) which is the actuator tip position ball
            self.position = self.stiffNode.getObject('goalMO') # position is a position object defining the tip ball's position
            self.stiffNode = self.node.getChild('goal2') # redefine StiffNode as the goal2 child of rootNode, which is the user input position
            self.position2 = self.stiffNode.getObject('goalM2') # position2 is a position object defining the user ball's position
            # self.controlledPoints = self.node.getChild('controlledPoints')
            # self.effector = self.controlledPoints.getObject('PositionEffector')

        def onAnimateBeginEvent(self,e):

            d = copy.copy(self.position.position.value) # get value of tip ball position

            d2 = copy.copy(self.position2.position.value) # get value of user ball position

            d[0][0] = d2[0][0] -3
            d[0][1] = d2[0][1]
            d[0][2] = d2[0][2]

            self.position.position = [d[0]]



class ArduinoVolume(Sofa.Core.Controller):
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]#        self.stiffNode = self.RootNode.getChild('RigidFrames')
        #self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre #poutre = beams
        # self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
#        self.nb_volume = module.nb_volume
        # self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm (gap = ecart)
        ind = -1
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.RootNode,module=module)
        #board = pyfirmata.Arduino("COM8")

        ### Stefan version #for the arduino im using the port is just 3
       # self.SerialObj1 = serial.Serial('/dev/ttyUSB', 115200, timeout=0.5) #port used by the arduino mega board

    def onAnimateBeginEvent(self, dt):
        board = pyfirmata.Arduino("COM8")
        vol_tab = [copy(self.pressure[0].volumeGrowth.value),copy(self.pressure[1].volumeGrowth.value),copy(self.pressure[2].volumeGrowth.value)]
        S = "{:,.3f}".format(vol_tab[0]) + ", " + "{:,.3f}".format(vol_tab[1]) + ", " + "{:,.3f}".format(vol_tab[2]) + "\n"
        print(S)
        board.send_sysex(STRING_DATA, util.str_to_two_byte_iter(S))
        #self.SerialObj1.write(ByteStr)


class VolumePrinter(Sofa.Core.Controller):
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        # self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        self.pressure , txt_chmbre = connect.CavityConnect(RootNode = self.RootNode,module = module)
        # txt_chmbre = "" # pour ne pas avoir d'entête fausse
        # self.nf, self.fichier_csv = connect.OpenPrintFile(module,txt_chmbre,'.csv','pressure_record_')


    def onAnimateBeginEvent(self, dt): #is there a volume attribute of the volume part of the object? same with the pressure thing?
        vol_tab = [copy.copy(self.pressure[0].volumeGrowth.value),copy.copy(self.pressure[1].volumeGrowth.value),copy.copy(self.pressure[2].volumeGrowth.value)]
        #S = "{:,.3f}".format(vol_tab[0]) + ", " + "{:,.3f}".format(vol_tab[1]) + ", " + "{:,.3f}".format(vol_tab[2]) + "\n"
        print(vol_tab)


#converges on correct volume after about .5 s 
class goToPosition(Sofa.Core.Controller):
    def __init__(self, x,  y, z, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('goal')
        self.position = self.stiffNode.getObject('goalMO')
        self.x = x
        self.y = y 
        self.z = z 

       # self.prev_pos = 

        f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "w", newline='')
        self.header = ["X", "Y", "Z"]
        self.writer = csv.DictWriter(f,fieldnames=self.header)
        self.writer.writeheader()
        #f.close()
    
    def onAnimateBeginEvent(self, dt):
        d = copy.copy(self.position.position.value)
        print("here")
        xCurr = d[0][0]
        yCurr = d[0][1]
        zCurr = d[0][2] # need to verify these are what i think they correspond to

        x = repr(xCurr)
        y = repr(yCurr)
        z = repr(zCurr)

        print(x  +  " " + y + " " + z)

        if (xCurr - self.x < 0): # means that you need to move up
            x_mv = self.x - xCurr
           # print(x_mv)
            d[0][0] += x_mv
        
        if not(xCurr - self.x < 0):
            x_mv = xCurr - self.x
           # print(x_mv)
            d[0][0] -= x_mv

        if (yCurr - self.y < 0):
            y_mv = self.y - yCurr
           # print(y_mv)
            d[0][1] += y_mv

        if not(yCurr - self.y < 0):
            y_mv = yCurr - self.y
           # print(y_mv)
            d[0][1] -= y_mv

        if (zCurr - self.z < 0):
            z_mv = self.z - zCurr
          #  print(z_mv)
            d[0][2] += z_mv

        if not(zCurr - self.z < 0):
            z_mv = zCurr - self.z
          #  print(z_mv)
            d[0][2] -= z_mv

        #self.position.position = [d[0]]


        f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "a") 
        #f = open("C:\Volume\volume.csv", "a") 
        D = repr(d.tolist())
        print(D)
        vector = d.tolist()
       # print(len(vector))

        print(vector[0][0])
        print(vector[0][1])
        print(vector[0][2])

        x_1=str(vector[0][0])
        y_1=str(vector[0][1])
        z_1=str(vector[0][2])

        print(x_1  +  " " + y_1 + " " + z_1)
        
        self.writer.writerow({"X": x_1,
        "Y":y_1,
        "Z":z_1
        })

        f.close()

        self.position.position = [d[0]]





#################################################################################################################

#go to position reading from csv 


#converges on correct volume after about .5 s 
class goToPositionFromCSV(Sofa.Core.Controller):
    def __init__(self, x,  y, z, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('goal')
        self.position = self.stiffNode.getObject('goalMO')
        self.x = x
        self.y = y 
        self.z = z 

        self.prev_pos = copy.copy(self.position.position.value)

        self.counter = 0 # perhaps the counters # should correspond to what line of the file you want to pass

        #self.line_counter = 0 #this will just keep track of how many lines have been read 

        self.done_flag = 0




    def processStr(self, vector_pos): # this will extract the digits from the the file and set them as the x y z coordinates
        num_space = 0 
        vec_pos = vector_pos

        print('string recieved in proc')
        print(vec_pos)
        
        tempX = ""
        tempY = "" 
        tempZ = "" 

        for element in range(0, len(vec_pos)):  #note that the delimiter is actually a ,
            #print("element")
            #print(vec_pos[element])
            
            if vec_pos[element] == ",":
                num_space+=1
            
            if num_space == 3:
                break

            if (not (vec_pos[element] == ",") and num_space == 0): 
                tempX += str(vec_pos[element])
                #print(tempX)
            
            if (not (vec_pos[element] == ",") and num_space == 1) :
                tempY += str(vec_pos[element])
                #print(tempY)

            if (not (vec_pos[element] == ",") and num_space == 2) :
                tempZ += str(vec_pos[element])
               # print(tempZ)


        #print(type(tempX))
      #  print('within process string')
       # print(tempX +  "      " + tempY + "       " + tempZ)
      #  print("end of process string")


        self.x = float(tempX)
        self.y = float(tempY)
        self.z = float(tempZ)

    def onAnimateBeginEvent(self, dt):

        # get the current position of the simulation
        self.prev_pos = copy.copy(self.position.position.value) #maybe this should be later??? 

        xCurr = self.prev_pos[0][0]
        yCurr = self.prev_pos[0][1]
        zCurr = self.prev_pos[0][2] # need to verify these are what i think they correspond to


        f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "r", newline='')

        position_to_traverse_to = f.readline()
        f.close()

        print("strings to process")

        print(position_to_traverse_to)

        self.processStr(position_to_traverse_to) 

        x = repr(xCurr)
        y = repr(yCurr)
        z = repr(zCurr)

       # print("the points")

        print(x  +  "    " + y + "     " + z)


       # if self.counter == 1: # note that im unsure about this one
         #   self.done_flag == 1


        #if (self.done_flag == 1): 
        #    print("done")
        #    self.prev_pos[0][0] = self.prev_pos[0][0]
        #    self.prev_pos[0][1] = self.prev_pos[0][1] 
        #    self.prev_pos[0][2] = self.prev_pos[0][2]



        #if (self.done_flag == 0):


        if (xCurr - self.x < 0): # means that you need to move up
            x_mv = self.x - xCurr
           # print(x_mv)
            self.prev_pos[0][0] += x_mv
        
        if not(xCurr - self.x < 0):
            x_mv = xCurr - self.x
             # print(x_mv)
            self.prev_pos[0][0] -= x_mv

        if (yCurr - self.y < 0):
            y_mv = self.y - yCurr
           # print(y_mv)
            self.prev_pos[0][1] += y_mv

        if not(yCurr - self.y < 0):
            y_mv = yCurr - self.y
           # print(y_mv)
            self.prev_pos[0][1] -= y_mv

        if (zCurr - self.z < 0):
            z_mv = self.z - zCurr
          #  print(z_mv)
            self.prev_pos[0][2] += z_mv

        if not(zCurr - self.z < 0):
            z_mv = zCurr - self.z
          #  print(z_mv)
            self.prev_pos[0][2] -= z_mv

        #self.counter+=1
        #self.position.position = [d[0]]


        #f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "a") 


       # if self.counter == self.line_counter: # note that im unsure about this one
        #    self.done_flag == 1
        

        

        self.position.position = [self.prev_pos[0]]
        self.prev_pos = self.position.position




############################################################################################################################################


#converges on correct volume after about .5 s 
class goToPositionFromLast(Sofa.Core.Controller):
    def __init__(self, x,  y, z, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('goal')
        self.position = self.stiffNode.getObject('goalMO')
        self.x = x
        self.y = y 
        self.z = z 

        self.prev_pos = copy.copy(self.position.position.value)

        self.counter = 0 # perhaps the counters # should correspond to what line of the file you want to pass

        self.line_counter = 0 #this will just keep track of how many lines have been read 

        self.done_flag = 0



        print("hello MBL")

        f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "r", newline='')

        all_lines = f.readlines()
        print("all lines")
        print(all_lines)

        self.line_list = [] 

        for line in all_lines:
            this_line = line
            print(this_line)
            self.line_list.append(this_line)
            self.line_counter +=1
            # lol do i need a list to store all the strings; i think so

        
        print("line list and count")
        print(self.line_list)
        print(self.line_counter)

        f.close()




       # self.header = ["X", "Y", "Z"]
       # self.writer = csv.DictWriter(f,fieldnames=self.header)
      #  self.writer.writeheader()


    def processStr(self, vector_pos): # this will extract the digits from the the file and set them as the x y z coordinates
        num_space = 0 
        vec_pos = vector_pos

        print('string recieved in proc')
        print(vec_pos)
        
        tempX = ""
        tempY = "" 
        tempZ = "" 

        for element in range(0, len(vec_pos)):  #note that the delimiter is actually a ,
            #print("element")
            #print(vec_pos[element])
            
            if vec_pos[element] == ",":
                num_space+=1
            
            if num_space == 3:
                break

            if (not (vec_pos[element] == ",") and num_space == 0): 
                tempX += str(vec_pos[element])
                #print(tempX)
            
            if (not (vec_pos[element] == ",") and num_space == 1) :
                tempY += str(vec_pos[element])
                #print(tempY)

            if (not (vec_pos[element] == ",") and num_space == 2) :
                tempZ += str(vec_pos[element])
               # print(tempZ)


        #print(type(tempX))
      #  print('within process string')
       # print(tempX +  "      " + tempY + "       " + tempZ)
      #  print("end of process string")


        self.x = float(tempX)
        self.y = float(tempY)
        self.z = float(tempZ)

    def onAnimateBeginEvent(self, dt):

        # get the current position of the simulation
        self.prev_pos = copy.copy(self.position.position.value) #maybe this should be later??? 

        xCurr = self.prev_pos[0][0]
        yCurr = self.prev_pos[0][1]
        zCurr = self.prev_pos[0][2] # need to verify these are what i think they correspond to


      #  position_to_traverse_to = self.line_list[self.counter]

       # print("strings to process")

       # print(position_to_traverse_to)

     #   self.processStr(position_to_traverse_to) 

        x = repr(xCurr)
        y = repr(yCurr)
        z = repr(zCurr)

       # print("the points")

        print(x  +  "    " + y + "     " + z)


        if self.counter == self.line_counter: # note that im unsure about this one
            self.done_flag == 1


        if (self.done_flag == 1): 
            print("done")
            self.prev_pos[0][0] = self.prev_pos[0][0]
            self.prev_pos[0][1] = self.prev_pos[0][1] 
            self.prev_pos[0][2] = self.prev_pos[0][2]



        if (self.done_flag == 0):


            position_to_traverse_to = self.line_list[self.counter]

          #  print("strings to process")

            #print(position_to_traverse_to)

            self.processStr(position_to_traverse_to) 



            if (xCurr - self.x < 0): # means that you need to move up
                x_mv = self.x - xCurr
           # print(x_mv)
                self.prev_pos[0][0] += x_mv
        
            if not(xCurr - self.x < 0):
                x_mv = xCurr - self.x
             # print(x_mv)
                self.prev_pos[0][0] -= x_mv

            if (yCurr - self.y < 0):
                y_mv = self.y - yCurr
           # print(y_mv)
                self.prev_pos[0][1] += y_mv

            if not(yCurr - self.y < 0):
                y_mv = yCurr - self.y
           # print(y_mv)
                self.prev_pos[0][1] -= y_mv

            if (zCurr - self.z < 0):
                z_mv = self.z - zCurr
          #  print(z_mv)
                self.prev_pos[0][2] += z_mv

            if not(zCurr - self.z < 0):
                z_mv = zCurr - self.z
          #  print(z_mv)
                self.prev_pos[0][2] -= z_mv

            self.counter+=1
        #self.position.position = [d[0]]


        #f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_1.csv", "a") 


       # if self.counter == self.line_counter: # note that im unsure about this one
        #    self.done_flag == 1
        

        

        self.position.position = [self.prev_pos[0]]
        self.prev_pos = self.position.position

   # def onAnimateEndEvent(self, dt):
   #     print(self.x + " " + self.y + " " + self.z)


   # def processStr(self, vector_pos):
  #      num_space = 0 
    #    vec_pos = vector_pos
        
   #     tempX = ""
   #     tempY = "" 
   #     tempZ = "" 

     #   for element in range(0, vec_pos.len()):
    #        if element == " ":
    #            num_space+=1
            
     #       if num_space == 3:
      #          break

      #      if (not (element == " ") and num_space == 0): 
      #          tempX += element
            
      #      if (not (element == " ") and num_space == 1) :
      #          tempY += element

       #     if (not (element == " ") and num_space == 2) :
       #         tempZ += element


        #    self.x = float(tempX)
        #    self.y = float(tempY)
         #   self.z = float(tempZ)




class goToPositionSMALL(Sofa.Core.Controller):
    def __init__(self, x,  y, z, nb_iterations, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('goal')
        self.position = self.stiffNode.getObject('goalMO')
        self.x = x
        self.y = y 
        self.z = z 

        self.x_step = 0
        self.y_step = 0
        self.z_step = 0

        #let 0 be down and 1 be up 
        self.x_status = 0 
        self.y_status = 0
        self.z_status = 0 

        self.nb_iter = nb_iterations
        self.iter = 0 


        self.done_flag = 0 



        f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_3.csv", "w", newline='')
        self.header = ["X", "Y", "Z"]
        self.writer = csv.DictWriter(f,fieldnames=self.header)
        self.writer.writeheader()
        #f.close()
    
    def onAnimateBeginEvent(self, dt):
        d = copy.copy(self.position.position.value)
        #print("here")
        xCurr = d[0][0]
        yCurr = d[0][1]
        zCurr = d[0][2] # need to verify these are what i think they correspond to

        x_curr = repr(xCurr)
        y_curr = repr(yCurr)
        z_curr = repr(zCurr)

        #key object at same 
      #  if (self.done_flag == 1):
       #     d[0][0] = d[0][0]
       #     d[0][1] = d[0][1]
       #     d[0][2] = d[0][2] 

      #  print(x_curr  +  " " + y_curr + " " + z_curr)
        if(self.iter == 0 and self.done_flag != 1): 
            if (xCurr - self.x < 0): # means that you need to move up
                x_mv = self.x - xCurr
                self.x_step = x_mv / self.nb_iter
                self.x_status = 1
                print(x_mv)
                # d[0][0] += x_mv
        
            if not(xCurr - self.x < 0):
                x_mv = xCurr - self.x
                self.x_step = x_mv / self.nb_iter
                self.x_status = 0
                print(x_mv)
                # d[0][0] -= x_mv

            if (yCurr - self.y < 0):
                y_mv = self.y - yCurr
                self.y_step = y_mv / self.nb_iter
                self.y_status = 1
                print(y_mv)
                #  d[0][1] += y_mv

            if not(yCurr - self.y < 0):
                y_mv = yCurr - self.y
                self.y_step = y_mv / self.nb_iter
                self.y_status = 0
                print(y_mv)
                #  d[0][1] -= y_mv

            if (zCurr - self.z < 0):
                z_mv = self.z - zCurr
                self.z_step = z_mv / self.nb_iter
                self.z_status = 1
        
                print(z_mv)
                #  d[0][2] += z_mv

            if not(zCurr - self.z < 0):
                z_mv = zCurr - self.z
                self.z_step = z_mv / self.nb_iter
                self.z_status = 0
                print(z_mv)
            #   d[0][2] -= z_mv

            
       # x_p = repr(self.x_step)
      #  y_p = repr(self.y_step)
       # z_p = repr(self.z_step)

        #print(x_p + " " + y_p + " " + z_p)

        # self.position.position = [d[0]]
        print(self.x_step)
        print(self.y_step)
        print(self.z_step)

        if(self.iter < self.nb_iter and self.iter != 0 and self.done_flag != 0):

            if (self.x_status == 0 and self.y_status == 0 and self.z_status == 0):
                d[0][0] -= self.x_step
                d[0][1] -= self.y_step
                d[0][2] -= self.z_step

            if(self.x_status == 1 and self.y_status == 0 and self.z_status == 0):
                d[0][0] += self.x_step
                d[0][1] -= self.y_step
                d[0][2] -= self.z_step

            if(self.x_status == 1 and self.y_status == 1 and self.z_status == 0):
                d[0][0] += self.x_step
                d[0][1] += self.y_step
                d[0][2] -= self.z_step

            if(self.x_status == 1 and self.y_status == 1 and self.z_status == 1):
                d[0][0] += self.x_step
                d[0][1] += self.y_step
                d[0][2] += self.z_step

            if(self.x_status == 1 and self.y_status == 0 and self.z_status == 1):
                d[0][0] += self.x_step
                d[0][1] -= self.y_step
                d[0][2] += self.z_step

            if(self.x_status == 0 and self.y_status == 1 and self.z_status == 1):
                d[0][0] -= self.x_step
                d[0][1] += self.y_step
                d[0][2] += self.z_step
            
            if(self.x_status == 0 and self.y_status == 1 and self.z_status == 0):
                d[0][0] -= self.x_step
                d[0][1] += self.y_step
                d[0][2] -= self.z_step

            if(self.x_status == 0 and self.y_status == 0 and self.z_status == 1):
                d[0][0] -= self.x_step
                d[0][1] -= self.y_step
                d[0][2] += self.z_step

        self.position.position = [d[0]]


        self.iter = self.iter + 1 


        if (self.iter == self.nb_iter and self.done_flag == 0):
            self.iter = 0
            self.done_flag = 1

        if (self.done_flag == 1):
            self.position.position = [d[0]]

       #now i will adjsut all the coordinates accordingly 


        f = open("C:\PaulsCode_15_Inflate\Paul Simulations\sofa_scene\positions_3.csv", "a") 
        #f = open("C:\Volume\volume.csv", "a") 
       # D = repr(d.tolist())

       # D = repr(d)
      #  print(D)
        vector = d.tolist()
        #print(len(vector))

       # print(vector[0][0])
       # print(vector[0][1])
       # print(vector[0][2])

        x_1=str(vector[0][0])
        y_1=str(vector[0][1])
        z_1=str(vector[0][2])

       # print(x_1  +  " " + y_1 + " " + z_1)
        
        self.writer.writerow({"X": x_1,"Y":y_1,"Z":z_1})

        f.close()




#what will happen if i use 2 time steps?
class goToPositionWithTimeStep(Sofa.Core.Controller):
    def __init__(self, x,  y, z, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('goal')
        self.position = self.stiffNode.getObject('goalMO')
        self.x = x
        self.y = y 
        self.z = z 
    
    def onAnimateBeginEvent(self, dt):
        d = copy.copy(self.position.position.value)

        xCurr = d[0][0]
        yCurr = d[0][1]
        zCurr = d[0][2] # need to verify these are what i think they correspond to

        x = repr(xCurr)
        y = repr(yCurr)
        z = repr(zCurr)

        print(x  +  " " + y + " " + z)

        if (xCurr - self.x < 0 and yCurr - self.y < 0 and zCurr - self.z < 0): # means that you need to move up
            x_mv = self.x - xCurr
            y_mv = self.y - yCurr
            z_mv = self.z - zCurr
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
            #time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]


        if (not(xCurr - self.x) < 0 and yCurr - self.y < 0 and zCurr - self.z < 0):
            x_mv = xCurr - self.x
            y_mv = self.y - yCurr
            z_mv = self.z - zCurr
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
          #  time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]

        if (not(xCurr - self.x < 0) and not(yCurr - self.y < 0) and zCurr - self.z < 0):
            x_mv = xCurr - self.x
            y_mv = yCurr - self.y
            z_mv = self.z - zCurr
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
           # time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]

        if (not(xCurr - self.x < 0) and yCurr - self.y < 0 and not(zCurr - self.z < 0)):
            x_mv = xCurr - self.x
            y_mv = self.y - yCurr
            z_mv = zCurr - self.z
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
          #  time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]

        if (not(xCurr - self.x < 0) and not(yCurr - self.y < 0) and not(zCurr - self.z < 0)):
            x_mv = xCurr - self.x
            y_mv = yCurr - self.y
            z_mv = zCurr - self.z
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
          #  time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]

        if (xCurr - self.x < 0 and not(yCurr - self.y < 0) and not(zCurr - self.z < 0)):
            x_mv = self.x - xCurr
            y_mv = yCurr - self.y
            z_mv = zCurr - self.z
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
       #     time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]

        if (xCurr - self.x < 0 and not(yCurr - self.y < 0) and zCurr - self.z < 0):
            x_mv = self.x - xCurr
            y_mv = yCurr - self.y
            z_mv = self.z - zCurr
            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
     #       time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]

        if (xCurr - self.x < 0 and yCurr - self.y < 0 and not(zCurr - self.z < 0)):
            x_mv = self.x - xCurr
            y_mv = self.y - yCurr
            z_mv = zCurr - self.z

            #print(x_mv)
            dis_step_x = x_mv/2 
            dis_step_y = y_mv/2
            dis_step_z = z_mv/2
            
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
         #   time.sleep(.1)
            d[0][0] += dis_step_x
            d[0][1] += dis_step_y
            d[0][2] += dis_step_z
            self.position.position = [d[0]]
        
        #self.position.position = [d[0]]
