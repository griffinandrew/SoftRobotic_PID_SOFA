import csv
import time
from simple_pid import PID
import os

v_1 = 0.0
v_2 = 0.0
v_3 = 0.0


aurora_1 = 0.0
aurora_2 = 0.0
aurora_3 = 0.0


def processStr(vector_pos): # this will extract the digits from the the file and set them as the x y z coordinates
    global v_1, v_2, v_3
    num_space = 0 
    vec_pos = vector_pos

        
    tempX = ""
    tempY = "" 
    tempZ = "" 

    for element in range(0, len(vec_pos)):

        if vec_pos[element] == " ":
            num_space+=1
            
        if num_space == 3:
            break

        if vec_pos[element] == "]":
            break

        if (not (vec_pos[element] == " ") and not (vec_pos[element] == "[") 
            and not (vec_pos[element] == "]") and not(vec_pos[element] == '"') and not (vec_pos[element] == ",") and num_space == 0): 
            tempX += str(vec_pos[element])

            
        if (not (vec_pos[element] == " ") and not (vec_pos[element] == "[") and not (vec_pos[element] == "]") and not (vec_pos[element] == ",") and num_space == 1) :
            tempY += str(vec_pos[element])

        if (not (vec_pos[element] == " ") and not (vec_pos[element] == "[") and not (vec_pos[element] == "]") and not (vec_pos[element] == ",") and num_space == 2) :
            tempZ += str(vec_pos[element])

    v_1 = float(tempX)
    v_2 = float(tempY)
    v_3 = float(tempZ)


def processStr2(vector_pos): # this will extract the digits from the the file and set them as the x y z coordinates
    global aurora_1, aurora_2, aurora_3
    num_space = 0 
    vec_pos = vector_pos

    tempX = ""
    tempY = "" 
    tempZ = "" 

    for element in range(0, len(vec_pos)): 

        if vec_pos[element] == ",":
            num_space+=1
            
        if num_space == 3:
            break

        if vec_pos[element] == "]":
            break

        if (not (vec_pos[element] == ",") and not (vec_pos[element] == "[") 
            and not (vec_pos[element] == "]") and not(vec_pos[element] == '"') and not (vec_pos[element] == ",") and num_space == 0): 
            tempX += str(vec_pos[element])

            
        if (not (vec_pos[element] == ",") and not (vec_pos[element] == "[") and not (vec_pos[element] == "]") and not (vec_pos[element] == ",") and num_space == 1) :
            tempY += str(vec_pos[element])


        if (not (vec_pos[element] == ",") and not (vec_pos[element] == "[") and not (vec_pos[element] == "]") and not (vec_pos[element] == ",") and num_space == 2) :
            tempZ += str(vec_pos[element])

    aurora_1 = float(tempX)
    aurora_2 = float(tempY)
    aurora_3 = float(tempZ)



def fileWrite(c_1, c_2,c_3):

        control_1 = c_1
        control_2 = c_2
        control_3 = c_3

        with open("C:/Driver_Positions/convert_pos2vol.csv", "w") as f:
            f.truncate()
            csvwriter = csv.writer(f, delimiter = ",")
            csvwriter.writerow([control_1, control_2, control_3])
        f.close()


def main():
    while(True):
        f = open("C:\Driver_Positions\PositonsOfDriver.csv", "r", newline='')
        position_to_traverse_to = f.readline()
        
        if position_to_traverse_to != "":

            f.close()
            processStr(position_to_traverse_to)
            pid_1 = PID(1, 0.1, 0.05, setpoint=v_1) # this set point needs to be the systems (simlulations) current position
            pid_2 = PID(1, 0.1, 0.05, setpoint=v_2)
            pid_3 = PID(1, 0.1, 0.05, setpoint=v_3)


            f = open("C:\Driver_Positions\Aurora_pos.csv", "r", newline='')
            current_pos = f.readline()
            print("positon to traverse: " + current_pos)

            if current_pos != "" :
                processStr2(current_pos)


                control_1 = pid_1(aurora_1)  
                control_2 = pid_2(aurora_2)
                control_3 = pid_3(aurora_3)
                print(control_1)
                print(control_2)
                print(control_3)
            

                fileWrite(control_1, control_2, control_3)


if __name__ == "__main__":
    main()
