import os
from numpy import *
from matplotlib.pyplot import *
from control.matlab import * 
os.system('clear')
print("Escreva os parametros do sistema:")
M1 = input("Massa sobre cada roda do onibus (Massa do onibus em Kg/4): ")
M2 = input("Massa de cada suspensao (em Kg): ")
K1 = input("Constante da mola do sistema de suspensao (N/m): ")
b1 = input("Constante de amortecimento do sistema de suspensao (N.s/m): ")
K2 = input("Constante de mola do sistema roda-pneu (N/m): ")
b2 = input("Constante de amortecimento do sistema roda-pneu (N.s/m): ")

