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

#G1 = ((M1+M2)*s^2+b2*s+K2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));
#G2 = (-M1*b2*s^3-M1*K2*s^2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));

delta = [M1*M2,(M1*(b1+b2) + M2*b1),(M1*(K1+K2) + b1*(b1+b2) + M2*K1 - b1),(b1*(K1+K2) + K1*(b1+b2) - 2*K1*b1),K1*K2]
G1 = tf([0,0,M1+M2,b2,K2],delta)
G2 = tf([0,-M1*b2,-M1*K2,0,0],delta)
print ""
print "Sua funcao de transferencia considerando a forca Normal (W) = 0 eh: ", G1
print "Sua funcao de transferencia considerando a forca de Controle (U) = 0 eh: ", G2

