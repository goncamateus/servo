import os
from numpy import *
import matplotlib.pyplot as plt
from matplotlib.pyplot import *
from control.matlab import * 
os.system('clear')
M1 =2500.0 #input("Massa sobre cada roda do onibus (Massa do onibus em Kg/4): ")
M2 =320.0 #input("Massa de cada suspensao (em Kg): ")
K1 =80000.0 #input("Constante da mola do sistema de suspensao (N/m): ")
b1 =350.0 #input("Constante de amortecimento do sistema de suspensao (N.s/m): ")
K2 =500000.0 #input("Constante de mola do sistema roda-pneu (N/m): ")
b2 =15020.0 #input("Constante de amortecimento do sistema roda-pneu (N.s/m): ")

#G1 = ((M1+M2)*s^2+b2*s+K2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));
#G2 = (-M1*b2*s^3-M1*K2*s^2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));

delta = [M1*M2, (M1*(b1+b2)+M2*b1), (M1*(K1+K2)+b1*b2+M2*K1), (b1*(K1+K2)+K1*(b1+b2)-2*K1*b1), K1*K2]
G1 = tf([M1+M2,b2,K2],delta)
G2 = tf([-M1*b2,-M1*K2,0,0],delta)
print ""
print "Sua funcao de transferencia considerando a forca Normal (W) = 0 eh: ", G1
print "Sua funcao de transferencia considerando a forca de Controle (U) = 0 eh: ", G2

[y, T] = step(G1 , linspace(0,50,10000))
plt.plot(T,y)
plt.show()

[y, T] = step(0.1*G2 , linspace(0,50,10000))
plt.plot(T,y)
plt.show()