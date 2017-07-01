import os
import time
from numpy import *
import matplotlib.pyplot as plt
from matplotlib.pyplot import *
from control import *
from math import *

os.system('clear')
M1 =2500.0 #input("Massa sobre cada roda do onibus (Massa do onibus em Kg/4): ")
M2 =320.0 #input("Massa de cada suspensao (em Kg): ")
K1 =80000.0 #input("Constante da mola do sistema de suspensao (N/m): ")
b1 =350.0 #input("Constante de amortecimento do sistema de suspensao (N.s/m): ")
K2 =500000.0 #input("Constante de mola do sistema roda-pneu (N/m): ")
b2 =15020.0 #input("Constante de amortecimento do sistema roda-pneu (N.s/m): ")

#System modeling

#G1 = ((M1+M2)*s^2+b2*s+K2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));
#G2 = (-M1*b2*s^3-M1*K2*s^2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));

delta = [M1*M2, (M1*(b1+b2)+M2*b1), (M1*(K1+K2)+b1*b2+M2*K1), (b1*(K1+K2)+K1*(b1+b2)-2*K1*b1), K1*K2]
num1 =[M1+M2,b2,K2]
num2 =[-M1*b2,-M1*K2,0,0]
G1 = tf(num1,delta)
G2 = tf(num2,delta)
print ""
print "Sua funcao de transferencia considerando a variacao de relevo da pista (W) = 0 eh: ", G1
print "Sua funcao de transferencia considerando a forca de Controle (U) = 0 eh: ", G2

#System Analysis

[y, T] = step(G1 , linspace(0,50,10000))
plt.plot(T,y)
plt.show("Fig 1")

[y1, T1] = step(0.1*G2 , linspace(0,50,10000))
plt.plot(T1,y1)
plt.show("Fig 2")


#PID

Kd = 208025
Kp = 832100
Ki = 624075

F = tf(num2,num1)

numCs = [Kd,Kp,Ki]
Cs = tf(numCs,[1,0])
print ""
print "Sua funcao de transferencia de PID eh: ", Cs


closed_loop = F*feedback(F*G1,Cs)
[y2, T2] = step(closed_loop*0.1, linspace(0,5,100))
plt.plot(T2,y2)
plt.show()

zero1 = 1
zero2 = 3
polo1 = 0

Cs = tf([1,zero1+zero2,zero1*zero2],[1,polo1])
polos, zeros = pzmap.pzmap(Cs*G1)
plt.show("Fig 4")

root_locus(Cs*G1,None,-80)
plt.show("Fig 5")

Kd = 2*Kd
Kp = 2*Kp
Ki = 2*Ki
numCs = [Kd,Kp,Ki]

Cs = tf(numCs,[1,0])
closed_loop = F*feedback(F*G1,Cs)
[y3, T3] = step(0.1*closed_loop , linspace(0,5,100))
plt.plot(T3,y3)
axis([0, 5, -0.01, 0.01])
plt.show("Fig 6")


#Root Locus

R = roots(delta)
print ""
print R

rlocus(G1)
z = -log(0.05)/sqrt(pow(pi,2)+pow(log(0.05),2))
print ""
print z
