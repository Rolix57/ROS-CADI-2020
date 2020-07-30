# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 09:57:11 2020

@author: broke
"""

import csv
import matplotlib.pyplot as plt

# hacer un diccionario
people_data = dict()
# abrir el archivo csv
with open('TitanicSurvival.csv', mode='r') as csv_file:
    csv_reader = csv.DictReader(csv_file)
    
    # almacenar el contenido del csv en un diccionario
    i = 0
    for row in csv_reader:
        people_data[i] = dict(row)
        i += 1

# variables para guardar el contenido
w_counter = 0
m_counter = 0

# ciclo principal para obtener información
for p in range(len(people_data)):
    # revisa el sexo de la persona
    if people_data[p]['sex'] == 'female':
        # aumento contador de mujeres
        w_counter += 1

# calculo cantidad de hombre
m_counter = len(people_data) - w_counter

# imprimir cantidad de personas
print('men: {}, women: {}'.format(m_counter, w_counter))
# guardo todo en una lista
sizes = [m_counter, w_counter]
# etiquetas
labels_w_m = 'Men', 'Women'

plt.pie(sizes, labels=labels_w_m, autopct='%1.1f%%')
plt.title('Passengers by gender')
plt.savefig('men_vs_women.jpeg', dpi=1200)
plt.show()

# grafica 2
age_list = list()
survived_list = list()
died_list = list()

# ciclo principal para obtener información
for p in range(len(people_data)):
    # revisa el sexo de la persona
    if people_data[p]['age'].isdigit():
        # agregar a la lista de la edad
        age_list.append(float(people_data[p]['age']))
        
        # checo si sobrevivió o no
        if people_data[p]['survived'] == 'yes':
            survived_list.append(float(people_data[p]['age']))
        else:
            died_list.append(float(people_data[p]['age']))
        
        
print('passengers {}'.format(len(age_list)))
print('survived {}'.format(len(survived_list)))
print('died {}'.format(len(died_list)))

# para ordenar
age_list.sort()
survived_list.sort()
died_list.sort()

# definir la topografía
fig, ax = plt.subplots(3, 1, tight_layout=True)

ax[0].hist(age_list, bins=100)
ax[0].set_xlabel('Age (years)')
ax[0].set_ylabel('Frequency')
ax[0].set_title('Passenger frequency')

ax[1].hist(survived_list, bins=100)
ax[1].set_xlabel('Age (years)')
ax[1].set_ylabel('Frequency')
ax[1].set_title('Survivor frequency')

ax[2].hist(died_list, bins=100)
ax[2].set_xlabel('Age (years)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('Fatalities frequency')

plt.savefig('histogram.jpeg', dpi=1200)
plt.show()






































