"""
 * Copyright 2020, Departamento de sistemas y Computación, Universidad
 * de Los Andes
 *
 *
 * Desarrolado para el curso ISIS1225 - Estructuras de Datos y Algoritmos
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along withthis program.  If not, see <http://www.gnu.org/licenses/>.
 """

import config as cf
import sys
import controller
import threading
from DISClib.ADT import list as lt
from DISClib.ADT import map as m
from DISClib.DataStructures import mapentry as me
assert cf


"""
La vista se encarga de la interacción con el usuario
Presenta el menu de opciones y por cada seleccion
se hace la solicitud al controlador para ejecutar la
operación solicitada
"""

initialLPoint = None

def printMenu():
    print("Bienvenido")
    print("1- Cargar información en el catálogo")
    print("2- Encontrar componentes conectados")
    print("3- Encontrar los Landing Points que sirven como punto de interconexión a más cables en la red")
    print("4- Definir el país base")
    print("5- Calcular el camino más corto entre el país base y otro país")
    print("0- Salir")


def optionOne(cont):
    '''
    Carga la información en el analizador
    '''
    controller.loadAnalyzerData(cont)
    numedges = controller.totalConnections(cont)
    numvertex = controller.totalLandingPoints(cont)
    points = cont['points']
    pointKeys = m.keySet(points)
    numLPoints = lt.size(pointKeys)
    primKey = lt.firstElement(pointKeys)
    primLPoint = me.getValue(m.get(points,primKey))
    primLine = 'El primer Landing Point cargado es {}, identificado con el id {}, de latitud {} y longitud {}'.format\
        (primLPoint['name'],primLPoint['landing_point_id'],primLPoint['latitude'],primLPoint['longitude'])
    countries = cont['countries']
    contKeys = m.keySet(countries)
    numCountries = lt.size(contKeys)
    lastContKey = lt.lastElement(contKeys)
    lastCont = me.getValue(m.get(countries,lastContKey))
    secLine = 'El último país cargado es {}, con una población de {} y su número de usuarios de Internet es de {}'.format\
        (lastCont['CountryName'],lastCont['Population'],lastCont['Internet users'])
    print('Numero de Landing Points '+ str(numLPoints))
    print(primLine)
    print('Numero de paises '+ str(numCountries))
    print(secLine)
    print('Numero de vertices: ' + str(numvertex))
    print('Numero de arcos: ' + str(numedges))
    #print('El limite de recursion actual: ' + str(sys.getrecursionlimit()))


def optionTwo(cont):
    print('El número de componentes conectados es: ' +
          str(controller.connectedComponents(cont)))
    print('Escriba el nombre de los landing points sobre los que quiere saber si se encuentran en el mismo clúster')
    lanPrim = input('Primer Landing Point: ')
    lanPrim = lanPrim.title()
    lanSec = input('Segundo Landing Point: ')
    lanSec = lanSec.title()
    relation = controller.sameCluster(cont,lanPrim,lanSec)
    sino = None
    if relation[0] == False:
        sino = 'no'
    if relation[0] == True:
        sino = 'sí'
    linea = 'Los Landing Points {} identificado con {} y {} con id {} {} están en el mismo clúster'.format(lanPrim,relation[1],lanSec,relation[2],sino)
    print(linea)

def optionThree(cont):
    answer = controller.greaterDegree(cont)
    infoLPoint = answer[0]
    numCables = answer [1]
    size = lt.size(infoLPoint)
    print('Los landing points que sirven como punto de interoncexión a más cables en la red son:')
    for i in range(0,size):
        element = lt.getElement(infoLPoint,i)
        linea = '{} de{} con id {}'.format(element[0],element[1],element[2])
        print(linea)
    print('Con {} cables conectados'.format(numCables))

def optionFour(cont):
    msg = "País base: "
    country = input(msg)
    countries = cont['countries']
    initialLPoint = me.getValue(m.get(countries,country))['CapitalName']
    controller.minimumCostPaths(cont, initialLPoint)

def optionFive(cont):
    country = input('Ingrese el país destino: ')
    countries = cont['countries']
    final = me.getValue(m.get(countries,country))['CapitalName']
    path = controller.minimumCostPath(cont,final)
    inicial = lt.firstElement(path)['vertexA']
    final = lt.lastElement(path)['vertexB']
    print('Para llegar desde {} hasta {} hay que tomar el camino:'.format(inicial,final))
    distTotal = 0
    for i in range(0,lt.size(path)):
        camino = lt.getElement(path,i)
        punto = camino['vertexB']
        distancia = camino['weight']
        distTotal += distancia
        linea = '{}: {}km'.format(punto,distancia)
        print(linea)
    distTotal = round(distTotal,2)
    print('Con una distancia total de {}km'.format(distTotal))

def optionSix(cont, destStation):
    haspath = controller.hasPath(cont, destStation)
    print('Hay camino entre la estación base : ' +
          'y la estación: ' + destStation + ': ')
    print(haspath)
    
def optionSix(cont, destStation):
    path = controller.minimumCostPath(cont, destStation)
    if path is not None:
        pathlen = stack.size(path)
        print('El camino es de longitud: ' + str(pathlen))
        while (not stack.isEmpty(path)):
            stop = stack.pop(path)
            print(stop)
    else:
        print('No hay camino')

def optionSeven(cont):
    maxvert, maxdeg = controller.servedRoutes(cont)
    print('Estación: ' + maxvert + '  Total rutas servidas: '
          + str(maxdeg))

catalog = None

"""
Menu principal
"""
def thread_cycle():
    while True:
        printMenu()
        inputs = input('Seleccione una opción para continuar\n>')

        if int(inputs[0]) == 1:
            print("Cargando información de los archivos ....")
            ana = controller.init()
            optionOne(ana)

        elif int(inputs[0]) == 2:
            optionTwo(ana)

        elif int(inputs[0]) == 3:
            optionThree(ana)

        elif int(inputs[0]) == 4:
            optionFour(ana)                

        elif int(inputs[0]) == 5:
            optionFive(ana)

        elif int(inputs[0]) == 6:
            pass
            
        elif int(inputs[0]) == 7:
            pass

        else:
            sys.exit(0)
    sys.exit(0)

if __name__ == "__main__":
    threading.stack_size(67108864)  # 64MB stack
    sys.setrecursionlimit(2 ** 20)
    thread = threading.Thread(target=thread_cycle)
    thread.start()