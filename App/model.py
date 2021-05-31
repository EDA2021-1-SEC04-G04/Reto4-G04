"""
 * Copyright 2020, Departamento de sistemas y Computación,
 * Universidad de Los Andes
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
 *
 * Contribuciones:
 *
 * Dario Correal - Version inicial
 """


import config as cf
from DISClib.ADT.graph import gr
from DISClib.ADT import list as lt
from DISClib.ADT import map as m
from DISClib.DataStructures import mapentry as me
from DISClib.Algorithms.Sorting import shellsort as sa
from DISClib.Algorithms.Graphs import scc
from DISClib.Algorithms.Graphs import dijsktra as djk
from DISClib.Utils import error as error
from math import radians, cos, sin, asin, sqrt
import reprlib
assert cf

"""
Se define la estructura de un catálogo de videos. El catálogo tendrá dos listas, una para los videos, otra para las categorias de
los mismos.
"""

#---------------------------------------------------------
#               Construccion de modelos
#---------------------------------------------------------

def newAnalyzer():
    """ Inicializa el analizador

    points: Tabla de hash para guardar los vertices del grafo
    connections: Grafo para representar los cables entre landing points
    components: Almacena la informacion de los componentes conectados
    countries: Tabla de hash para guardar información de los países y sus capitales
    paths: Estructura que almancena los caminos de costo minimo desde un
           vertice determinado a todos los otros vértices del grafo
    relatedVertex: Tabla de hash que toma como llave el codigo de un landing point y 
                   guarda una lista con todos los vertices asociados
    """
    try:
        analyzer = {
                    'points': None,
                    'connections': None,
                    'components': None,
                    'countries': None,
                    'paths': None,
                    'relatedVertex':None,
                    'componentsGraph':None
                    }

        analyzer['points'] = m.newMap(numelements=1400,
                                     maptype='PROBING',
                                     comparefunction=compareStopIds)

        analyzer['connections'] = gr.newGraph(datastructure='ADJ_LIST',
                                              directed=False,
                                              size=14000,
                                              comparefunction=compareStopIds)
        analyzer['countries'] = m.newMap(numelements=300, 
                                     maptype='PROBING',
                                     comparefunction=compareCountries)
        analyzer['relatedVertex'] = m.newMap(numelements=14000,
                                     maptype='PROBING',
                                     comparefunction=compareStopIds)
        analyzer['componentsGraph'] = gr.newGraph(datastructure='ADJ_LIST',
                                                  directed=False,
                                                  size = 14000,
                                                  comparefunction=compareStopIds)
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:newAnalyzer')

#---------------------------------------------------------
#   Funciones para agregar informacion al catalogo
#---------------------------------------------------------

def addLPoint(analyzer, point):
    """
    Adiciona un landing point a la tabla de hash de puntos
    """
    cable_id = point['landing_point_id']
    m.put(analyzer['points'],cable_id,point)
    return analyzer

def addCountry(analyzer, country):
    """
    Adiciona un pais a la tabla de hash countries
    """
    countryName = country['CountryName']
    m.put(analyzer['countries'],countryName,country)
    return analyzer

def addLandConnection(analyzer, connection):
    """
    Adiciona los vértices al grafo con el formato:
    código*nombre_del_cable

    Añade conexiones entre los vértices tomando como peso
    la distancia entre los 2 puntos calculada con la 
    función de haversine

    Guarda los vértices con el mismo código inicial
    en el mapa relatedVertex
    """
    congr = analyzer['connections'] #connections graph = congr
    rela = analyzer['relatedVertex']
    points = analyzer['points']
    try:
        origin = connection['\ufefforigin']
        oriKey = m.get(points,origin)
        oriPoint = me.getValue(oriKey)
        verOrigin = formatVertex(connection,'\ufefforigin')
        gr.insertVertex(congr,verOrigin)
        destination = connection['destination']
        destKey = m.get(points,destination) 
        destPoint = me.getValue(destKey)
        verDest = formatVertex(connection,'destination')
        gr.insertVertex(congr,verDest)
        distance = haversine(oriPoint,destPoint)
        addConnection(congr, verOrigin,verDest , distance)
        esta = m.get(rela,origin)
        if esta is not None:
            lista = me.getValue(esta)
            lt.addLast(lista,verOrigin)
        else:
            listaVertex = lt.newList(datastructure='ARRAY_LIST')
            lt.addLast(listaVertex,verOrigin)
            m.put(rela,origin,listaVertex)
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:addLandConnection')

def addConnection(congr, origin, destination, distance):
    """
    Adiciona un arco entre dos Landing Points
    """
    edge = gr.getEdge(congr, origin, destination)
    if edge is None:
        gr.addEdge(congr, origin, destination, distance)
    return congr

def connectLocalVertex(analyzer):
    '''
    conecta los vertices con un mismo código entre sí
    y con la ciudad capital de su país
    '''
    connectSameCode(analyzer)
    connectSameCapital(analyzer)
    return analyzer

def connectSameCode(analyzer):
    rela = analyzer['relatedVertex']
    congr = analyzer['connections']
    codes = m.keySet(rela)
    size = lt.size(codes)
    for i in range(0,size):
        key = lt.getElement(codes,i)
        entry = m.get(rela,key)
        codeList = me.getValue(entry)
        codeSize = lt.size(codeList)
        if codeSize >1:
            for j in range(0,codeSize):
                if j+1 <= codeSize:
                    vertex = lt.getElement(codeList,j)
                    nextVertex = lt.getElement(codeList,j+1)
                    gr.addEdge(congr,vertex,nextVertex,0.1)
            first = lt.firstElement(codeList)
            last = lt.lastElement(codeList)
            gr.addEdge(congr,last,first,0.1)
    return analyzer

def connectSameCapital(analyzer):
    rela = analyzer['relatedVertex']
    congr = analyzer['connections']
    countries = analyzer['countries']
    points = analyzer['points']
    codes = m.keySet(rela)
    size = lt.size(codes)
    for i in range(0,size):
        key = lt.getElement(codes,i)
        landPoint = me.getValue(m.get(points,key))
        location = landPoint['name']
        city_country = location.split(',')
        if len(city_country) == 1:
            country = city_country[0]
        else:
            country = city_country[1]
        country = country.strip()
        countryEntry = m.get(countries,country)
        if countryEntry is not None:
            countryInfo = me.getValue(countryEntry)
            capital = countryInfo['CapitalName']
            gr.insertVertex(congr,capital)
            cap_lat = countryInfo['CapitalLatitude']
            cap_lon = countryInfo['CapitalLongitude']
            cap = {'latitude':cap_lat,'longitude':cap_lon}
            entry = m.get(rela,key)
            codeList = me.getValue(entry)
            codeSize = lt.size(codeList)
            distance = haversine(cap,landPoint)
            for j in range(0,codeSize):
                vertex = lt.getElement(codeList,j)
                gr.addEdge(congr,capital,vertex,distance) 
    return analyzer

#---------------------------------------------------------
#           Funciones para creacion de datos
#---------------------------------------------------------

def formatVertex(connection,orde):
    """
    Se formatea el nombrer del vertice con el id del landing point
    seguido del nombre del cable
    """
    name = connection[orde] + '*'
    name = name + connection['cable_name']
    return name
    
def haversine(oriPoint,destPoint):
    latOri = float(oriPoint['latitude'])
    lonOri = float(oriPoint['longitude'])
    latDest = float(destPoint['latitude'])
    lonDest = float(destPoint['longitude'])
    
    latOri,lonOri,latDest,lonDest = map(radians,[latOri,lonOri,latDest,lonDest])

    dlon = lonDest - lonOri
    dlat = latDest - latOri
    a = sin(dlat/2)**2 + cos(latOri)*cos(latDest)*sin(dlon/2)**2
    r = 6399.594
    d = 2*r*asin(sqrt(a))
    dist = round(d)
    return dist

#---------------------------------------------------------
# Funciones de consulta
#---------------------------------------------------------

def connectedComponents(analyzer):
    """
    Calcula los componentes conectados del grafo
    Se utiliza el algoritmo de Kosaraju
    """
    analyzer['components'] = scc.KosarajuSCC(analyzer['connections'])
    return scc.connectedComponents(analyzer['components'])

def sameCluster(ana,lanPrim,lanSec):
    id1 = None
    id2 = None
    keys = m.keySet(ana['points'])
    size = lt.size(keys)
    encontrados = False
    i = 0
    while i < size and encontrados==False:
        key = lt.getElement(keys,i)
        info = me.getValue(m.get(ana['points'],key))
        name = info['name']
        city = name.split(',')[0]
        if city == lanPrim:
            id1 = info['landing_point_id']
        elif city == lanSec:
            id2 = info['landing_point_id']
        if id1 != None and id2!=None:
            encontrados = True
        i += 1
    rela = ana['relatedVertex']
    first = lt.firstElement(me.getValue(m.get(rela,id1)))
    second = lt.firstElement(me.getValue(m.get(rela,id2)))
    relation = scc.stronglyConnected(ana['components'],first,second)
    return relation,id1,id2

def greaterDegree(ana):
    rela = ana['relatedVertex']
    points = ana['points']
    mayor = -1
    llaves = []
    keys = m.keySet(rela)
    for i in range(0,lt.size(keys)):
        key = lt.getElement(keys,i)
        lista = me.getValue(m.get(rela,key))
        size = lt.size(lista)
        if size > mayor:
            mayor = size
            llaves = []
            llaves.append(key)
        elif size == mayor:
            llaves.append(key)
    mayores = lt.newList()
    for llave in llaves:
        info = me.getValue(m.get(points,llave))
        location = info['name']
        city_country = location.split(',')
        if len(city_country) == 1:
            country = city_country[0]
        else:
            country = city_country[1]
        name = info['id']
        retorno = name,country,llave
        lt.addLast(mayores,retorno)
    return mayores, mayor
        

def minimumCostPaths(analyzer, initialStation):
    """
    Calcula los caminos de costo mínimo desde la estacion initialStation
    a todos los demas vertices del grafo
    """
    analyzer['paths'] = djk.Dijkstra(analyzer['connections'], initialStation)
    return analyzer


def hasPath(analyzer, destStation):
    """
    Indica si existe un camino desde la estacion inicial a la estación destino
    Se debe ejecutar primero la funcion minimumCostPaths
    """
    return djk.hasPathTo(analyzer['paths'], destStation)


def minimumCostPath(analyzer, destStation):
    """
    Retorna el camino de costo minimo entre la estacion de inicio
    y la estacion destino
    Se debe ejecutar primero la funcion minimumCostPaths
    """
    path = djk.pathTo(analyzer['paths'], destStation)
    return path


def totalLandingPoints(analyzer):
    """
    Retorna el total de estaciones (vertices) del grafo
    """
    return gr.numVertices(analyzer['connections'])


def totalConnections(analyzer):
    """
    Retorna el total arcos del grafo
    """
    return gr.numEdges(analyzer['connections'])


def servedRoutes(analyzer):
    """
    Retorna la estación que sirve a mas rutas.
    Si existen varias rutas con el mismo numero se
    retorna una de ellas
    """
    lstvert = m.keySet(analyzer['stops'])
    maxvert = None
    maxdeg = 0
    for vert in lt.iterator(lstvert):
        lstroutes = m.get(analyzer['stops'], vert)['value']
        degree = lt.size(lstroutes)
        if(degree > maxdeg):
            maxvert = vert
            maxdeg = degree
    return maxvert, maxdeg

#-----------------------------------------------------------------
# Funciones utilizadas para comparar elementos dentro de una lista
#-----------------------------------------------------------------

def compareStopIds(stop, keyvaluestop):
    """
    Compara dos estaciones
    """
    stopcode = keyvaluestop['key']
    if (stop == stopcode):
        return 0
    elif (stop > stopcode):
        return 1
    else:
        return -1


def compareroutes(route1, route2):
    """
    Compara dos rutas
    """
    if (route1 == route2):
        return 0
    elif (route1 > route2):
        return 1
    else:
        return -1

def compareCountries(country1,country2):
    country2 = country2['key']
    if (country1 == country2):
        return 0
    elif (country1 > country2):
        return 1
    else:
        return -1

#---------------------------------------------------------
#               Funciones de ordenamiento
#---------------------------------------------------------

