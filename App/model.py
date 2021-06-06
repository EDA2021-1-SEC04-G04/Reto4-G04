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


from DISClib.DataStructures.chaininghashtable import keySet
import config as cf
from DISClib.ADT.graph import gr
from DISClib.ADT import list as lt
from DISClib.ADT import map as m
from DISClib.DataStructures import mapentry as me
from DISClib.Algorithms.Sorting import shellsort as sa
from DISClib.Algorithms.Graphs import scc
from DISClib.Algorithms.Graphs import dijsktra as djk
from DISClib.Algorithms.Graphs import prim as prim
from DISClib.Algorithms.Sorting import mergesort as mrg
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
                    'componentsGraph':None,
                    'cables':None
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
        analyzer['cables'] = m.newMap(numelements=14000,
                                    maptype='PROBING',
                                    comparefunction=compareCountries)
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
        origin = connection['origin']
        oriPoint = me.getValue(m.get(points,origin))
        verOrigin = formatVertex(connection,'origin')
        addVertex(congr,verOrigin)
        destination = connection['destination']
        destPoint = me.getValue(m.get(points,destination))
        verDest = formatVertex(connection,'destination')
        addVertex(congr,verDest)
        distance = haversine(oriPoint,destPoint)
        addConnection(congr, verOrigin,verDest , distance)
        esta = m.get(rela,origin)
        if esta is not None:
            lista = me.getValue(esta)
            if lt.isPresent(lista,verOrigin)==False:
                lt.addLast(lista,verOrigin)
        else:
            listaVertex = lt.newList(datastructure='ARRAY_LIST')
            lt.addLast(listaVertex,verOrigin)
            m.put(rela,origin,listaVertex)
        return analyzer
    except Exception as exp:
        error.reraise(exp, 'model:addLandConnection')
'''
def test(ana):
    congr = ana['connections']
    rela = ana['relatedVertex']
    edge = gr.getEdge(congr,'3347*America Movil Submarine Cable System-1 (AMX-1)','5693*America Movil Submarine Cable System-1 (AMX-1)')
    print(edge)
    l = me.getValue(m.get(rela,'3347'))
    print(l)
    ady = gr.adjacents(congr,lt.getElement(l,0))
    print(ady)
'''

def addVertex(congr,vertex):
    vert = gr.containsVertex(congr,vertex)
    if vert == False:
        gr.insertVertex(congr,vertex)

def addConnection(congr, origin, destination, distance):
    """
    Adiciona un arco entre dos Landing Points
    """
    edge = gr.getEdge(congr, origin, destination)
    if edge is None:
        gr.addEdge(congr, origin, destination, distance)

def addCable(ana,connection):
    cables = ana['cables']
    cable = connection['cable_name']
    if m.contains(cables,cable) == False:
        m.put(cables,cable,connection['capacityTBPS'])

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
                    addConnection(congr,vertex,nextVertex,0.1)
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
        country = location.split(',')[-1]
        country = country.strip()
        countryEntry = m.get(countries,country)
        if countryEntry is not None:
            countryInfo = me.getValue(countryEntry)
            capital = countryInfo['CapitalName']
            if gr.containsVertex(congr,capital)==False:
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
                addConnection(congr,capital,vertex,distance) 
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
    vertices = gr.vertices(ana['connections'])
    countries = ana['countries']
    keysCountries = m.keySet(countries)
    mayor = -1
    llaves = []
    size = lt.size(vertices)
    for i in range(0,size):
        vertex = lt.getElement(vertices,i)
        degree = gr.degree(ana['connections'],vertex)
        if degree>mayor:
            mayor = degree
            llaves= []
            llaves.append(vertex)
        elif degree == mayor:
            llaves.append(vertex)
    mayores = lt.newList()
    for llave in llaves:
        encontrado = False
        i=0
        while i<lt.size(keysCountries) and encontrado == False:
            country = lt.getElement(keysCountries,i)
            info = me.getValue(m.get(countries,country))
            if info['CapitalName'] == llave:
                encontrado = True
            i+=1
        retorno = llave,country
        lt.addLast(mayores,retorno)
    return mayores, mayor

def minimumCostPaths(analyzer, initialCapital):
    """
    Calcula los caminos de costo mínimo desde la capital del país seleccionado
    a todos los demas vertices del grafo
    """
    analyzer['paths'] = djk.Dijkstra(analyzer['connections'], initialCapital)
    return analyzer

def minimumCostPath(analyzer, destCapital):
    """
    Retorna el camino de costo minimo entre la capital de inicio
    y la capital destino
    Se debe ejecutar primero la funcion minimumCostPaths
    """
    path = djk.pathTo(analyzer['paths'], destCapital)
    return path

r = reprlib.Repr()
r.maxlist = 20
r.maxstring = 20
r.maxlevel = 20

def minimumSpanningTree(ana):
    search = prim.PrimMST(ana['connections'])
    relaxed = prim.prim(ana['connections'],search,'Bogota')
    edges = prim.edgesMST(ana['connections'],search)
    mst = relaxed['mst']
    size = lt.size(mst)
    '''
    for i in range(0,size):
        element = lt.getElement(mst,i)
        print(element['phase'])
    '''
    weight = round(prim.weightMST(ana['connections'],relaxed),2)
    return weight,size

def impact(ana,vertex):
    rela = ana['relatedVertex']
    points = ana['points']
    congr = ana['connections']
    pointKeys = m.keySet(points)
    j = 0
    encontrado = False
    while j<lt.size(pointKeys) and encontrado == False:
        key = lt.getElement(pointKeys,j)
        LPoint = me.getValue(m.get(points,key))
        city = LPoint['name'].split(',')[0]
        if vertex == city:
            encontrado = True
            id = LPoint['landing_point_id']
        j+=1
    vertices = me.getValue(m.get(rela,id))
    countries = lt.newList(cmpfunction=compareDistance)
    llaves = []
    for i in range(0,lt.size(vertices)):
        vert = lt.getElement(vertices,i)
        adyList = gr.adjacents(congr,vert)
        for k in range(0,lt.size(adyList)):
            ady = lt.getElement(adyList,k)
            initial = ady.split('*')[0]
            try:
                code = int(initial)
                lPoint = me.getValue(m.get(points,str(code)))
                country = lPoint['name'].split(',')[-1]
                country = country.strip()
                distance = gr.getEdge(congr,vert,ady)['weight']
                info = country,distance
                if country not in llaves:
                    llaves.append(country)
                    lt.addLast(countries,info)
            except:
                pass
            
    sorted = sortCountries(countries)
    return sorted

def cable(ana,country,cable):
    countries = ana['countries']
    congr = ana['connections']
    cables = ana['cables']
    rela = ana['relatedVertex']
    band = lt.newList(datastructure='ARRAY_LIST')
    countryKeys = m.keySet(countries)
    j = 0
    found = False
    while j<lt.size(countryKeys) and found == False:
        key = lt.getElement(countryKeys,j)
        info = me.getValue(m.get(countries,key))
        Country = info['CountryName']
        if country == Country:
            found = True
        j+=1
    city = info['CapitalName']
    tbps = float(me.getValue(m.get(cables,cable)))*1000000
    adjacents = gr.adjacents(congr,city)
    for i in range(0,lt.size(adjacents)):
        adj = lt.getElement(adjacents,i)
        cableName = adj.split('*')[1]
        id = int(adj.split('*')[0])
        if cable == cableName:
            vertAdj = gr.adjacents(congr,adj)
            for k in range(0,lt.size(vertAdj)):
                vert = lt.getElement(vertAdj,k)
                vertID = vert.split('*')[0]
                vertId = int(vertID)
                if id != vertId:
                    otherEndAdj = gr.adjacents(congr,vert)
                    for l in range(0,lt.size(otherEndAdj)):
                        side = lt.getElement(otherEndAdj,l)
                        sideAdj = gr.adjacents(congr,side)
                        size = lt.size(sideAdj)
                        for num in range(0,size):
                            sideVert = lt.getElement(sideAdj,num+1)
                            sideVertCode = sideVert.split('*')
                            try:
                                cableName2 = sideVertCode[1]
                                if cable == cableName2:
                                    adjacents2 = gr.adjacents(congr,sideVert)
                                    for nu in range(0,lt.size(adjacents2)):
                                        cap = lt.getElement(adjacents2,nu+1).split('*')
                                        try:
                                            int(cap[0])
                                        except:
                                            cap = cap[0]
                                            if cap != 'Havana':
                                                n = 0
                                                found = False
                                                while n < lt.size(countryKeys):
                                                    key = lt.getElement(countryKeys,n)
                                                    info = me.getValue(m.get(countries,key))
                                                    Capital = info['CapitalName']
                                                    if cap == Capital:
                                                        found = True
                                                        country = info['CountryName']
                                                        users = float(info['Internet users'])
                                                        capacity = round(tbps/users,3)
                                                        save = country,capacity
                                                        if lt.isPresent(band,save)==False:
                                                            lt.addLast(band,save)
                                                    n+=1

                            except:
                                capital = sideVertCode[0]
                                if capital != 'Havana':
                                    n = 0
                                    found = False
                                    while n < lt.size(countryKeys):
                                        key = lt.getElement(countryKeys,n)
                                        info = me.getValue(m.get(countries,key))
                                        Capital = info['CapitalName']
                                        if capital == Capital:
                                            found = True
                                            country = info['CountryName']
                                            users = float(info['Internet users'])
                                            capacity = round(tbps/users,3)
                                            save = country,capacity
                                            if lt.isPresent(band,save)==False:
                                                lt.addLast(band,save)
                                        n+=1
                                    
    return band

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

def compareDistance(country1,country2):
    return country1[1] > country2[1]

#---------------------------------------------------------
#               Funciones de ordenamiento
#---------------------------------------------------------

def sortCountries(countries):
    size = lt.size(countries)
    sortedlist = lt.subList(countries, 1, size)
    sublist = mrg.sort(sortedlist, compareDistance)
    return sublist