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
 """

import config as cf
import model
import csv


"""
El controlador se encarga de mediar entre la vista y el modelo.
"""
#---------------------------------------------------------
#       Inicialización del Catálogo de libros
#---------------------------------------------------------

def init():
    """
    Llama la funcion de inicializacion  del modelo.
    """
    # analyzer es utilizado para interactuar con el modelo
    analyzer = model.newAnalyzer()
    return analyzer
    
#---------------------------------------------------------
#       Funciones para la carga de datos
#---------------------------------------------------------

def loadAnalyzerData(analyzer):
    '''
    Carga toda la información al catálogo
    '''
    loadPoints(analyzer,'landing_points.csv')
    loadConnections(analyzer,'connections.csv')
    loadCountries(analyzer,'countries.csv')
    connectLocalVertex(analyzer)
    return analyzer

def loadPoints(analyzer, servicesfile):
    """
    Carga los datos del archivo Landing Points CSV en el modelo.
    """
    servicesfile = cf.data_dir + servicesfile
    input_file = csv.DictReader(open(servicesfile, encoding="utf-8"),
                                delimiter=",")
    for point in input_file:
        model.addLPoint(analyzer,point)
    return analyzer

def loadConnections(analyzer, servicesfile):
    """
    Carga los datos del archivo de conexiones CSV en el modelo.
    Se crea un arco entre cada punto de origen y destino
    """
    servicesfile = cf.data_dir + servicesfile
    input_file = csv.DictReader(open(servicesfile, encoding="utf-8"),
                                delimiter=",")
    for connection in input_file:
        model.addLandConnection(analyzer,connection)
    return analyzer

def loadCountries(analyzer, servicesfile):
    """
    Carga los datos del archivo Countries CSV en el modelo.
    """
    servicesfile = cf.data_dir + servicesfile
    input_file = csv.DictReader(open(servicesfile, encoding="utf-8"),
                                delimiter=",")
    for country in input_file:
        model.addCountry(analyzer,country)
    return analyzer

def connectLocalVertex(analyzer):
    return model.connectLocalVertex(analyzer)

def loadComponentGraphData(analyzer):
    '''
    Carga toda la información al catálogo
    '''
    loadPoints(analyzer,'landing_points.csv')
    loadConnections(analyzer,'connections.csv')
    loadCountries(analyzer,'countries.csv')
    connectLocalVertex(analyzer)
    return analyzer

#---------------------------------------------------------
#            Funciones de ordenamiento
#---------------------------------------------------------



#---------------------------------------------------------
#        Funciones de consulta sobre el catálogo
#---------------------------------------------------------
    
def totalLandingPoints(analyzer):
    """
    Total de paradas de autobus
    """
    return model.totalLandingPoints(analyzer)


def totalConnections(analyzer):
    """
    Total de enlaces entre las paradas
    """
    return model.totalConnections(analyzer)


def connectedComponents(analyzer):
    """
    Numero de componentes fuertemente conectados
    """
    return model.connectedComponents(analyzer)

def sameCluster(ana,lanPrim,lanSec):
    return model.sameCluster(ana,lanPrim,lanSec)

def greaterDegree(ana):
    return model.greaterDegree(ana)

def minimumCostPaths(analyzer, initialStation):
    """
    Calcula todos los caminos de costo minimo de initialStation a todas
    las otras estaciones del sistema
    """
    return model.minimumCostPaths(analyzer, initialStation)


def hasPath(analyzer, destStation):
    """
    Informa si existe un camino entre initialStation y destStation
    """
    return model.hasPath(analyzer, destStation)


def minimumCostPath(analyzer, destStation):
    """
    Retorna el camino de costo minimo desde initialStation a destStation
    """
    return model.minimumCostPath(analyzer, destStation)


def servedRoutes(analyzer):
    """
    Retorna el camino de costo minimo desde initialStation a destStation
    """
    maxvert, maxdeg = model.servedRoutes(analyzer)
    return maxvert, maxdeg
    