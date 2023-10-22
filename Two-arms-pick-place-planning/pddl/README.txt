
## MODO DE EJECUCUION DEL PDDL ##

Se ha generado 4 ficheros de domain.pddl para cada uno de los 4 modos definidos
Y en cada uno de los modos se han generado 6 ficheros de task.pddl, para el numero de piezas a simular (2,4,6,8,9 y 10)

Para ejecutarlos:

Se ha utilizado como planner Fast-Downwrad (https://www.fast-downward.org/HomePage)
Instalado en un PC con Windows 10.
Se ha ido ejecutando con python todos los modos con cada configuracion de piezas.
El comando de ejecucion ha sido: python fast-downward.py domain_file.pddl task_file.pddl --search "astar(blind())"