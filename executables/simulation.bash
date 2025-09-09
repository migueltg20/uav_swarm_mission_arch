#!/bin/bash

ros2 daemon start

source /opt/ros/humble/setup.bash

# Moverse al directorio especificado
cd ~/simulaciones/TB2_Panel_Inspection_Simulation || {
    echo "Error: No se pudo cambiar al directorio especificado.";
    exit 1;
}

# Ejecutar el comando
./launch_as2.bash -n 4 -p 5 -r 4
