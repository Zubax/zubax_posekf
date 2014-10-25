#!/bin/bash

FIRST="$1"
SECOND="$2"

if [[ -z "$1" ]] || [[ -z "$2" ]]
then
    echo "$0 <first-imu-topic> <second-imu-topic>"
fi

rqt_plot $FIRST/orientation/x:y:z:w       $SECOND/orientation/x:y:z:w       &
rqt_plot $FIRST/angular_velocity/x:y:z    $SECOND/angular_velocity/x:y:z    &
rqt_plot $FIRST/linear_acceleration/x:y:z $SECOND/linear_acceleration/x:y:z &
