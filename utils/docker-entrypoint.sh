#!/bin/bash
set -e

export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

if [ -z "${OM_DATUM_LAT}" ]; then
  export OM_DATUM_LAT=30.0

  echo "OM_DATUM_LAT not set, using default value: ${OM_DATUM_LAT}"
fi

if [ -z "${OM_DATUM_LONG}" ]; then
  export OM_DATUM_LONG=0.5

  echo "OM_DATUM_LONG not set, using default value: ${OM_DATUM_LONG}"
fi

if [ -z "${OM_MAP_PATH}" ]; then
  export OM_MAP_PATH=${WORKSPACE}/map.json

  echo "OM_MAP_PATH not set, using default value: ${OM_MAP_PATH}"
fi

if [ ! -f "${OM_MAP_PATH}" ]; then
  echo '{"type": "FeatureCollection", "features": []}' > ${OM_MAP_PATH}
  echo "Created empty map file at ${OM_MAP_PATH}"
fi

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
  source ${WORKSPACE}/install/local_setup.bash
fi

# Execute the command passed to the container
exec "$@"
