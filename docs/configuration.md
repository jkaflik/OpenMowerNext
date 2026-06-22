---
title: Configuration
---
# {{ $frontmatter.title }}

## Overview

Currently most of the configuration is hardcoded everywhere in the code.
Only a few things are configurable using environment variables.
The goal is to:
- all ROS nodes should be configurable using ROS parameters
- when applicable, parameter should support a hot reload
- all parameter change should be persisted and restored on the next start
- documentation should be available for all parameters

## Environment variables

- `OM_MAP_PATH` - path to the map file (see: [map server](architecture/map-server.md))
- `OM_DATUM_LAT` - latitude of the datum point
- `OM_DATUM_LONG` - longitude of the datum point
