This repository is a playground containing my work on gis tools.

Structure
=========

This repository grew in an ad-hoc fashion and it's structure is likely to change.
Here is the description of the current structure.

* visualization : Web application used to vizualize shapefile/geojson data on a Map.
* spat : Various python 2.7 scripts for analysis, notably map matching.
* gis : some low-level io utility in c++, mostly abandonned.
* data : This folder is not being tracked, but some tools assume it exists.


Dependancies
============

Here's a list of python depandancies. All of them can be installed with pip.

* [networkx](https://networkx.github.io/documentation/development/install.html)
* [numpy/scipy](http://scipy.org/install.html)
* [Shapely](https://pypi.python.org/pypi/Shapely)
* [pyproj](https://pypi.python.org/pypi/pyproj?)
* [pyshp](https://pypi.python.org/pypi/pyshp)
* [geojson](https://pypi.python.org/pypi/geojson)
* [Rtree](https://pypi.python.org/pypi/Rtree/), requires [libspatialindex](http://libspatialindex.github.io/)

Usage
=====

Here's a list of useful standalone python scripts. Help is provided for more info.

* spat.trajectory.preprocess
* spat.trajectory.mapmatch
* spat.trajectory.cluster
* spat.trajectory.features
* spat.geobase.preprocess 
* spat.osm.preprocess