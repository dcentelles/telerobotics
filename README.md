# telerobotics
this library implements some useful protocols for telerobotics. Specially for low-rate links as in the typical underwater environments.

### Compile:

Once cloned this repository. Go inside and run the following:

```bash
$ git submodule init
$ git submodule update --init --recursive
$ mkdir build
$ cd build
$ cmake ../ -DCMAKE_INSTALL_PREFIX=$PREFIX
$ make
```
$PREFIX is the path where *telerobotics* will be installed. Usually /usr/local

### Install:

After build, and once inside the build directory:

```bash
$ make install
```
this will install *telerobotics* in $PREFIX (default prefix in cmake is /usr/local/)

## Create an Eclipse project to improve *telerobotics*

[See Eclipse CD4 Generator](https://cmake.org/Wiki/Eclipse_CDT4_Generator)
