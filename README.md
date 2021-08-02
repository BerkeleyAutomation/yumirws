# Installation
First, install the `abb_librws` python bindings:
```shell
git clone --recursive https://github.com/mjd3/abb_librws.git
cd abb_librws/python
mkdir build
cd build
cmake .. 
make install
```

Next, install this library:
```shell
git clone https://github.com/BerkeleyAutomation/yumirws.git
cd yumirws
pip install -e .
```

# Example Script
TODO: add