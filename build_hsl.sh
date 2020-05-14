#!/bin/sh
set -e

# cd to external directory
cd $(dirname "$0")
mkdir -p external

# Remove pre-existing folders
rm -rf external/Ipopt.git/ThirdParty/HSL/coinhsl

# Extract HSL and move to Ipopt folder
tar -xzf coinhsl.tgz -C external
cd external
mkdir -p Ipopt.git/ThirdParty/HSL
mv coinhsl-* Ipopt.git/ThirdParty/HSL/coinhsl
cd Ipopt.git/ThirdParty/HSL/coinhsl

# Download and extract Metis
wget http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/OLD/metis-4.0.3.tar.gz
tar -xzf metis-4.0.3.tar.gz

# Compile HSL with Metis
./configure
make
# make install
