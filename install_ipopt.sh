#!/bin/sh
set -e

# The following packages are required
# sudo apt-get install liblapack-dev libmetis-dev

# cd to script directory
DIR_LOGIC_OPT="$(cd "$(dirname "$0")" >/dev/null 2>&1; pwd -P)"
echo ${DIR_LOGIC_OPT}
cd "${DIR_LOGIC_OPT}"  # logic-opt

# cd to external directory
mkdir -p external
cd external  # logic-opt/external

# Remove pre-existing folders
rm -rf Ipopt.git

# Clone Ipopt
git clone https://github.com/coin-or/Ipopt.git -b releases/3.13.2 --single-branch Ipopt.git

# Download HSL
cd Ipopt.git  # logic-opt/external/Ipopt.git
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd ThirdParty-HSL  # logic-opt/external/Ipopt.git/ThirdParty-HSL
tar -xzf "${DIR_LOGIC_OPT}"/coinhsl.tgz -C .
mv coinhsl-* coinhsl

# Install HSL
./configure
make
sudo make install

# Compile HSL with Metis
cd ..  # logic-opt/external/Ipopt.git
./configure --prefix /usr/local
make
sudo make install
