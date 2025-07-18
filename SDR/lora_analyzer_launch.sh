# https://github.com/rpp0/gr-lora

sudo apt update
sudo apt install gnuradio

sudo apt update
sudo apt install liblog4cpp5-dev

sudo apt update
sudo apt install git build-essential autoconf libtool
git clone https://github.com/jgaeddert/liquid-dsp.git
cd liquid-dsp
./bootstrap.sh
./configure
make
sudo make install
sudo ldconfig

cd ..

git clone https://github.com/rpp0/gr-lora.git
cd gr-lora
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig

gnuradio-companion