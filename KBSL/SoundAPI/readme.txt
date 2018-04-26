 
Install SFML:

sudo apt-get install libsfml-dev


compile/run:

g++ -c sound.cpp
g++ sound.o -o app_name -lsfml-audio
./app_name