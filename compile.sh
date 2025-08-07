#!/bin/bash
g++ -o "kompilat/$1.exe" $1.cpp -lX11 -lGL -lpthread -lpng -lstdc++fs -std=c++17 && echo "compiled to \"./kompilat/$1.exe\""