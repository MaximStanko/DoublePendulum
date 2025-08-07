@echo off
set arg1=%1
echo on && g++ -o "kompilat/%arg1%.exe" %arg1%.cpp -lX11 -lGL -lpthread -lpng -lstdc++fs -std=c++17&& echo compiled && "kompilat/%arg1%.exe"