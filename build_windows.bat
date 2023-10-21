rem This script requires that the Emscripten SDK has been set up in the directory above this one.
rem Follow the instructions here: https://emscripten.org/docs/getting_started/downloads.html

call python src/parse_mjxmacro.py
rmdir /s /q build
call ../emsdk/emsdk activate latest
mkdir build
cd build
call emcmake cmake ..
call emmake make
cd ../dist
pause