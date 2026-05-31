# boidz
boidzzz
flap flap flap
## Dipendenze
sudo apt install libsfml-dev cmake
## Comandi
Per creare cartella build:
\texttt{cmake -S . -B build -G"Ninja Multi-Config"} 

Compila in debug:
\texttt{cmake --build build --config Debug}

Compila i test in debug:
\texttt{cmake --build build --config Debug --target test}

per eseguire fai \texttt{./build/Debug/progettoboids} e \texttt{./build/Debug/testboids.t}

Solo a fine progetto:
cmake --build build --config Release
cmake --build build --config Release --target test
