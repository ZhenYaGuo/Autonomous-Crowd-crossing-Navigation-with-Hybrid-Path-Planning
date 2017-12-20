# Code Folder 

this folder contains the matlab & c code

to run the simulation, first compile the c code using the matlab function
'compileC.m' (install a c compiler first, eg gcc on UNIX).

run the simulation with: simulate() with the optional config file as argument
(eg '../data/config1.conf').

video generation (on unix):
set the save\_frames in the config to 1, then after simulation, use the script
eps2avi.sh in this folder to generate a video in the folder ../videos (run
./eps2avi.sh to see how to use it).


## some conventions

- image coordinates:
 img(y,x), where y is vertcal index, x is horizontal index
 img(1,1) is the upper leftmost point

- data format:
 all internal data is stored in pixels not in meters

