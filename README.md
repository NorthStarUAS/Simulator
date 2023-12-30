# Visuals

## FlightGear setup notes

In the FlightGear main Launcher page -> Settings -> Additional
Settings: copy these two lines (each option must be on separate lines)

    --generic=socket,out,100,localhost,6501,udp,aura-gps
    --generic=socket,out,100,localhost,6502,udp,aura-imu

## Panda3d notes

## convert obj to egg and add normals

    obj2egg -nv 30 .\sr22.obj -o .\sr22.egg
