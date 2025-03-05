# STACK-A-BOT
STACK-A-BOT: Smart Tech And Compact Knowledge A Better Optimization Tool.
----
This repository contains source code for the Capstone Project at WPI. 

## How to use?
```
git clone https://github.com/harshal-14/STACK-A-BOT.git
```

```
cd STACK-A-BOT
```

# Build the image
```
sudo docker build -t ros2-pybullet-env .
```

# Run with X11 forwarding
```	
sudo docker run --net=host --rm -it \
    --env DISPLAY=:1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    ros2-pybullet-env
```
    
This will take around 12-15 mins to build. Patience is the key!! Alternatively, pull the docker image from this.(Coming soon)


# Style Guide

Google offers a great style guide for python code...

Please use their [docustring](https://google.github.io/styleguide/pyguide.html#383-functions-and-methods) format for commenting functions when possible. 


    
