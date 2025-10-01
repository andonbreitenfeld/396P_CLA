# 396P_CLA
CLAude course project for ME 396P - Clara Summerford, Luke Pronga, & Andon Breitenfeld

## First Time Setup
Run the following commands to build and run the docker image.

First build the image:
```
docker build -t cla-nav2:latest .
```

Add X11 connections for GUI apps:
```
xhost +local:root
```

Then start the container:
```
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name nav2 \
  cla-nav2:latest

```
