Compiling / running under WSL
=====

To prepare build (?) :

    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX:PATH=~/wz/install -GNinja -DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE
(last param seems to be required on WSL)

To compile:

    cmake --build build --target install

Start Xming and do:

    export DISPLAY=:0

To run:

    /root/wz/install/bin/warzone2100

My todos for the game
=====================

* [x] Adjust trackHeight
* [x] Make zooming "softer"
* [x] Make rotation "softer"
* [x] Turn off the "pulsating" of the selection box. (BOX_PULSE_SIZE)
* [ ] add fade to title screen
* [ ] remove blue box from logo
* [ ] fix "landing" effect
* [ ] fix blinking of start pad X
* [ ] make loading effect smoother
* [ ] make startup images 1080p
* [ ] randomize animation frame on eg. oil derricks
* [ ] highlight units by [stencil testing](https://learnopengl.com/Advanced-OpenGL/Stencil-testing)
* [ ] replace flash 2D blobs on eg power plants with effects (if possible? or improve texture quality)
* [ ] STOP scrolling when mouse is outside window!!!
* [ ] what's the meaning of the big X cursor that appears on the sides of the screen?
* [ ] acc / dec of tanks
* [ ] add easing to projectiles
* [ ] check if models can be exploded upon destruction
* [ ] UI refresh?

Make startup images 1080p
-------------------------

Hm ... Tried changing the backdrops but they don't update in-game ...

Make loading effect smoother
----------------------------

Problem: Since the loading blocks the main thread, we might not be able to update more smoothly before first refactoring the loading screen initialization logic. BTW, is the logic and the rendering synched? Isn't that bad practice? ... TBC ...

Add easing to zoom distance
---------------------------

zoom()

Cubic easing https://easings.net/en#easeInCubic

Adjust trackHeight
------------------

Cubic easing https://easings.net/en#easeInCubic

Don't consider terrain outside the FOW.

Randomize animation frame on eg. oil derricks
---------------------------------------------

Maybe something to do with `psDroid->sDisplay.frameNumber`
