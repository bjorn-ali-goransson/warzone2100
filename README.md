Compiling / running under WSL
=====

    sudo -i
    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX:PATH=~/wz/install -GNinja -DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE
    cmake --build build --target install
    export DISPLAY=:0
    /root/wz/install/bin/warzone2100

My todos for the game
=====================

* [x] Adjust trackHeight
* [x] Make zooming "softer"
* [x] Make rotation "softer"
* [ ] lock & hide mouse while rotating
* [x] Make a new pulsating effect (BOX_PULSE_SIZE)
* [ ] add fade to title screen
* [x] add fade to game start
* [ ] remove blue box from logo
* [ ] fix "landing" effect
* [ ] fix blinking of start pad X
* [ ] make loading effect smoother
* [ ] make startup images 1080p
* [ ] fix "move unit here" click effect on cursor, replace with lake ripple + 3d version animated effect of cursor
* [ ] is there sparks flying currently? or add it ...
* [ ] remove machine gun fire projectiles. speed should be 100!
* [ ] somewhere it said that the Muzzle flash has too fast .... smk of PIE flag (4000?) to be displayed.
* [x] randomize animation frame on eg. oil derricks
* [x] randomize rotation on radars
* [x] animation interpolation between frames
* [x] highlight units by [stencil testing](https://learnopengl.com/Advanced-OpenGL/Stencil-testing)
* [ ] replace flash 2D blobs on eg power plants with effects (if possible? or improve texture quality)
* [ ] STOP scrolling when mouse is outside window!!!
* [ ] STOP scrolling when mouse is on title bar!!!
* [ ] mouse rotating inverts its origin when too close
* [x] make possible to have totally top-down camera pitch
* [ ] either redraw the positional square on the radar (to accurately convey the camera position) or pitch the actual map
* [ ] what's the meaning of the big X cursor that appears on the sides of the screen?
* [ ] acc / dec of tanks
* [ ] add easing to projectiles
* [ ] check if models can be exploded upon destruction
* [ ] UI refresh?
* [ ] sky refresh; look to tribal trouble with procedural clouds
* [ ] lightposts
* [ ] lights on tanks
* [ ] small radars have "gaps" in the middle of the dish, cyborgs seem to have too
* [ ] smoke trails from bullets
* [ ] muzzle flashes
* [ ] rain looks wierd when rotating?
* [ ] rain stops abruptly
* [ ] oil derrick should stop at the same frame
* [ ] camera position is not saved between games

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

Lightposts / lights / lights on tanks
-------------------------------------



Links
-----

https://gamedev.stackexchange.com/questions/141844/how-to-draw-a-circle-on-the-ground-like-in-warcraft-3

https://github.com/Warzone2100/warzone2100/issues/744

"opengl bullet tracer trails"
