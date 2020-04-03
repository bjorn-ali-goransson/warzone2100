My todos for the game
=====================

* [ ] add fade to title screen
* [ ] remove blue box from logo
* [ ] fix "landing" effect
* [ ] fix blinking of start pad X
* [ ] fix acc / dec of tanks
* [ ] add easing to projectiles
* [ ] check if models can be exploded upon destruction
* [ ] randomize animation frame on eg. oil derricks
* [ ] UI refresh?
* [ ] Replace flash 2D blobs on eg power plants with effects (if possible? or improve texture quality)
* [ ] During arc rotate, multiply the player.r.y = rotInitial ..... by rotational X angle. A lower angle (?) should equal a slower rotation
* [ ] In trackHeight (I guess?), should not make the camera "jump off a cliff" - maybe make it snap as well
* [ ] Add easing to zoom distance
* [ ] Make camera pitching "snap" to certain angles (and add easing)

Add easing to zoom distance
---------------------------

This code change concerns display.cpp. The new variables, at line 116. The added logic, line 1004 in scroll().

First off, DON'T show the scroll value as a console message. :) that info is relegated to the logs.

We will increase the zoom amount of each mousewheel scroll step. It's a bit too weeny as it is now - we can afford to increase the heaviness somewhat.

