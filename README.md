============================
Quick and Dirty Instructions
============================

Add odefl.p to Lightwave and 3 plug-ins should install: ODEGeneric and ODEDisplace and a panel for ODEDisplace.  I like to map the generic to alt+o, but that's just me.


Release Notes
=============
ODEfL v0.4.4 04/27/08
- Plug-in now named odefl.p
- Removed debug info from public build
- Fixed bug in gravity option "Disable until Collision" which caused objects to activate when other objects were "close" but had not actually collided.


ODEfL v0.4.3 04/07/08
- Added enable/disable setting for the ground plane
- Added gravity options for ode objects


ODEfL v0.4.2 03/30/08
- Added automeshing for polys with more than 3 points


ODEfL v0.4.1 03/18/08
- fixed bug causing sims without trimeshes to crash on last frame


ODEfL v0.4.0 03/16/08
- added trimeshes
- BUG: need to fix segfault causing Lightwave to crash
- Caveat: trimeshes cannot be parented to other objects
- Need to compile for LW7 and LW8 and Mac


ODEfL v0.3.4 03/06/08
- compiled for LW7


ODEfL v0.3.3 01/13/07
- Added Infinite Friction control
- Added ODEfL logo to the generic and diplace panels
- Bug fixes


ODEfL v0.3.2 10/13/06
- Bug fixes


ODEfL v0.3.1 10/11/06
- Added Step Size Multiplier
- Fixed bug with composite bodies that crashed Lightwave after 1 or 2 sim runs.


ODEfL v0.3.0 10/08/06
- Keyed Forces
	- To me this is HUGE,  It's the point I feel ODEfL is finally REALLY useful, not just at neat toy.
	- Keyed bodies are disabled until the last key and have no interaction with the simulation until then.
	- Rotation is not keyable... yet!
	- Keys must be added after the ODEfL plug is added, keys added prior to the plugin are ignored.  I hope to fix this.


ODEfL v0.2.2-v0.2.4: 10/04/06
- Gravity now user definable with XYZ components
- Added material properties
	- bounciness
	- bounce velocity
	- density
	- cfm (softness)
	- erp (softness)
	- mu (sliding friction)
- Added two new primitives
	- cylinders
	- capsule
	(note: cylinder-capsule interaction not posible and creation for both default to length along the Y axis)


ODEfL v0.2.1: 09/17/06
- Added code to make spheres to be added to composite bodies
- Set default gravity to -9.8 (was -2.0)
- add bounciness to the default
- renamed ODE to ODEfl, next verison will require re-adding plug-ins.


ODE v0.2.0: 09/14/06

- Plug-in compiled for Lightwave 9.0 (Win) with ODE 0.7 (may work with older versions of lightwave, but not tested)
- Composite Bodies now functional with boxes!  Try loading the table scene file and unparenting one of the legs.  Max 10 children (kinda arbitrary).
- There is a memory bug that pops up with composite bodies on second or third run calculations, I'll spend the next few days tracking it down.

Next up:

- Composites for all types
- Add capped cylinder and capsule types
- Forces and fields
- Joints
- Make it purdy


ODE v0.1.2 2004

I've made a few updates to ODE...

- I fixed a stupid memory issue that crashed Lightwave whenever you cleared or loaded a scene.
- Static objects (non-static is the default).  Just keyframe an object, click static, and it won't budge.
- Saving! Your settings now save with the scene file.  No more tedious clicking.


If you're interested in the source just email me.


Cheers,


Kevin

kevinmacphail@yahoo.com
