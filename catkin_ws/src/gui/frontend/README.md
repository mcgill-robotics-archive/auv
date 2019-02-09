# McGill Robotics AUV Dockside Companion

## Development server

(Note you must have Angular installed to run the dev server)

Run `ng serve` for a dev server. Navigate to `http://localhost:4200/`. The app will automatically reload if you change any of the source files.

## TODO
### Short-term
- CSS Transitions so the gui elements don't look so choppy when updating
- Unsubscribe on destroy for all elements to prevent memory leak
- Recalculate depth gauge scale dynamically (maybe?)
### Long-term
- Handle mission planning stuff
- Establish a camera feed
- Setup a factory for getting whatever subscribers you want (might be difficult)
- Setup a dynamic reconfigure tool (low-priority)
