The purpose of this project is to provide a useful tool for lecturing doing board work over a projector.
See [using the board in teaching](https://luthert.web.illinois.edu/blog/posts/776.html) for more on my philosophy behind this kind of tool
and [variable-width Bézier curves](https://luthert.web.illinois.edu/blog/posts/775.html) for an explanation of some of the more mathy code.

This project is in a usable state, but not completed.
To run it

1. Install [Python](https://www.python.org/), [aiohttp](https://docs.aiohttp.org/en/stable/index.html), and [reportlab](https://docs.reportlab.com/install/open_source_installation/) if needed
2. Run `python3 server.py`{.sh} in your terminal of choice
3. Navigate to <http://[::1]:44008> in your browser of choice
4. Use a stylus in the window

A log of your actions will be made in a `.json` file storing newline-delimited JSON as you write and draw.
When the browser page closes a PDF copy of the entire session will be generated as well.

You can also open `index.html` directly in a browser and get the user experience with no server,
but neither JSON log nor PDF will be generated and all work will be lost when navigating away from the page.

# Implementation status

- server.py
    - depends on aiohttp
    - IPv6 loopback only
    - [x] serves code files
    - [x] serves user image files (via a hack)
    - [x] logs any message sent to its websocket
    - [x] refresh-safe: re-loads old logs and continues appending to them
        - [ ] allows configuring what logs it reloads
    - support for multiple editors
        - [x] if using same log, all actions go there
        - [ ] if using same log, actions of one get streamed to other
    - [x] streaming to viewers as well as to editor (not used since first added, so not tested much either)
    - [x] graceful exit on first Ctrl+C (close any open websockets)
    - [ ] launches a browser automatically

- HTML and CSS
    - [x] full-screen canvas
    - [x] floating time
    - [x] slide number
    - [x] pen size indicator
    - serverless mode
        - [x] runs as a file:/// URL
        - [ ] SVG export
        - [ ] single-file bundle
    - [x] lurker in lurk.html mode
        - Note: this is less tested; after creating it I realized I had no immediate use for it and stopped testing as I added additional features.

- boardwork.js
    - [x] connects all canvases, no matter where they are on the page
        - but HUD logic might not work right if not just a single full-screen canvas
    - [x] updates the time
    - [x] one board set per canvas, stack of boards per board set
    - [x] keys sent to one canvas at a time (by making it focusable)
    - [x] works with multiple tools (to the degree the OS does)
    - [x] shift-drag to pan
    - [x] detects pen vs eraser
    - [x] handles leave-and-enter sequences
        - bug? if Shift changes while off screen, pan/draw changes too
    - [x] accepts any logged commands that come over websocket as if artist entered them (barely tested)
    - Adding images
        - [x] basic image from URL that fills scree
        - [ ] other sizing and placement options
        - [ ] paste image data (problem getting the events)
        - [ ] convert SVG to something PDF can handle
    - Render quality:
        - [x] dots on each action
        - [x] lines between dots live
        - [x] spline between dots live
        - [x] post-processing hook
        - [x] lines between dots post-hoc
        - [x] splines between dots post-hoc
    - Stroke optimization:
        - [ ] prune extraneous samples on slow/smooth/straight strokes
            - I had this implemented in `downres`, but it was not behaving predictably so I disabled it
            - The `class VWStroke` is an unfinished second attempt
        - [x] spline between xyr points
        - [x] round to scale-appropriate number of decimal digits for smaller JSON files
        - [x] remove bumps in slow strokes
            - note: these are from input device resolution, not from uneven hand nor rounding
        - [x] remove duplicate, overlapping points
        - [ ] connect Bezier spline edges into one Path2D per stroke
            - The `class VWStroke` is an unfinished attempt
    - [x] Workaround for chrome bug
        - For some reason, Chrome sometimes inserts a 0-pressure point in the middle of an otherwise correct point sequence. To prevent this messing up the drawing, motions with zero pressure are ignored
    - [x] streams non-loggable actions to server
    - [x] design to support both lurkers and actors

- pdfmaker.py
    - depends on reportlab
    - [x] renders slide deck to PDF
    - [x] same zoom, pan, colors, etc, as last state of log
    - [x] connect Bezier spline edges into one path per stroke
    - [x] automatic execution when websocket disconnects
    - [ ] cull parts under layers
    - [ ] cull erase with nothing under it
    - [ ] merge regions of same color
    - [ ] option for zoom-to-fit instead of last zoom and pan
    - [x] images render properly
        - Caveat: using `urllib` to download images sometimes gives 403 errors
        - Caveat: only rasters (jpg, png, etc) work, not vectors (svg)

- [x] make keyboard shortcuts configurable
    - if `web/keys.txt` exists, it is used; else default is present in js file
- [ ] allow PDF, HTML import
    - HTML will be tricky for pdfmaker.py; PDF will be tricky for web UI
- [ ] augment logs with timing, audio, transcript
- [ ] have help pop-up for live reminders of keyboard controls
- [ ] have dialog keys to pop up color pickers, jump-to-board, overview mode

# Keyboard shortcuts:

Keyboard shortcuts are provided in a simple line-based text file.
Each line starts with the key pressed,
as defined by [KeyboardEvent.key](https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/key).
This may be preceded by `Shift+`, `Ctrl+`, `Alt+`, and/or `Meta+` for modified keys.
Note that `Shift+` should not be used for printable characters like `F` or `$`.
After the key is a space, a command name, and any arguments separated by spaces.
To have a key do several things, give it several lines.

Known commands and arguments:

| Command | Arguments |
|---------|-----------|
| color   | any value CSS color string, like `#fff`, `#123456`, `black`, etc |
| toggle_hud | toggles visibility of informative overlay sch as the current time |
| invert_colors | toggles between light and dark mode. Note this is visual only: generated PDFs are alwasy in light mode to facilitate printing. |
| softness | an exponent to apply to pressure. Small values like 0.5 will make a marker-like eixperience where the pen goes from nothing to fully-on quickly; large values like 2 will make a brush-like experience where small changes in pressure have large impacts in visual width. |
| pensize | a multiplier to the current pensize; or the keyword `reset` to resume initial size |
| zoom | a multiplier to the current viewport size; or the keyword `reset` to resume initial size. Numbers > 1 make the viewport bigger and thus the contents smaller, often called zooming out. |
| pan | two numbers, dx and dy, modifying the center of the view by that offset; or the keyword `reset` to resume initial centering. |
| insert | `back` or `forward` to indicate if the new blank board (which will become visible) should be placed behind or in front of the previous board. |
| clone | `back` or `forward` to indicate if the new visible copy of the current board should be placed behind or in front of the previous board. |
| go | `back` to back up one board; `forward` to advance one board; `start` to go to the first board; or `end` to go to the last board |
| putimage | none (shows pop-up when pressed) |


The default keys are:

```txt
` color #000
1 color #f00
2 color #33f
3 color #080
4 color #80f
5 color #840
6 color #f90
7 color #9f0
8 color #fae
9 color #ee0
0 color #fff
default color #000

t toggle_hud
i invert_colors

b softness 1
n softness 2
m softness 0.5
default softness 1

p putimage

_ zoom 1.2599210498948732
+ zoom 0.7937005259840998

- pensize 0.7937005259840998
= pensize 1.2599210498948732
default pensize 3

Ctrl+Home pensize reset
Ctrl+Home zoom reset
Ctrl+Home pan reset
Home zoom reset
Home pan reset

Alt+PageUp clone back
Shift+PageUp insert back
PageUp go back
Alt+PageDown clone forward
Shift+PageDown insert forward
PageDown go forward
```
