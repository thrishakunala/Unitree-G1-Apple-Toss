UNITREE G1 APPLE TOSS (MuJoCo)
==============================

What this is
  A Unitree G1 humanoid tosses an apple from one hand to the other, back and forth.
  Each throw is planned from projectile motion, the catching arm predicts the landing
  point from the apple's measured state, and overlays show the predicted parabola
  against the path actually flown.

Quickest look
  Open apple_toss_demo.mp4. It is a recording of 12 tosses made with this exact code.

Run it yourself (Windows)
  1. Install Python 3.9 or newer from python.org if you do not have it.
  2. Double-click run_apple_toss.bat.
     It installs the needed packages (mujoco, numpy, matplotlib, imageio, pillow) the
     first time, then opens the interactive viewer. Drag to orbit, scroll to zoom,
     Space pauses, close the window to stop.
  Or from a terminal in this folder:
     pip install mujoco numpy matplotlib imageio imageio-ffmpeg pillow
     python apple_toss.py
     python apple_toss.py --record my_video.mp4 --tosses 12

Files
  apple_toss.py         the whole simulation and controller, one file, commented
  scene_toss.xml        the scene: robot, apple, lighting, cameras, grasp constraints
  g1/                   Unitree G1 model and meshes from MuJoCo Menagerie (see g1/LICENSE)
  apple_toss_demo.mp4   recorded demo
  toss_analysis.png     flown path against the analytic parabola, and landing error per throw
  toss_log.csv          per-toss numbers (release velocity, time of flight, apex, catch offset)
  run_apple_toss.bat    one-click launcher

What is physically simulated, and what is simplified
  * Simulated: full rigid-body dynamics of the robot, torque-limited joint servos on the
    arms, the apple as a free body under gravity, contacts, and the catch as a real
    interception problem (the palm must get within 4 cm of the apple's path or it misses).
  * Simplified: grasping uses a soft weld constraint between palm and apple that is on
    while a hand holds the apple and off at the instant of release. The fingers open and
    close but do not carry the load.
  * Simplified: the pelvis is held by a virtual support stand. There is no balance
    controller, and the robot falls without the stand.

Numbers from the recorded run (12 tosses, all caught)
  Release speed 2.0 to 2.5 m/s, time of flight 0.43 to 0.52 s.
  Measured time of flight and apex height agree with the projectile-motion prediction to
  about 2 ms and 2 mm. The remaining gap is the integrator's step error (2 ms timestep).
