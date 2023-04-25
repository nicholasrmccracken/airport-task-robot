<div align="center">
  <h2> The Bulldozer Bot: An autonomous task-based robot for the Brutus International Airport. </h2>
  <img src="/media/bulldozer-bot-test.gif?raw=true" width="900px">
</div>

## Introduction
The Bulldozer Bot can automate mundane tasks at the Brutus International Airport such as:
- Printing boarding passes by determining colors and pressing the corresponding button
- Fueling airplanes via one of three specified levers
- Carrying and dropping off luggage
- Stamping passports via lever

## Features
To perform the tasks listed in the introduction, The Bulldozer Bot, pictured below, features:
- A bulldozer blade mechanism with two servo motors providing two degrees of freedom designed to perform each task
- Bump switches on the front and back of the robot to align against walls
- A CdS cell to read color values
- Proportion-based navigation using Proportional, Integral, Derivative (PID)
- Error correction in heading angles, x, and y values using Robot Positioning System (RPS)
- MicroSD card data logging
- Status updates via FEH Proteus Controller screen

<img width="450" alt="Screenshot 2023-04-22 at 5 42 28 PM" src="https://user-images.githubusercontent.com/112589972/233807789-732a9efb-f8a0-4bfb-bd42-9291b819a853.png">

## Installation
To use the code, it must be compiled to an MicroSD card and ran in conjunction with a [FEH Proteus Controller](https://u.osu.edu/fehproteus).
    
## License
Distributed under the MIT License.

## Acknowledgements
 - Austin Bian
 - Rohan Jaiswal
 - Jack Tencza
 - Dr. Ben Grier
 - Spring 2023 FEH TAs

## Credits
 - [ChatGPT](https://chat.openai.com)
 - [FEH Library Guide](https://u.osu.edu/fehproteus/programming-syntax)
 - [FEH Materials](https://eed.osu.edu/node/5928/fundamentals-engineering-honors-feh)
