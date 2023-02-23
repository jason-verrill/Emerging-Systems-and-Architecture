THERMOSTAT PROEJCT

Summarize the project and what problem it was solving.
The Thermostat project was a prototype to create a thermostat that was capable of sampling temperature, controlling a heat generator through integrated buttons, updating LED signals for communication, controlling timers, and sending data updates to the server via WiFi. This project was an initial test for a theoretical company to get into the smart thermostat market. Therefore, the WiFi interface was not yet implemented, and instead, the UART interface was used to simulate data being sent to the server.


What did you do particularly well?
The organization and clarity of the code are the best examples in this project. Related code is grouped within respective functions to form a single purpose per function. Repetitive patterns are used throughout the project without redundant code such as using Moore state machine framework, setup functions, and grouped task structures. Adding additional tasks or timers is as simple as changing a single number variable and defining the task or timer within its respective setup function.


Where could you improve?
There is some cleanup and rework that could be done to improve code performance and reduce memory footprint. For example, initially the project searched for the correct sensor register by testing each one within an array. However, now that the exact register is known, this search is unnecessary to perform every time and could be reformatted to just use the correct one. In addition, multiple messages are used via the UART interface if errors occur. These messages are currently statically pre-written and sent as needed. This is taking up extra memory space that isn’t necessary. Making the messages more dynamic and reuse the same memory space as needed would be more efficient with memory use, though would require more complex code. An alternative is to only create a message once it becomes necessary, rather than ahead of time, so that the additional memory is only used if an error actually took place. 


What tools and/or resources are you adding to your support network?
In the future, this thermostat project will make use of the WiFi interface to send updates to a server through the cloud. This will also require using secure communications rather than the current clear text over the serial UART interface. 


What skills from this project will be particularly transferable to other projects and/or course work?
Much of the code makes use of interfaces that can be tailored to suit any project. For example, the UART is simply an interface to send information back and forth including additional programming and could be used as an API to control or edit configuration. The GPIO interface is a general purpose interface that controls physical buttons, LED lights, and external pins for power or data and can be configured to control any machinery and communicate status. Learning how to control these interfaces has taught me the ability to create a vast array of projects and fulfill their requirements.


How did you make this project maintainable, readable, and adaptable?
As stated above, code is organized by related function with easy to read and understand functions and variables. Regarding adaptability, the code functionality can be easily extended or modified to suit the purposes of the project. 

