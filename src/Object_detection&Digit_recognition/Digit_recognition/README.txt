digit_recognition_service_node.py
Enable the Digit Recognition Node

digit_recognizer.py
Digit Recognition Algorithm

navigator_interactive.py
Usage Example

Place the template folder in the same directory as digit_recognizer.py.

To enable the digit recognition node, open a command window and type:
python3 digit_recognition_service_node.py

To run the usage example, open another command window and type:
python3 navigator_interactive.py
After the user inputs "A" in the terminal, the program starts the digit recognition service;
then, after waiting for the user to press the Enter key, the program stops the digit recognition service and outputs the best-recognized digit.

Format of the returned result when calling the digit recognition node:
For example, if the digit "8" is recognized, the returned string might look something like:
"The best recognition result is: 8"

After successfully running the navigator_interactive.py example, modify your own code following the example's code to call the digit recognition node.