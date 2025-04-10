# Digit Recognition System (ROS + OpenCV)

## 📁 File Overview

| File | Description |
|------|-------------|
| `digit_recognition_service_node.py` | Enable the Digit Recognition Node |
| `digit_recognizer.py` | Digit Recognition Algorithm |
| `navigator_interactive.py` | Usage Example |

> 📂 **Note:** Place the `templates/` folder (with digit images `1.png` ~ `9.png`) in the **same directory** as `digit_recognizer.py`.

---

##  How to Run the  example

### 1. Start the Digit Recognition Node

Open a terminal and run:
```bash
python3 digit_recognition_service_node.py
```

### 2.  open another command terminal and run:
```bash
python3 navigator_interactive.py
```
After the user inputs "a" in the terminal, the program starts the digit recognition service and start to detect digit;
then, after the user pressing the Enter key, the program stops the digit recognition service and outputs the best-recognized digit.

### 3. Format of the returned result when calling the digit recognition node:
For example, if the digit "8" is recognized, the returned string might look something like:
```bash
"The best recognition result is: 8"
```
### 4. After successfully running the navigator_interactive.py example, modify your own code following the example's code to call the digit recognition node.
