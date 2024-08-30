
# Three-alternative forced choice task (3AC)

### *Low cost implementation of three-alternative forced choice task with Arduino mega 2560 &amp; 3D printing*

![KakaoTalk_20240830_165530499](https://github.com/user-attachments/assets/3024c292-0f9f-4de6-937f-36c966796199)

---
### *What is 3AC?*
![그림1](https://github.com/user-attachments/assets/8388e439-3c4d-453c-91f6-efe0cd8e4f30)

- 3AC is extended version of conventional two-alternative forced choice task. 
- 3AC task comprises two phases:
	- i) blind tasting of either a pure tastant or mixture (e.g., sweet, salty, or sweet-salty mixture)
	- ii) selection of one of the three bottles corresponding to the tasted solution.

- **A servo motor** switches the tastant solution provided to the mouse for each trial. The liquids are never mixed because they flow through separate lines.
- **Linear actuators** move back and forth, allowing the mouse access to the lick spout for a limited time period.
- **DIY capacitive lickometer** detects and records licks.

- *This is the **first attempt** to match three choices to the three decision spouts.*
---

### *This repository includes:*
- Whole Arduino codes and 3D printing schematics for implementing 3AC behavioral apparatus
- Imaged guideline for making DIY capacitive sensor
- You can use this project for any task that requires the mouse to identify one of three possible options. (though, I developed it for a study on *taste mixture identification*.
