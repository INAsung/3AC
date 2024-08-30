
# Description for task schedule and following Arduino codes

- ❗ You have to install additional Arduino libraries: **CapacitiveSensor (v 0.5.1)**, **Keypad (v 3.1.1)**
- **You should read each .ino files carefully and change each parameters as you want.** (pinmaps, servo motor's reducer ratio and encoder resolution, training schedules,  etc)

- **Tip for preventing choice bias**: Mice tend to lick the closest spout, so each spouts should be placed equally distant from target position.

- from stage3, I added keypad codes for convenience. (refer to code)
---

|name | explanation |expected period|
|:--:|:--:|:--:|
|stage1 |only central spouts move, give water unconditionally | 1~2 days|
|stage2 | only lateral spouts move, give water when lick is detected (choice should be counterbalanced) | 3 days
|stage3 |both lateral spouts and central spout move. give water when lick is deteced (choice should be counterbalanced) |5~7 days
|stage4|central spout come in and give water unconditionally. then, both lateral spouts and central spout come in and give water when lick is detected|1~2 days
|stage5 (semi-3AC)|easy version of 3AC. Sampling liquid does not change until the mouse makes three consecutive right choices or ten right choices.|3~4 weeks
|stage6 (real 3AC)| now, let's start 3AC task! (sampling liquid will be randomized for each trials) | 1 week
