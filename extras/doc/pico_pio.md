This flow is grok4 generated, after correction and guiding comments. Best result compared to claude/grok/chatGPT/copilot/gemini2.5 and all open source llms available to me like gemma3, deepseek:70b, phi4-reasoning:plus, Hunyuan T1, devstral,...
```mermaid
graph TD
    Main["Main Loop<br>ISR: P<br>X: ?<br>Y: ?<br>OSR: ?"] -->|"pull (block, no autopull); mov x, osr; out null, 9<br>Cycles: 3"| StepStart["Step Loop Start<br>ISR: P<br>X: R<<11 | U<<10 | D<<9 | C<br>Y: ?<br>OSR: R<<2 | U<<1 | D"]
    StepStart -->|"out y, 1 (dir to y)<br>Cycles: 1"| DirBranch["Dir Branch<br>ISR: P<br>X: R<<11 | U<<10 | D<<9 | C<br>Y: D<br>OSR: R<<1 | U"]
    DirBranch -->|"D=1: jmp !y (no jump); set pins, 1; jmp after<br>Cycles: 3"| AfterDir["After Dir<br>ISR: P<br>X: R<<11 | U<<10 | D<<9 | C<br>Y: D<br>OSR: R<<1 | U<br>Dir Pin: 1"]
    DirBranch -->|"D=0: jmp !y (jump); set pins, 0 [delay 1]<br>Cycles: 3"| AfterDir
    AfterDir -->|"out y, 1 (up to y); jmp x-- +1 (dec always); mov pins, x; mov osr, x; jmp pin +2<br>Cycles: 5"| StepBranch["Step Branch<br>ISR: P<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: U<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)<br>Step Pin: LSB(C-1)"]
    StepBranch -->|"Step=0: jmp !pin (no jump); jmp target [delay 6]<br>Cycles: 7"| AfterUpdate["After Update<br>ISR: P<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: U<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)"]
    StepBranch -->|"Step=1: jmp pin (jump)<br>Cycles: 1"| UpdateStart["Update Start<br>ISR: P<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: U<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)"]
    UpdateStart -->|"U=1: mov x, isr; jmp !y (no); mov x, ~isr; jmp x-- +1 (dec always); mov isr, ~x; jmp y-- +2 (no); mov isr, x; mov x, osr<br>Cycles: 8 (mismatch)"| AfterUpdateU1["After Update (U=1)<br>ISR: ~(~P -1) then ~P -1 (overwritten)<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: 0<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)<br>Note: Logic may have issue; cycles mismatch with other paths"]
    UpdateStart -->|"U=0: mov x, isr; jmp !y (yes); jmp x-- +1 (dec always); mov isr, ~x; jmp y-- +2 (yes); mov x, osr<br>Cycles: 6 (mismatch)"| AfterUpdateU0["After Update (U=0)<br>ISR: ~(P -1)<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: -1<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)<br>Note: Logic may have issue; cycles mismatch with other paths"]
    AfterUpdateU1 --> AfterUpdateCombined["After Update Combined<br>ISR: P' (per path)<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: U'<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)<br>Note: Mismatch with Step=0 (8/6 vs 7)"]
    AfterUpdateU0 --> AfterUpdateCombined
    AfterUpdate --> AfterUpdateCombined
    AfterUpdateCombined -->|"out null, 11; mov y, osr; mov osr, x; mov x, isr<br>Cycles: 4"| PeriodStart["Period Loop Start<br>ISR: P'<br>X: P'<br>Y: R<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)"]
    PeriodStart -->|"Loop Body: push (from ISR); mov isr, x; jmp y-- (jump if !=0)<br>Cycles: 3 * (R - 1)"| PeriodStart
    PeriodStart -->|"Final: push (from ISR); mov isr, x; jmp y-- (no jump)<br>Cycles: 3"| AfterPeriod["After Period<br>ISR: P'<br>X: P'<br>Y: 0<br>OSR: R<<11 | U<<10 | D<<9 | (C-1)"]
    AfterPeriod -->|"mov x, osr; out y, 9<br>Cycles: 2"| EndBranch["End Branch<br>ISR: P'<br>X: R<<11 | U<<10 | D<<9 | (C-1)<br>Y: (C-1)<br>OSR: R<<2 | U<<1 | D"]
    EndBranch -->|"(C-1)=0: jmp !y (jump to Main)<br>Cycles: 1 (but +3 from Main to next StepStart if ready)"| Main
    EndBranch -->|"(C-1)!=0: jmp !y (no); jmp Step Loop [delay 2]<br>Cycles: 4"| StepStart
```
