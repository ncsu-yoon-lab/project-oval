The two manuals above are for the brushless motor and ESC from Hobbywing.
Important notes about motors not included:
- The brushless motor uses pwm values of 1000 - 2000 (with 1500 being stop) and the command writeMicroSeconds() with the Teensy
- The servo motor (from Traxxas, not Hobbywing) uses pwm_values of 0 - 180 and the command write() with the Teensy
