#  Tuning Notes 

### 02/28/2026 Notes
1. Intake 
   2. Added a small triage to the front to channel balls away from the center.  
      3. Issue observed was 3rd ball will stuck when there is already left/right balls in place
      4. Channel 3rd ball to one side helps push either left or right ball to the back instead of pushing in the middle
5. Flickers
   6. Best sequence ( Right, Left, Back)
   7. Right, Left flickers needs tuning, may be let the flickers go up higher
   8. During initialization, right flickers did not rest on lowest point, it needs to be review
9. Shooter
   10. Pending on 2nd motor for flywheel before test/tuning
   11. IMU for AutoFire after execute AutoAIM/AutoShoot(Adjustment)
   12. Sensor fusion: Combine IMU + odometry + AprilTag to avoid single-sensor dependency.
11. Engineering Portfolio
    12. Pictures taken, draw some illistration from robot to target for AutoShooting
13. Offense
    14. LimeLight interruption 
        15. *** LimeLight Team, to see find LimeLight's weakness
        15. Bright LED ring lights
        16. Glossy surfaces reflecting light into camera
        17. Can we have a bright light in the back? 
        18. Can we have a flashing light in the back? 
        19. Will it affect our Aliance's shooting ability? 
        20. *** Some of this might affect judging to mark as un-sportmanship
        17. Fake off
            18. Polycarbonate 
            19. Glossy laminated tags 
            20. Clear lexan 
            21. Metallic field elements 
            22. Reflection can:
            23. Create fake edges 
            24. Break the square outline 
            25. Cause flickering detection
    26. Note on Offense
        27. FTC strongly emphasizes Gracious Professionalism. 
        28. Designing elements primarily to degrade another team’s sensors goes against both the rules and the culture of the program.


Changes Log 03/01/29026
1. Flickers Servo update
   2. Slot1
      3. Up   - slot1.setPosition(0.2); -> slot1.setPosition(0.47);
      4. Down - slot1.setPosition(0.9); -> slot1.setPosition(0.98);
   3. Slot2
      4. UP   - slot2.setPosition(0.65); -> slot2.setPosition(0.58);
      5. Down - slot2.setPosition(0);    -> slot2.setPosition(0.05);
   4. Slot3
      5. Up   - slot3.setPosition(0.65); -> slot3.setPosition(0.63);
      6. Down - slot3.setPosition(0.1);  -> slot3.setPosition(0.12);

