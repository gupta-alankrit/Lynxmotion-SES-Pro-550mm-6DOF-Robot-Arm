# Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm

1. Useful links.
   - Documentation: [Here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro/ses-pro-arms/ses-pro-550-6-dof-arm/)
   - SES-PRO Robotic Arm UI software: [Here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro/ses-pro-software/ses-pro-arm-ui/)
2. Joint Limites (deg.):
   - J1: [-180, +180]
   - J2: [- 90, + 90]
   - J3: [-115, +115]
     Though J2 and J3 have different limits, but actually their limits are coupled with each other in the sense that the instantaneous limit of J2 or J3 joint must be computed as per the current position of the other one.
   - J4: [-130, +160]
     Cannot go to -180 and +180 deg. because of the excessive tension appearing in the connection wires.
   - J5: [-180, +105]
     Cannot go to +180 deg. because of the excessive tension appearing in the connection wires.
   - J6
     Cannot test since we don’t have any grippers yet.
3. as
